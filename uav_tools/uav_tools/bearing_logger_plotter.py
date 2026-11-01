#!/usr/bin/env python3
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""
bearing_logger_plotter.py.

=========================
Nodo ROS 2 que graba los datos de bearing de TODOS los pares
(observador→target) en tiempo real y los guarda en un archivo .mat
compatible con MATLAB (scipy.io.savemat).

Arranca cuando /state = AUTOMATIC y graba durante record_duration_sec.
También responde a Ctrl+C guardando lo que haya grabado hasta ese momento.

Estructura del .mat generado
─────────────────────────────
El archivo se llama:  bearing_data_YYYYMMDD_HHMMSS.mat

Contiene:
  · Una struct por cada par (obs→target) activo:
        pair_<obs>_to_<tgt>
    con los campos:
        .t          [1 x N]   tiempo relativo desde AUTOMATIC [s]
        .g_est      [3 x N]   bearing estimado por el filtro  [adim, unitario]
        .g_meas     [3 x N]   bearing medido por ArUco        [adim, unitario]
        .g_dot      [3 x N]   ḃ estimado                     [rad/s]
        .observer   string
        .target     string

  · Una struct por observador con todos sus pares agrupados:
        observer_<name>
    con los campos:
        .pairs      cell array con los nombres de par ('cf2_to_cf1', ...)
        .n_pairs    número de targets activos para ese observador
        .n_samples  vector [1 x n_pairs] muestras por par

En MATLAB:
    data = load('bearing_data_20250101_120000.mat');
    % Ver todos los pares grabados:
    fieldnames(data)
    % Acceder a un par concreto:
    pair = data.pair_crazyflie2_to_crazyflie1;
    t = pair.t;  g_est = pair.g_est;  g_meas = pair.g_meas;
    % Ver resumen por observador:
    obs = data.observer_crazyflie2;
    obs.n_pairs, obs.n_samples

Parámetros ROS:
    robot_names                  lista de nombres de robots
    record_duration_sec          segundos de grabación (default 50)
    output_dir                   carpeta de salida    (default '.')
    aruco_id_to_robot_name_map   mismo mapa que en get_bearing_kalman
    min_samples                  mínimo de muestras para guardar un par (default 5)
    estimator_type               kalman | hinf  (solo metadato, default 'unknown')
"""
from collections import defaultdict
from dataclasses import dataclass
from dataclasses import field
from datetime import datetime
import os
import signal
import threading
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import scipy.io
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Int32
from uav_msgs.msg import BearingMeasurementsArray

# =============================================================================
# BUFFER POR PAR
# =============================================================================


@dataclass
class PairBuffer:
    times: List[float] = field(default_factory=list)
    g_est: List[np.ndarray] = field(default_factory=list)
    g_meas: List[np.ndarray] = field(default_factory=list)
    g_dot: List[np.ndarray] = field(default_factory=list)
    g_gt: List[np.ndarray] = field(default_factory=list)   # GT Gazebo (body frame)
    gt_dot: List[np.ndarray] = field(default_factory=list) # ġ GT analítico (body frame)


@dataclass
class ComparisonBuffer:
    """Buffer para modo comparativa: graba los 2 estimadores en sincronía."""
    times: List[float] = field(default_factory=list)
    g_meas: List[np.ndarray] = field(default_factory=list)
    g_gt: List[np.ndarray] = field(default_factory=list)
    gt_dot: List[np.ndarray] = field(default_factory=list) # ġ GT analítico (body frame)
    kalman_est: List[np.ndarray] = field(default_factory=list)
    kalman_dot: List[np.ndarray] = field(default_factory=list)
    hinf_est: List[np.ndarray] = field(default_factory=list)
    hinf_dot: List[np.ndarray] = field(default_factory=list)


@dataclass
class RawOnlyBuffer:
    """Buffer para raw_only_mode: solo medición cruda + GT, sin estimadores."""
    times: List[float] = field(default_factory=list)
    g_meas: List[np.ndarray] = field(default_factory=list)
    g_gt:   List[np.ndarray] = field(default_factory=list)
    gt_dot: List[np.ndarray] = field(default_factory=list) # ġ GT analítico (body frame)
    dists:  List[float] = field(default_factory=list)  # distancia GT inter-dron [m]


# =============================================================================
# NODO
# =============================================================================

class BearingDataRecorder(Node):

    STATE_AUTOMATIC = 1

    def __init__(self):
        super().__init__('bearing_data_recorder')

        # ── Parámetros ───────────────────────────────────────────────────────
        self.declare_parameter('robot_names',
                               ['crazyflie1',
                                'crazyflie2',
                                'crazyflie3',
                                'crazyflie4',
                                'crazyflie5',
                                'crazyflie6',
                                'crazyflie7'])
        self.declare_parameter('record_duration_sec', 45.0)
        self.declare_parameter('output_dir', '.')
        self.declare_parameter('aruco_id_to_robot_name_map',
                               ['1=crazyflie1',
                                '2=crazyflie2',
                                '3=crazyflie3',
                                '4=crazyflie4',
                                '5=crazyflie5',
                                '6=crazyflie6',
                                '7=crazyflie7'])
        self.declare_parameter('min_samples', 5)
        self.declare_parameter('estimator_type', 'unknown')
        self.declare_parameter('auto_start_recording', False)
        # Modo comparativa: suscribe a /vision_bearings_{kalman,hinf}
        self.declare_parameter('comparison_mode', False)
        # Modo raw-only: guarda solo g_meas + g_gt, sin correr estimadores
        self.declare_parameter('raw_only_mode', False)
        # Nombre del mundo Gazebo para suscribirse a poses del modelo
        # Topic esperado: /world/{gz_world_name}/pose/info  (PoseArray)
        # Dejar '' para deshabilitar GT (el .mat no incluirá g_gt)
        self.declare_parameter('gz_world_name', 'crazyflies')

        self.robot_names = self.get_parameter('robot_names').value
        self.record_dur = self.get_parameter('record_duration_sec').value
        self.output_dir = self.get_parameter('output_dir').value
        self.min_samples = self.get_parameter('min_samples').value
        self.estimator_type = self.get_parameter('estimator_type').value
        auto_start = self.get_parameter('auto_start_recording').value
        self.gz_world_name = self.get_parameter('gz_world_name').value
        self.comparison_mode = self.get_parameter('comparison_mode').value
        self.raw_only_mode   = self.get_parameter('raw_only_mode').value
        os.makedirs(self.output_dir, exist_ok=True)

        # Cache de poses Gazebo: nombre_robot → (posición [3], cuaternión [4] xyzw)
        self._pose_cache: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}

        # Cache de velocidades Gazebo (frame mundo): nombre_robot → velocidad [3]
        self._vel_cache: Dict[str, np.ndarray] = {}

        # ── Tablas de resolución de IDs ──────────────────────────────────────
        # Mapa ArUco 1-based ID → nombre robot
        self.id_to_name: Dict[int, str] = {}
        for e in self.get_parameter('aruco_id_to_robot_name_map').value:
            if '=' in e:
                k, v = e.split('=', 1)
                try:
                    self.id_to_name[int(k)] = v
                except ValueError:
                    pass

        # Mapa índice 0-based (posición en robot_names) → nombre robot
        # Usado por get_bearing_kalman que publica observer_id =
        # enumerate-index
        self._idx_to_name: Dict[int, str] = dict(enumerate(self.robot_names))

        # Mapa inverso nombre → 0-based índice (para logging)
        self._name_to_idx: Dict[str, int] = {
            n: i for i, n in enumerate(self.robot_names)
        }

        self.get_logger().info(
            f'ID→nombre (ArUco 1-base): {self.id_to_name}')
        self.get_logger().info(
            f'Idx→nombre (0-base):      {self._idx_to_name}')

        # ── Estado interno ───────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._recording = auto_start
        self._start_time = self._sim_now() if auto_start else 0.0
        self._done = False

        # Buffers principales
        self._buffers: Dict[Tuple[str, str],
                            PairBuffer] = defaultdict(PairBuffer)

        # Último g_meas recibido por par (para sincronizar con g_est)
        self._last_meas: Dict[Tuple[str, str], np.ndarray] = {}

        # Último g_dot recibido por par (del topic dedicado)
        self._last_dot: Dict[Tuple[str, str], np.ndarray] = {}

        # Buffers y cache para modo comparativa
        self._comp_buffers: Dict[Tuple[str, str],
                                 ComparisonBuffer] = defaultdict(ComparisonBuffer)

        # Buffer para raw_only_mode
        self._raw_buffers: Dict[Tuple[str, str],
                                RawOnlyBuffer] = defaultdict(RawOnlyBuffer)
        # Cache: último bearing+dot de hinf (el kalman actúa como reloj)
        self._last_comp: Dict[str, Dict[Tuple[str, str], Tuple[np.ndarray, np.ndarray]]] = {
            'hinf':  {},
        }

        # ── QoS ──────────────────────────────────────────────────────────────
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # ── Subscripciones ───────────────────────────────────────────────────
        self.create_subscription(
            Int32, '/state', self._state_cb, latched_qos)

        if self.raw_only_mode:
            # raw_only: /vision_bearings_raw actúa como reloj de muestreo
            self.create_subscription(
                BearingMeasurementsArray, '/vision_bearings_raw',
                self._raw_master_cb, qos_profile_sensor_data)
        else:
            self.create_subscription(
                BearingMeasurementsArray, '/vision_bearings',
                self._est_cb, qos_profile_sensor_data)

            self.create_subscription(
                BearingMeasurementsArray, '/vision_bearings_raw',
                self._meas_cb, qos_profile_sensor_data)

            self.create_subscription(
                BearingMeasurementsArray, '/vision_bearings_dot',
                self._dot_cb, qos_profile_sensor_data)

        if self.comparison_mode and not self.raw_only_mode:
            # Kalman actúa como reloj; hinf se cachean
            self.create_subscription(
                BearingMeasurementsArray, '/vision_bearings_kalman',
                self._comp_kalman_cb, qos_profile_sensor_data)
            self.create_subscription(
                BearingMeasurementsArray, '/vision_bearings_hinf',
                lambda msg: self._comp_cache_cb(msg, 'hinf'),
                qos_profile_sensor_data)

        # ── Poses Gazebo (GT bearing) ─────────────────────────────────────────
        # El bridge expone un topic individual por robot: /{name}/pose
        # (geometry_msgs/Pose). Se suscribe a uno por robot_name.
        if self.gz_world_name:
            for rname in self.robot_names:
                pose_topic = f'/{rname}/pose'
                self.create_subscription(
                    Pose, pose_topic,
                    lambda msg, n=rname: self._pose_cb(msg, n),
                    qos_profile_sensor_data)
                vel_topic = f'/{rname}/velocity'
                self.create_subscription(
                    Twist, vel_topic,
                    lambda msg, n=rname: self._vel_cb(msg, n),
                    qos_profile_sensor_data)
            self.get_logger().info(
                f'GT Gazebo: suscrito a /{{{"|".join(self.robot_names)}}}/pose|velocity')
        else:
            self.get_logger().warn(
                'gz_world_name vacío — g_gt y gt_dot NO se grabarán en el .mat.')

        self.create_timer(0.1, self._check_duration)

        mode = 'AUTO-START' if auto_start else 'espera /state=AUTOMATIC'
        raw_str  = ' | RAW-ONLY (sin estimadores)' if self.raw_only_mode else ''
        comp_str = ' | COMPARATIVA (kalman+hinf)' if (self.comparison_mode and not self.raw_only_mode) else ''
        self.get_logger().info(
            f'BearingDataRecorder listo | '
            f'duración={self.record_dur} s | output={self.output_dir} | '
            f'modo={mode}{raw_str}{comp_str}')

    # =========================================================================
    # HELPERS
    # =========================================================================

    def _sim_now(self) -> float:
        """Simulated time (requires use_sim_time + /clock bridge) instead of
        wall-clock — Gazebo's real-time-factor isn't guaranteed to be 1.0.
        Used for recording duration/timestamps so they reflect sim seconds,
        not wall seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _resolve(self, obs_idx: int, tgt_idx: int) -> Tuple[str, str]:
        """
        Resuelve (observer_id, target_id) → (nombre_obs, nombre_tgt).

        get_bearing_kalman publica índices 0-base (posición en
        robot_names). Como salvaguarda se prueba también el mapa ArUco
        1-base por si el publisher usa una convención distinta.
        """
        # Intento 1: índice 0-base (convención de get_bearing_kalman)
        obs = self._idx_to_name.get(obs_idx)
        tgt = self._idx_to_name.get(tgt_idx)

        # Intento 2: ID ArUco 1-base (fallback)
        if obs is None:
            obs = self.id_to_name.get(obs_idx)
        if tgt is None:
            tgt = self.id_to_name.get(tgt_idx)

        return obs, tgt

    @staticmethod
    def _unit(v: np.ndarray) -> np.ndarray:
        n = np.linalg.norm(v)
        return v / n if n > 1e-6 else np.zeros(3)

    @staticmethod
    def _quat_to_rot(q: np.ndarray) -> np.ndarray:
        """Cuaternión (x,y,z,w) → matriz de rotación 3×3 (mundo→cuerpo = R_wb)."""
        x, y, z, w = q
        R = np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])
        return R

    def _gt_bearing_body(self, obs_name: str, tgt_name: str,
                         return_dist: bool = False):
        """
        Calcula el bearing ground truth en el frame del cuerpo del observador.

        g_gt_world = (p_tgt - p_obs) / ||p_tgt - p_obs||
        g_gt_body  = R_wb_obs.T @ g_gt_world

        Si return_dist=True devuelve (g_body, dist_m); si no, solo g_body.
        Devuelve zeros(3) (y dist=0.0) si alguna pose no está en el cache.
        """
        if obs_name not in self._pose_cache or tgt_name not in self._pose_cache:
            return (np.zeros(3), 0.0) if return_dist else np.zeros(3)
        p_obs, q_obs = self._pose_cache[obs_name]
        p_tgt, _     = self._pose_cache[tgt_name]
        diff = p_tgt - p_obs
        n = np.linalg.norm(diff)
        if n < 1e-6:
            return (np.zeros(3), 0.0) if return_dist else np.zeros(3)
        g_world = diff / n
        R_wb = self._quat_to_rot(q_obs)   # R del mundo al cuerpo
        g_body = R_wb.T @ g_world          # g en frame del cuerpo del observador
        return (g_body, float(n)) if return_dist else g_body

    def _gt_bearing_dot_body(self, obs_name: str, tgt_name: str) -> np.ndarray:
        """
        Bearing-dot GT en frame del cuerpo del observador, calculado analíticamente
        a partir de poses y velocidades de Gazebo.

        Fórmula (idéntica a dot_gij_c del controlador C++, ec. cinemática de bearing):
            ġ_world = P_g · (v_tgt − v_obs) / d
            ġ_body  = R_wb · ġ_world          (R_wb.T = R_bw^T convierte mundo → cuerpo)

        donde P_g = I − g·gᵀ es la proyección sobre el espacio tangente de S²
        y d = ‖p_tgt − p_obs‖ es la distancia inter-robot.

        Devuelve zeros(3) si algún cache (pose o velocidad) aún no tiene datos.
        """
        if obs_name not in self._pose_cache or tgt_name not in self._pose_cache:
            return np.zeros(3)
        if obs_name not in self._vel_cache or tgt_name not in self._vel_cache:
            return np.zeros(3)

        p_obs, q_obs = self._pose_cache[obs_name]
        p_tgt, _     = self._pose_cache[tgt_name]
        v_obs = self._vel_cache[obs_name]
        v_tgt = self._vel_cache[tgt_name]

        diff = p_tgt - p_obs
        d = np.linalg.norm(diff)
        if d < 1e-6:
            return np.zeros(3)

        g_world = diff / d
        Pg      = np.eye(3) - np.outer(g_world, g_world)   # proyección tangente
        gdot_world = Pg @ (v_tgt - v_obs) / d

        R_wb = self._quat_to_rot(q_obs)         # misma convención que _gt_bearing_body
        return R_wb.T @ gdot_world              # mundo → cuerpo del observador

    # =========================================================================
    # CALLBACKS
    # =========================================================================

    def _pose_cb(self, msg: Pose, robot_name: str):
        """Actualiza el cache de poses desde /{robot_name}/pose (Pose individual)."""
        p = np.array([msg.position.x, msg.position.y, msg.position.z])
        q = np.array([msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w])
        self._pose_cache[robot_name] = (p, q)

    def _vel_cb(self, msg: Twist, robot_name: str):
        """Actualiza el cache de velocidades (frame mundo) desde /{robot_name}/velocity."""
        self._vel_cache[robot_name] = np.array(
            [msg.linear.x, msg.linear.y, msg.linear.z])

    def _state_cb(self, msg: Int32):
        if msg.data == self.STATE_AUTOMATIC and not self._recording and not self._done:
            with self._lock:
                self._recording = True
                self._start_time = self._sim_now()
            self.get_logger().info(
                f'AUTOMATIC detectado — grabando {self.record_dur} s...')

    def _est_cb(self, msg: BearingMeasurementsArray):
        """
        Bearing estimado por KF → g_est.

        Se usa como reloj de muestreo.
        """
        if not self._recording:
            return
        t_rel = self._sim_now() - self._start_time

        skipped_ids: List[Tuple[int, int]] = []

        for m in msg.measurements:
            obs, tgt = self._resolve(m.observer_id, m.target_id)
            if obs is None or tgt is None:
                skipped_ids.append((m.observer_id, m.target_id))
                continue

            g_e = self._unit(np.array([m.bearing_vector.x,
                                       m.bearing_vector.y,
                                       m.bearing_vector.z]))
            # ḃ: usa bearing_vector_dot embebido en /vision_bearings como
            # fuente primaria
            gd_inline = np.array([m.bearing_vector_dot.x,
                                  m.bearing_vector_dot.y,
                                  m.bearing_vector_dot.z])

            # GT bearing y bearing-dot en frame del cuerpo (requiere caches de Gazebo)
            g_gt   = self._gt_bearing_body(obs, tgt)
            gd_gt  = self._gt_bearing_dot_body(obs, tgt)

            key = (obs, tgt)
            with self._lock:
                buf = self._buffers[key]
                buf.times.append(t_rel)
                buf.g_est.append(g_e)

                # g_meas: último valor crudo recibido (o g_est si no hay)
                buf.g_meas.append(
                    self._last_meas.get(key, g_e).copy())

                # g_dot: topic dedicado > inline > zeros
                buf.g_dot.append(
                    self._last_dot.get(key, gd_inline).copy())

                # g_gt y gt_dot: GT analítico desde Gazebo
                buf.g_gt.append(g_gt)
                buf.gt_dot.append(gd_gt)

        if skipped_ids:
            self.get_logger().warn(
                f'_est_cb: IDs sin resolver → {skipped_ids}. '
                f'Verifica que robot_names y aruco_id_to_robot_name_map '
                f'coincidan con el nodo de bearing.',
                throttle_duration_sec=5.0)

    def _meas_cb(self, msg: BearingMeasurementsArray):
        """Bearing crudo ArUco: actualiza _last_meas."""
        if not self._recording:
            return
        for m in msg.measurements:
            obs, tgt = self._resolve(m.observer_id, m.target_id)
            if obs is None or tgt is None:
                continue
            g = self._unit(np.array([m.bearing_vector.x,
                                     m.bearing_vector.y,
                                     m.bearing_vector.z]))
            with self._lock:
                self._last_meas[(obs, tgt)] = g

    def _dot_cb(self, msg: BearingMeasurementsArray):
        """Ḃ del topic dedicado → actualiza _last_dot."""
        if not self._recording:
            return
        for m in msg.measurements:
            obs, tgt = self._resolve(m.observer_id, m.target_id)
            if obs is None or tgt is None:
                continue
            gd = np.array([m.bearing_vector.x,
                           m.bearing_vector.y,
                           m.bearing_vector.z])
            with self._lock:
                self._last_dot[(obs, tgt)] = gd

    def _raw_master_cb(self, msg: BearingMeasurementsArray):
        """raw_only_mode: /vision_bearings_raw actúa como reloj de muestreo."""
        if not self._recording:
            return
        t_rel = self._sim_now() - self._start_time
        for m in msg.measurements:
            obs, tgt = self._resolve(m.observer_id, m.target_id)
            if obs is None or tgt is None:
                continue
            g = self._unit(np.array([m.bearing_vector.x,
                                     m.bearing_vector.y,
                                     m.bearing_vector.z]))
            g_gt  = self._gt_bearing_body(obs, tgt)
            gd_gt = self._gt_bearing_dot_body(obs, tgt)
            # Distancia solvePnP publicada por el nodo (misma que usa el estimador
            # en línea); fallback a distancia GT si el campo aún es 0 (dato antiguo)
            dist_m = float(m.distance) if m.distance > 1e-6 else float(
                self._gt_bearing_body(obs, tgt, return_dist=True)[1])
            key = (obs, tgt)
            with self._lock:
                buf = self._raw_buffers[key]
                buf.times.append(t_rel)
                buf.g_meas.append(g)
                buf.g_gt.append(g_gt)
                buf.gt_dot.append(gd_gt)
                buf.dists.append(dist_m)

    def _comp_cache_cb(self, msg: BearingMeasurementsArray, est_type: str):
        """Cachea el último bearing+dot de hinf (sin grabar todavía)."""
        if not self._recording:
            return
        for m in msg.measurements:
            obs, tgt = self._resolve(m.observer_id, m.target_id)
            if obs is None or tgt is None:
                continue
            g = self._unit(np.array([m.bearing_vector.x,
                                     m.bearing_vector.y,
                                     m.bearing_vector.z]))
            gd = np.array([m.bearing_vector_dot.x,
                           m.bearing_vector_dot.y,
                           m.bearing_vector_dot.z])
            with self._lock:
                self._last_comp[est_type][(obs, tgt)] = (g, gd)

    def _comp_kalman_cb(self, msg: BearingMeasurementsArray):
        """Kalman comp llega → graba una muestra con los 2 estimadores."""
        if not self._recording:
            return
        t_rel = self._sim_now() - self._start_time

        for m in msg.measurements:
            obs, tgt = self._resolve(m.observer_id, m.target_id)
            if obs is None or tgt is None:
                continue

            g_kf = self._unit(np.array([m.bearing_vector.x,
                                        m.bearing_vector.y,
                                        m.bearing_vector.z]))
            gd_kf = np.array([m.bearing_vector_dot.x,
                               m.bearing_vector_dot.y,
                               m.bearing_vector_dot.z])
            g_gt  = self._gt_bearing_body(obs, tgt)
            gd_gt = self._gt_bearing_dot_body(obs, tgt)
            key = (obs, tgt)

            with self._lock:
                g_meas = self._last_meas.get(key, g_kf).copy()
                g_hi, gd_hi = self._last_comp['hinf'].get(key, (g_kf.copy(), gd_kf.copy()))

                buf = self._comp_buffers[key]
                buf.times.append(t_rel)
                buf.g_meas.append(g_meas)
                buf.g_gt.append(g_gt)
                buf.gt_dot.append(gd_gt)
                buf.kalman_est.append(g_kf)
                buf.kalman_dot.append(gd_kf)
                buf.hinf_est.append(g_hi.copy())
                buf.hinf_dot.append(gd_hi.copy())

    # =========================================================================
    # CONTROL DE DURACIÓN
    # =========================================================================

    def _check_duration(self):
        if not self._recording or self._done:
            return
        elapsed = self._sim_now() - self._start_time
        if elapsed >= self.record_dur:
            self.get_logger().info(
                f'Duración alcanzada ({self.record_dur} s). Guardando...')
            self._done = True
            self._recording = False
            threading.Thread(target=self._save, daemon=True).start()

    # =========================================================================
    # GUARDADO EN .MAT
    # =========================================================================

    def _save(self):
        if self.raw_only_mode:
            self._save_raw_only()
        elif self.comparison_mode:
            self._save_comparison()
        else:
            self._save_single()

    def _save_raw_only(self):
        """Guarda solo g_meas + g_gt + t; los estimadores se aplican offline."""
        with self._lock:
            snapshot: Dict[Tuple[str, str], dict] = {}
            for (obs, tgt), buf in self._raw_buffers.items():
                n = min(len(buf.times), len(buf.g_meas), len(buf.g_gt),
                        len(buf.dists) if buf.dists else len(buf.times))
                if n < self.min_samples:
                    self.get_logger().warn(
                        f'Par {obs}→{tgt}: solo {n} muestras '
                        f'(< min_samples={self.min_samples}), ignorado.')
                    continue
                g_gt_arr = np.array(buf.g_gt[:n], dtype=np.float64).T
                gt_valid = bool(np.any(np.abs(g_gt_arr) > 1e-6))
                dists_arr = (np.array(buf.dists[:n], dtype=np.float64).reshape(1, -1)
                             if buf.dists else np.ones((1, n), dtype=np.float64))
                gt_dot_arr = np.array(buf.gt_dot[:n], dtype=np.float64).T if buf.gt_dot else np.zeros((3, n))
                snapshot[(obs, tgt)] = {
                    't':       np.array(buf.times[:n], dtype=np.float64).reshape(1, -1),
                    'g_meas':  np.array(buf.g_meas[:n], dtype=np.float64).T,
                    'g_gt':    g_gt_arr,
                    'gt_dot':  gt_dot_arr,
                    'gt_valid': float(gt_valid),
                    'dists':   dists_arr,
                }

        if not snapshot:
            self.get_logger().warn('No hay datos crudos para guardar.')
            return

        mat_dict: dict = {}
        mat_dict['record_duration_sec'] = self.record_dur
        mat_dict['robot_names'] = np.array(self.robot_names, dtype=object)
        mat_dict['timestamp_utc'] = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S UTC')
        mat_dict['estimator_type'] = 'raw'

        obs_groups: Dict[str, List[Tuple[str, int]]] = defaultdict(list)
        for (obs, tgt), arrays in snapshot.items():
            var_name = f'pair_{obs}_to_{tgt}'.replace('-', '_')
            mat_dict[var_name] = {
                't':        arrays['t'],
                'g_meas':   arrays['g_meas'],
                'g_gt':     arrays['g_gt'],
                'gt_dot':   arrays['gt_dot'],
                'gt_valid': arrays['gt_valid'],
                'dists':    arrays['dists'],
                'observer': obs,
                'target':   tgt,
            }
            obs_groups[obs].append((tgt, arrays['t'].shape[1]))

        for obs_name, tgt_list in obs_groups.items():
            var_name = f'observer_{obs_name}'.replace('-', '_')
            pair_names = [f'pair_{obs_name}_to_{tgt}'.replace('-', '_')
                          for tgt, _ in tgt_list]
            mat_dict[var_name] = {
                'pairs':     np.array(pair_names, dtype=object),
                'n_pairs':   float(len(tgt_list)),
                'n_samples': np.array([ns for _, ns in tgt_list],
                                      dtype=np.float64).reshape(1, -1),
            }

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'bearing_raw_{ts}.mat'
        filepath = os.path.join(self.output_dir, filename)
        self._write_mat(mat_dict, filepath, obs_groups, snapshot)

    def _save_single(self):
        """Guardado estándar (un solo estimador)."""
        with self._lock:
            snapshot: Dict[Tuple[str, str], dict] = {}
            for (obs, tgt), buf in self._buffers.items():
                n = min(len(buf.times), len(buf.g_est),
                        len(buf.g_meas), len(buf.g_dot), len(buf.g_gt))
                if n < self.min_samples:
                    self.get_logger().warn(
                        f'Par {obs}→{tgt}: solo {n} muestras '
                        f'(< min_samples={self.min_samples}), ignorado.')
                    continue

                g_gt_arr   = np.array(buf.g_gt[:n], dtype=np.float64).T
                gt_valid   = bool(np.any(np.abs(g_gt_arr) > 1e-6))
                gt_dot_arr = np.array(buf.gt_dot[:n], dtype=np.float64).T if buf.gt_dot else np.zeros((3, n))

                snapshot[(obs, tgt)] = {
                    't':      np.array(buf.times[:n], dtype=np.float64).reshape(1, -1),
                    'g_est':  np.array(buf.g_est[:n],  dtype=np.float64).T,
                    'g_meas': np.array(buf.g_meas[:n], dtype=np.float64).T,
                    'g_dot':  np.array(buf.g_dot[:n],  dtype=np.float64).T,
                    'g_gt':   g_gt_arr,
                    'gt_dot': gt_dot_arr,
                    'gt_valid': float(gt_valid),
                }

        if not snapshot:
            self.get_logger().warn('No hay datos para guardar.')
            return

        mat_dict: dict = {}
        mat_dict['record_duration_sec'] = self.record_dur
        mat_dict['robot_names'] = np.array(self.robot_names, dtype=object)
        mat_dict['timestamp_utc'] = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S UTC')
        mat_dict['estimator_type'] = self.estimator_type

        for (obs, tgt), arrays in snapshot.items():
            var_name = f'pair_{obs}_to_{tgt}'.replace('-', '_')
            mat_dict[var_name] = {
                't':        arrays['t'],
                'g_est':    arrays['g_est'],
                'g_meas':   arrays['g_meas'],
                'g_dot':    arrays['g_dot'],
                'g_gt':     arrays['g_gt'],
                'gt_dot':   arrays['gt_dot'],
                'gt_valid': arrays['gt_valid'],
                'observer': obs,
                'target':   tgt,
            }

        obs_groups: Dict[str, List[Tuple[str, int]]] = defaultdict(list)
        for (obs, tgt), arrays in snapshot.items():
            obs_groups[obs].append((tgt, arrays['t'].shape[1]))

        for obs_name, tgt_list in obs_groups.items():
            var_name = f'observer_{obs_name}'.replace('-', '_')
            pair_names = [f'pair_{obs_name}_to_{tgt}'.replace('-', '_')
                          for tgt, _ in tgt_list]
            n_samples = np.array([ns for _, ns in tgt_list], dtype=np.float64)
            mat_dict[var_name] = {
                'pairs': np.array(pair_names, dtype=object),
                'n_pairs': float(len(tgt_list)),
                'n_samples': n_samples.reshape(1, -1),
            }

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'bearing_data_{self.estimator_type}_{ts}.mat'
        filepath = os.path.join(self.output_dir, filename)
        self._write_mat(mat_dict, filepath, obs_groups, snapshot)

    def _save_comparison(self):
        """Guardado modo comparativa: kalman+hinf en un solo .mat."""
        with self._lock:
            snapshot: Dict[Tuple[str, str], dict] = {}
            for (obs, tgt), buf in self._comp_buffers.items():
                n = min(len(buf.times), len(buf.kalman_est), len(buf.hinf_est))
                if n < self.min_samples:
                    self.get_logger().warn(
                        f'Par {obs}→{tgt}: solo {n} muestras, ignorado.')
                    continue

                g_gt_arr   = np.array(buf.g_gt[:n], dtype=np.float64).T
                gt_valid   = bool(np.any(np.abs(g_gt_arr) > 1e-6))
                gt_dot_arr = np.array(buf.gt_dot[:n], dtype=np.float64).T if buf.gt_dot else np.zeros((3, n))

                snapshot[(obs, tgt)] = {
                    't':          np.array(buf.times[:n], dtype=np.float64).reshape(1, -1),
                    'g_meas':     np.array(buf.g_meas[:n],     dtype=np.float64).T,
                    'g_gt':       g_gt_arr,
                    'gt_dot':     gt_dot_arr,
                    'gt_valid':   float(gt_valid),
                    'kalman_est': np.array(buf.kalman_est[:n], dtype=np.float64).T,
                    'kalman_dot': np.array(buf.kalman_dot[:n], dtype=np.float64).T,
                    'hinf_est':   np.array(buf.hinf_est[:n],   dtype=np.float64).T,
                    'hinf_dot':   np.array(buf.hinf_dot[:n],   dtype=np.float64).T,
                }

        if not snapshot:
            self.get_logger().warn('No hay datos de comparativa para guardar.')
            return

        mat_dict: dict = {}
        mat_dict['record_duration_sec'] = self.record_dur
        mat_dict['robot_names'] = np.array(self.robot_names, dtype=object)
        mat_dict['timestamp_utc'] = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S UTC')
        mat_dict['estimator_type'] = 'comparison'

        obs_groups: Dict[str, List[Tuple[str, int]]] = defaultdict(list)
        for (obs, tgt), arrays in snapshot.items():
            var_name = f'pair_{obs}_to_{tgt}'.replace('-', '_')
            mat_dict[var_name] = {
                't':          arrays['t'],
                'g_meas':     arrays['g_meas'],
                'g_gt':       arrays['g_gt'],
                'gt_dot':     arrays['gt_dot'],
                'gt_valid':   arrays['gt_valid'],
                'kalman_est': arrays['kalman_est'],
                'kalman_dot': arrays['kalman_dot'],
                'hinf_est':   arrays['hinf_est'],
                'hinf_dot':   arrays['hinf_dot'],
                'observer':   obs,
                'target':     tgt,
            }
            obs_groups[obs].append((tgt, arrays['t'].shape[1]))

        for obs_name, tgt_list in obs_groups.items():
            var_name = f'observer_{obs_name}'.replace('-', '_')
            pair_names = [f'pair_{obs_name}_to_{tgt}'.replace('-', '_')
                          for tgt, _ in tgt_list]
            mat_dict[var_name] = {
                'pairs':     np.array(pair_names, dtype=object),
                'n_pairs':   float(len(tgt_list)),
                'n_samples': np.array([ns for _, ns in tgt_list],
                                      dtype=np.float64).reshape(1, -1),
            }

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'bearing_comparison_{ts}.mat'
        filepath = os.path.join(self.output_dir, filename)
        self._write_mat(mat_dict, filepath, obs_groups, snapshot)

    def _write_mat(self, mat_dict, filepath, obs_groups, snapshot):
        """Escribe el .mat y muestra resumen en consola."""
        scipy.io.savemat(filepath, mat_dict, do_compression=True)

        filename = os.path.basename(filepath)
        is_raw  = self.raw_only_mode
        is_comp = self.comparison_mode and not is_raw

        lines = [f"\n{'=' * 60}",
                 f'  Datos guardados: {filepath}',
                 f'  Observadores activos: {len(obs_groups)} | '
                 f'Pares grabados: {len(snapshot)}',
                 '']

        for obs_name in sorted(obs_groups.keys()):
            lines.append(f'  {obs_name}:')
            for tgt, ns in sorted(obs_groups[obs_name]):
                key = (obs_name, tgt)
                t_arr = snapshot[key]['t']
                dt = float(np.diff(t_arr).mean()) if ns > 1 else 0.0
                lines.append(f'    → {tgt}: {ns} muestras  dt_medio={dt:.4f} s')

        matlab_hint = [
            '',
            '  En MATLAB:',
            f"    data = load('{filename}');",
            '    fieldnames(data)',
            '    par = data.pair_crazyflie2_to_crazyflie1;',
        ]
        if is_raw:
            matlab_hint += [
                '    t = par.t; g_meas = par.g_meas; g_gt = par.g_gt;',
            ]
        elif is_comp:
            matlab_hint += [
                '    t = par.t;',
            ]
        else:
            matlab_hint += [
                '    t = par.t; g_est = par.g_est; g_meas = par.g_meas;',
            ]
        matlab_hint.append(f"{'=' * 60}")
        lines += matlab_hint
        self.get_logger().info('\n'.join(lines))


# =============================================================================
# MAIN
# =============================================================================

def main():
    rclpy.init()
    node = BearingDataRecorder()

    def _sigint(sig, frame):
        node.get_logger().info('Ctrl+C — guardando datos...')
        if not node._done:
            node._done = True
            node._recording = False
            t = threading.Thread(target=node._save, daemon=True)
            t.start()
            t.join(timeout=15)
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, _sigint)

    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        if not node._done:
            node._done = True
            node._recording = False
            node._save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
