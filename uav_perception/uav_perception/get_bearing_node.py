#!/usr/bin/env python3
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""
get_bearing_node.py.

===================
Nodo ROS2 genérico para estimación de bearing con ArUco + solvePnP.

Selecciona el estimador mediante el parámetro ROS 'estimator_type':
    'kalman'  →  Filtro de Kalman lineal en espacio tangente  (baseline)
    'hinf'    →  Filtro H-infinito                            (robusto)

Publicaciones (idénticas para los 2 estimadores):
    /vision_bearings       → BearingMeasurementsArray  (bearing filtrado + ḃ)
    /vision_bearings_raw   → BearingMeasurementsArray  (bearing crudo ArUco)
    /vision_bearings_dot   → BearingMeasurementsArray  (ḃ estimada por el filtro)

Uso rápido:
    # KF original
    ros2 run <pkg> get_bearing_node --ros-args -p estimator_type:=kalman

    # H-infinito  (γ=2.5)
    ros2 run <pkg> get_bearing_node --ros-args \
        -p estimator_type:=hinf -p hinf_gamma:=2.5

"""

from dataclasses import dataclass
from dataclasses import field
import threading
from typing import Dict

import cv2
from cv_bridge import CvBridge
import numpy as np
from numpy.linalg import norm
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
# Librería de estimadores (mismo directorio o en el PYTHONPATH del paquete)
from uav_estimators.bearing_estimators import _TangentBase
from uav_estimators.bearing_estimators import ESTIMATOR_HIN
from uav_estimators.bearing_estimators import ESTIMATOR_KF
from uav_estimators.bearing_estimators import make_estimator
from uav_msgs.msg import BearingMeasurement
from uav_msgs.msg import BearingMeasurementsArray

# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTES
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_MARKER_LEN = 0.045
DEFAULT_FOV_DEG = 66.0
DEFAULT_W = 640
DEFAULT_H = 480
DEFAULT_PUB_HZ = 50.0
DEFAULT_MAX_DIST = 10.0
KALMAN_MAX_PRED_SEC = 2.0  # Segundos máximos de predicción sin medición


# ─────────────────────────────────────────────────────────────────────────────
# ESTRUCTURAS DE DATOS
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class BearingResult:
    bearing: np.ndarray = field(default_factory=lambda: np.zeros(3))
    bearing_dot: np.ndarray = field(default_factory=lambda: np.zeros(3))
    confidence: float = 0.0
    uncertainty: float = 0.0
    from_prediction: bool = False
    meas_ts: float = 0.0
    dist: float = 0.0   # solvePnP distance [m]; 0.0 for pure predictions


class ObserverShard:
    """Estado aislado por observador (hilo-seguro)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.est_map: Dict[str, _TangentBase] = {}   # estimador por target
        self.results: Dict[str, BearingResult] = {}
        self.raw_results: Dict[str, BearingResult] = {}
        # comparison mode: target → {est_type → estimador / resultado}
        self.est_comp: Dict[str, Dict[str, _TangentBase]] = {}
        self.res_comp: Dict[str, Dict[str, BearingResult]] = {}


# ═════════════════════════════════════════════════════════════════════════════
# NODO ROS2
# ═════════════════════════════════════════════════════════════════════════════
class BearingDetectorNode(Node):

    def __init__(self):
        super().__init__('get_bearing_node')

        # ── Parámetros generales ─────────────────────────────────────────────
        self.declare_parameter('robot_names',
                               ['crazyflie1', 'crazyflie2', 'crazyflie3'])
        self.declare_parameter('world_name', 'crazyflie')
        self.declare_parameter('publish_rate_hz', DEFAULT_PUB_HZ)
        self.declare_parameter('image_width', DEFAULT_W)
        self.declare_parameter('image_height', DEFAULT_H)
        self.declare_parameter('camera_fov_horizontal_deg', DEFAULT_FOV_DEG)
        self.declare_parameter('aruco_marker_length', DEFAULT_MARKER_LEN)
        self.declare_parameter('max_marker_distance', DEFAULT_MAX_DIST)
        self.declare_parameter(
            'aruco_id_to_robot_name_map', [
                '1=crazyflie1', '2=crazyflie2', '3=crazyflie3'])
        self.declare_parameter('allowed_connections', [''])
        self.declare_parameter('debug_mode', False)

        # ── Parámetro de selección del estimador ─────────────────────────────
        self.declare_parameter('estimator_type', ESTIMATOR_KF)

        # ── Parámetros específicos de cada estimador ─────────────────────────
        # KF
        self.declare_parameter('kf_q_vel', 0.05)
        self.declare_parameter('kf_r_base', 5e-3)
        self.declare_parameter('kf_r_scale', 1e-3)
        # H∞
        self.declare_parameter('hinf_gamma', 2.0)
        self.declare_parameter('hinf_q_vel', 0.05)
        self.declare_parameter('hinf_r_base', 5e-3)
        self.declare_parameter('hinf_r_scale', 1e-3)
        self.declare_parameter('hinf_l_pos', 1.0)
        self.declare_parameter('hinf_l_vel', 0.5)
        self.declare_parameter('hinf_l_acc', 0.1)
        # Modo comparativa: corre los 2 estimadores en paralelo
        self.declare_parameter('comparison_mode', False)

        # ── Leer parámetros ──────────────────────────────────────────────────
        self.robot_names = self.get_parameter('robot_names').value
        self.world_name = self.get_parameter('world_name').value
        self.pub_rate = self.get_parameter('publish_rate_hz').value
        self.img_w = self.get_parameter('image_width').value
        self.img_h = self.get_parameter('image_height').value
        self.fov_deg = self.get_parameter('camera_fov_horizontal_deg').value
        self.marker_len = self.get_parameter('aruco_marker_length').value
        self.max_dist = self.get_parameter('max_marker_distance').value
        self.debug_mode = self.get_parameter('debug_mode').value

        self.estimator_type = self.get_parameter('estimator_type').value
        self.comparison_mode = self.get_parameter('comparison_mode').value

        # Argumentos para la fábrica según el estimador elegido
        self._est_kwargs = self._build_estimator_kwargs()
        if self.comparison_mode:
            self._all_est_kwargs = self._build_all_est_kwargs()

        # Mapa ID ArUco → nombre de robot
        self.id_to_name: Dict[int, str] = {}
        for e in self.get_parameter('aruco_id_to_robot_name_map').value:
            if '=' in e:
                k, v = e.split('=', 1)
                try:
                    self.id_to_name[int(k)] = v
                except ValueError:
                    pass

        self.robot_index = {n: i for i, n in enumerate(self.robot_names)}

        # Conexiones permitidas (grafo laplaciano)
        raw_conn = self.get_parameter('allowed_connections').value
        self._allowed: set = set()
        for entry in raw_conn:
            entry = entry.strip()
            if entry and '->' in entry:
                obs, tgt = entry.split('->', 1)
                self._allowed.add((obs.strip(), tgt.strip()))

        # ── Setup ────────────────────────────────────────────────────────────
        self._setup_camera()
        self._setup_aruco()

        self.shards: Dict[str, ObserverShard] = {
            n: ObserverShard() for n in self.robot_names}
        self._processing: Dict[str, bool] = {
            n: False for n in self.robot_names}
        self._proc_locks: Dict[str, threading.Lock] = {
            n: threading.Lock() for n in self.robot_names}

        self._stat_lock = threading.Lock()
        self._frames_proc = 0
        self._frames_nodet = 0
        self._markers_det = 0

        self.bridge = CvBridge()
        self._setup_subs()
        self._setup_pubs()
        self.create_timer(1.0 / self.pub_rate, self._publish_bearings)
        # self.create_timer(10.0, self._log_stats)

        mode_str = (
            'COMPARATIVA (kalman+hinf)' if self.comparison_mode else self.estimator_type
        )
        self.get_logger().info(
            f'BearingDetector | mode={mode_str} | '
            f'robots={self.robot_names} | {self.pub_rate:.0f} Hz | '
            f'marker={self.marker_len} m'
        )
        if self._allowed:
            self.get_logger().info(
                f'Filtro de conexiones: {len(self._allowed)} pares activos')
            for obs, tgt in sorted(self._allowed):
                self.get_logger().info(f'  {obs} -> {tgt}')
        else:
            self.get_logger().info('Filtro de conexiones: INACTIVO (todos los pares)')

    # ── Construcción de kwargs por estimador ────────────────────────────────

    def _build_estimator_kwargs(self) -> dict:
        t = self.estimator_type
        if t == ESTIMATOR_KF:
            return {
                'q_vel': self.get_parameter('kf_q_vel').value,
                'r_base': self.get_parameter('kf_r_base').value,
                'r_scale': self.get_parameter('kf_r_scale').value,
            }
        elif t == ESTIMATOR_HIN:
            return {
                'gamma': self.get_parameter('hinf_gamma').value,
                'q_vel': self.get_parameter('hinf_q_vel').value,
                'r_base': self.get_parameter('hinf_r_base').value,
                'r_scale': self.get_parameter('hinf_r_scale').value,
                'l_pos': self.get_parameter('hinf_l_pos').value,
                'l_vel': self.get_parameter('hinf_l_vel').value,
                'l_acc': self.get_parameter('hinf_l_acc').value,
            }
        else:
            self.get_logger().warn(
                f"Estimador desconocido '{t}', usando KF por defecto.")
            return {
                'q_vel': 0.05,
                'r_base': 5e-3,
                'r_scale': 1e-3,
            }

    def _build_all_est_kwargs(self) -> Dict[str, dict]:
        """Devuelve kwargs para los 2 estimadores (modo comparativa)."""
        return {
            ESTIMATOR_KF: {
                'q_vel':   self.get_parameter('kf_q_vel').value,
                'r_base':  self.get_parameter('kf_r_base').value,
                'r_scale': self.get_parameter('kf_r_scale').value,
            },
            ESTIMATOR_HIN: {
                'gamma':   self.get_parameter('hinf_gamma').value,
                'q_vel':   self.get_parameter('hinf_q_vel').value,
                'r_base':  self.get_parameter('hinf_r_base').value,
                'r_scale': self.get_parameter('hinf_r_scale').value,
                'l_pos':   self.get_parameter('hinf_l_pos').value,
                'l_vel':   self.get_parameter('hinf_l_vel').value,
                'l_acc':   self.get_parameter('hinf_l_acc').value,
            },
        }

    # ── Cámara ──────────────────────────────────────────────────────────────

    def _setup_camera(self):
        fx = self.img_w / (2.0 * np.tan(np.radians(self.fov_deg / 2.0)))
        cx, cy = self.img_w / 2.0, self.img_h / 2.0
        self.K = np.array(
            [[fx, 0, cx], [0, fx, cy], [0, 0, 1.0]], dtype=np.float64)
        self.D = np.array(
            [-0.25, 0.12, 0.0, 0.0, 0.0], dtype=np.float64)
        # Rotación fija cámara → robot (ec. 5 del paper)
        self.R_cam2rob = np.array(
            [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float64)
        h = self.marker_len / 2.0
        self.obj_pts = np.array(
            [[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]],
            dtype=np.float32)

    # ── Detectores ArUco ────────────────────────────────────────────────────

    def _setup_aruco(self):
        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 23
        params.adaptiveThreshWinSizeStep = 10
        params.minMarkerPerimeterRate = 0.03
        params.maxMarkerPerimeterRate = 4.0
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.detectors = [
            (cv2.aruco.ArucoDetector(
                cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100),
                params), 'DICT_4X4_100'),
            (cv2.aruco.ArucoDetector(
                cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250),
                params), 'DICT_4X4_250'),
            (cv2.aruco.ArucoDetector(
                cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
                params), 'APRILTAG_36h11'),
        ]
        self.get_logger().info(
            'Detectores: DICT_4X4_100 + DICT_4X4_250 + APRILTAG_36h11')

    # ── Suscriptores ────────────────────────────────────────────────────────

    def _setup_subs(self):
        for name in self.robot_names:
            topic = (f'/world/{self.world_name}/model/{name}'
                     f'/link/body/sensor/rgb_camera_sensor/image')
            self.create_subscription(
                Image, topic,
                lambda msg, n=name: self._image_callback(msg, n),
                qos_profile_sensor_data)

        self.create_subscription(
            Float32MultiArray, '/estimator_params',
            self._cb_estimator_params,
            QoSProfile(depth=1))

    # ── Publicadores ────────────────────────────────────────────────────────

    def _setup_pubs(self):
        # BEST_EFFORT depth=1: bearing topics se publican a 100 Hz; los suscriptores
        # sólo necesitan el dato más reciente. RELIABLE con depth>1 acumula mensajes
        # en cola durante picos de CPU → burst de datos stale al recuperarse.
        rel = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self._pub_filt = self.create_publisher(
            BearingMeasurementsArray, '/vision_bearings', rel)
        self._pub_raw = self.create_publisher(
            BearingMeasurementsArray, '/vision_bearings_raw', rel)
        self._pub_dot = self.create_publisher(
            BearingMeasurementsArray, '/vision_bearings_dot', rel)

        if self.comparison_mode:
            self._pub_comp = {
                ESTIMATOR_KF:    self.create_publisher(
                    BearingMeasurementsArray, '/vision_bearings_kalman', rel),
                ESTIMATOR_HIN:   self.create_publisher(
                    BearingMeasurementsArray, '/vision_bearings_hinf', rel),
            }

    def _cb_estimator_params(self, msg: Float32MultiArray):
        from uav_estimators.bearing_estimators import PBOUNDS  # importar PBOUNDS
        keys = list(PBOUNDS.get(self.estimator_type, {}).keys())
        if len(msg.data) != len(keys):
            self.get_logger().warn(
                f'estimator_params: esperaba {len(keys)} valores, '
                f'recibí {len(msg.data)}')
            return
        self._est_kwargs = dict(zip(keys, [float(v) for v in msg.data]))
        self.get_logger().info(
            f'🔧 Nuevos params estimador: {self._est_kwargs}')
        # Limpiar estimadores activos → se recrean con nuevos kwargs
        for shard in self.shards.values():
            with shard.lock:
                shard.est_map.clear()

    # ── Callback de imagen ──────────────────────────────────────────────────

    def _image_callback(self, msg: Image, observer: str):
        with self._proc_locks[observer]:
            if self._processing[observer]:
                return
            self._processing[observer] = True
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if img is None or img.size == 0:
                return
            h, w = img.shape[:2]
            if w != self.img_w or h != self.img_h:
                img = cv2.resize(img, (self.img_w, self.img_h))

            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            shard = self.shards[observer]
            self._process_frame(img, observer, ts, shard)

            with self._stat_lock:
                self._frames_proc += 1

        except Exception as e:
            self.get_logger().error(f'image_callback [{observer}]: {e}')
        finally:
            with self._proc_locks[observer]:
                self._processing[observer] = False

    # ── Procesamiento de frame ──────────────────────────────────────────────

    def _process_frame(self, image: np.ndarray, observer: str,
                       ts: float, shard: ObserverShard):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detected_any = False

        for detector, _dict_name in self.detectors:
            corners, ids, _ = detector.detectMarkers(gray)
            if ids is None or len(ids) == 0:
                continue

            for i, tag_id in enumerate(ids.flatten()):
                target = self.id_to_name.get(int(tag_id))
                if target is None or target == observer:
                    continue

                # Filtro de conexiones
                if self._allowed and (observer, target) not in self._allowed:
                    continue

                # PnP
                ok, rvec, tvec = cv2.solvePnP(
                    self.obj_pts, corners[i], self.K, self.D,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue

                # Transformación al marco del robot
                t_rob = self.R_cam2rob @ tvec.flatten()
                dist = norm(t_rob)
                if dist < 1e-6 or dist > self.max_dist:
                    continue

                # Bearing medido crudo
                meas = t_rob / dist

                # Guardar medición cruda
                with shard.lock:
                    shard.raw_results[target] = BearingResult(
                        bearing=meas.copy(),
                        bearing_dot=np.zeros(3),
                        confidence=1.0,
                        uncertainty=0.0,
                        from_prediction=False,
                        meas_ts=ts,
                        dist=dist)

                # Crear estimador si es el primer avistamiento
                with shard.lock:
                    if target not in shard.est_map:
                        try:
                            shard.est_map[target] = make_estimator(
                                self.estimator_type, **self._est_kwargs)
                            if shard.est_map[target] is None:
                                self.get_logger().error(
                                    f'Failed to create estimator for {target}')
                                continue
                        except Exception as e:
                            self.get_logger().error(
                                f'Error creating estimator: {e}')
                            continue
                    est = shard.est_map[target]

                # Actualizar estimador
                try:
                    bearing_3d, bdot = est.bearing_and_dot(meas, ts, dist)
                except Exception as e:
                    self.get_logger().error(f'Error updating estimator: {e}')
                    continue

                with shard.lock:
                    shard.results[target] = BearingResult(
                        bearing=bearing_3d,
                        bearing_dot=bdot,
                        confidence=est.confidence(ts),
                        uncertainty=est.uncertainty(),
                        from_prediction=False,
                        meas_ts=ts,
                        dist=dist)

                # ── Estimadores de comparativa (misma medición, mismo instante) ──
                if self.comparison_mode:
                    with shard.lock:
                        if target not in shard.est_comp:
                            shard.est_comp[target] = {}
                            shard.res_comp[target] = {}
                        for etype, kwargs in self._all_est_kwargs.items():
                            if etype not in shard.est_comp[target]:
                                try:
                                    shard.est_comp[target][etype] = make_estimator(
                                        etype, **kwargs)
                                except Exception as exc:
                                    self.get_logger().error(
                                        f'Error creando est. comp [{etype}]: {exc}')

                    for etype, est_c in list(shard.est_comp.get(target, {}).items()):
                        try:
                            b_c, bd_c = est_c.bearing_and_dot(meas, ts, dist)
                        except Exception as exc:
                            self.get_logger().error(
                                f'Error actualizando est. comp [{etype}]: {exc}')
                            continue
                        with shard.lock:
                            shard.res_comp[target][etype] = BearingResult(
                                bearing=b_c,
                                bearing_dot=bd_c,
                                confidence=est_c.confidence(ts),
                                uncertainty=est_c.uncertainty(),
                                from_prediction=False,
                                meas_ts=ts,
                                dist=dist)

                detected_any = True
                with self._stat_lock:
                    self._markers_det += 1

        if not detected_any:
            with self._stat_lock:
                self._frames_nodet += 1

    # ── Publicación periódica ───────────────────────────────────────────────

    def _publish_bearings(self):
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        fmsg = BearingMeasurementsArray()
        fmsg.header.stamp = now.to_msg()
        fmsg.header.frame_id = 'world'

        rmsg = BearingMeasurementsArray()
        rmsg.header.stamp = now.to_msg()
        rmsg.header.frame_id = 'world'

        dmsg = BearingMeasurementsArray()
        dmsg.header.stamp = now.to_msg()
        dmsg.header.frame_id = 'world'

        for observer in self.robot_names:
            obs_idx = self.robot_index.get(observer)
            if obs_idx is None:
                continue
            shard = self.shards[observer]

            with shard.lock:
                est_snap = dict(shard.est_map)
                res_snap = {}
                raw_res_snap = {}
                for k, v in shard.results.items():
                    res_snap[k] = BearingResult(
                        bearing=v.bearing.copy(),
                        bearing_dot=v.bearing_dot.copy(),
                        confidence=v.confidence,
                        uncertainty=v.uncertainty,
                        from_prediction=v.from_prediction,
                        meas_ts=v.meas_ts,
                        dist=v.dist)
                    if not v.from_prediction:
                        v.from_prediction = True
                for k, v in shard.raw_results.items():
                    raw_res_snap[k] = BearingResult(
                        bearing=v.bearing.copy(),
                        bearing_dot=v.bearing_dot.copy(),
                        confidence=v.confidence,
                        uncertainty=v.uncertainty,
                        from_prediction=v.from_prediction,
                        meas_ts=v.meas_ts,
                        dist=v.dist)

            # ── /vision_bearings  (bearing filtrado + ḃ) ─────────────────────
            for target, est in est_snap.items():
                if not est.initialized:
                    continue
                tgt_idx = self.robot_index.get(target)
                if tgt_idx is None:
                    continue

                res = res_snap.get(target)
                fresh = (res is not None) and (not res.from_prediction)

                if not fresh:
                    # Predicción pura entre frames
                    try:
                        if not est.predict_only(now_sec):
                            continue
                        b = est.position
                        if b is None or norm(b) < 1e-6:
                            continue
                        b = b / norm(b)
                        Pg = np.eye(3) - np.outer(b, b)
                        bd = Pg @ est.velocity
                        res = BearingResult(
                            bearing=b,
                            bearing_dot=bd,
                            confidence=est.confidence(now_sec),
                            uncertainty=est.uncertainty(),
                            from_prediction=True,
                            meas_ts=res.meas_ts if res is not None else 0.0,
                            dist=res.dist if res is not None else 0.0)
                        with shard.lock:
                            shard.results[target] = res
                    except Exception as e:
                        self.get_logger().error(
                            f'Prediction error for {target}: {e}')
                        continue

                # Guard NaN/inf: un estimador divergente puede producir
                # valores no finitos que volcarían NaN al controlador y
                # crashearían Gazebo con "Invalid joint velocity [-nan]".
                if (not np.all(np.isfinite(res.bearing))
                        or not np.all(np.isfinite(res.bearing_dot))):
                    self.get_logger().warn(
                        f'NaN/inf en bearing {observer}→{target}: '
                        f'b={res.bearing}  bd={res.bearing_dot}  — '
                        f'medición descartada')
                    continue

                # Mensaje bearing filtrado
                mf = BearingMeasurement()
                mf.observer_id = obs_idx
                mf.target_id = tgt_idx
                mf.bearing_vector.x = float(res.bearing[0])
                mf.bearing_vector.y = float(res.bearing[1])
                mf.bearing_vector.z = float(res.bearing[2])
                mf.bearing_vector_dot.x = float(res.bearing_dot[0])
                mf.bearing_vector_dot.y = float(res.bearing_dot[1])
                mf.bearing_vector_dot.z = float(res.bearing_dot[2])
                mf.distance = float(res.dist)
                mf.uncertainty = float(res.uncertainty)
                mf.timestamp = now.to_msg()
                fmsg.measurements.append(mf)

                # Mensaje bearing_dot aislado
                md = BearingMeasurement()
                md.observer_id = obs_idx
                md.target_id = tgt_idx
                md.bearing_vector.x = float(res.bearing_dot[0])
                md.bearing_vector.y = float(res.bearing_dot[1])
                md.bearing_vector.z = float(res.bearing_dot[2])
                md.bearing_vector_dot.x = 0.0
                md.bearing_vector_dot.y = 0.0
                md.bearing_vector_dot.z = 0.0
                md.timestamp = now.to_msg()
                dmsg.measurements.append(md)

            # ── /vision_bearings_raw  (medición cruda) ───────────────────────
            for target, raw_res in raw_res_snap.items():
                tgt_idx = self.robot_index.get(target)
                if tgt_idx is None:
                    continue
                mr = BearingMeasurement()
                mr.observer_id = obs_idx
                mr.target_id = tgt_idx
                mr.bearing_vector.x = float(raw_res.bearing[0])
                mr.bearing_vector.y = float(raw_res.bearing[1])
                mr.bearing_vector.z = float(raw_res.bearing[2])
                mr.bearing_vector_dot.x = 0.0
                mr.bearing_vector_dot.y = 0.0
                mr.bearing_vector_dot.z = 0.0
                mr.distance = float(raw_res.dist)
                mr.timestamp = now.to_msg()
                rmsg.measurements.append(mr)

        if fmsg.measurements:
            self._pub_filt.publish(fmsg)
        if rmsg.measurements:
            self._pub_raw.publish(rmsg)
        if dmsg.measurements:
            self._pub_dot.publish(dmsg)

        if self.comparison_mode:
            for etype, pub in self._pub_comp.items():
                cmsg, cdmsg = self._collect_for_type(etype, now, now_sec)
                if cmsg.measurements:
                    pub.publish(cmsg)

    def _collect_for_type(
            self, est_type: str, now, now_sec: float
    ):
        """Construye BearingMeasurementsArray para un estimador de comparativa."""
        fmsg = BearingMeasurementsArray()
        fmsg.header.stamp = now.to_msg()
        fmsg.header.frame_id = 'world'

        dmsg = BearingMeasurementsArray()
        dmsg.header.stamp = now.to_msg()
        dmsg.header.frame_id = 'world'

        for observer in self.robot_names:
            obs_idx = self.robot_index.get(observer)
            if obs_idx is None:
                continue
            shard = self.shards[observer]

            with shard.lock:
                # Snapshot estimadores y resultados del tipo pedido
                est_snap = {
                    tgt: ests[est_type]
                    for tgt, ests in shard.est_comp.items()
                    if est_type in ests
                }
                res_snap: Dict[str, BearingResult] = {}
                for tgt, ests_res in shard.res_comp.items():
                    if est_type not in ests_res:
                        continue
                    r = ests_res[est_type]
                    res_snap[tgt] = BearingResult(
                        bearing=r.bearing.copy(),
                        bearing_dot=r.bearing_dot.copy(),
                        confidence=r.confidence,
                        uncertainty=r.uncertainty,
                        from_prediction=r.from_prediction,
                        meas_ts=r.meas_ts,
                        dist=r.dist)
                    if not r.from_prediction:
                        ests_res[est_type] = BearingResult(
                            bearing=r.bearing.copy(),
                            bearing_dot=r.bearing_dot.copy(),
                            confidence=r.confidence,
                            uncertainty=r.uncertainty,
                            from_prediction=True,
                            meas_ts=r.meas_ts,
                            dist=r.dist)

            for target, est_c in est_snap.items():
                if not est_c.initialized:
                    continue
                tgt_idx = self.robot_index.get(target)
                if tgt_idx is None:
                    continue

                res = res_snap.get(target)
                fresh = (res is not None) and (not res.from_prediction)

                if not fresh:
                    try:
                        if not est_c.predict_only(now_sec):
                            continue
                        b = est_c.position
                        if b is None or norm(b) < 1e-6:
                            continue
                        b = b / norm(b)
                        Pg = np.eye(3) - np.outer(b, b)
                        bd = Pg @ est_c.velocity
                        res = BearingResult(
                            bearing=b, bearing_dot=bd,
                            confidence=est_c.confidence(now_sec),
                            uncertainty=est_c.uncertainty(),
                            from_prediction=True,
                            meas_ts=res.meas_ts if res is not None else 0.0,
                            dist=res.dist if res is not None else 0.0)
                        with shard.lock:
                            if target in shard.res_comp:
                                shard.res_comp[target][est_type] = res
                    except Exception as e:
                        self.get_logger().error(
                            f'Predicción comp [{est_type}] {observer}→{target}: {e}')
                        continue

                if (not np.all(np.isfinite(res.bearing))
                        or not np.all(np.isfinite(res.bearing_dot))):
                    continue

                mf = BearingMeasurement()
                mf.observer_id = obs_idx
                mf.target_id = tgt_idx
                mf.bearing_vector.x = float(res.bearing[0])
                mf.bearing_vector.y = float(res.bearing[1])
                mf.bearing_vector.z = float(res.bearing[2])
                mf.bearing_vector_dot.x = float(res.bearing_dot[0])
                mf.bearing_vector_dot.y = float(res.bearing_dot[1])
                mf.bearing_vector_dot.z = float(res.bearing_dot[2])
                mf.distance = float(res.dist)
                mf.uncertainty = float(res.uncertainty)
                mf.timestamp = now.to_msg()
                fmsg.measurements.append(mf)

                md = BearingMeasurement()
                md.observer_id = obs_idx
                md.target_id = tgt_idx
                md.bearing_vector.x = float(res.bearing_dot[0])
                md.bearing_vector.y = float(res.bearing_dot[1])
                md.bearing_vector.z = float(res.bearing_dot[2])
                md.bearing_vector_dot.x = 0.0
                md.bearing_vector_dot.y = 0.0
                md.bearing_vector_dot.z = 0.0
                md.timestamp = now.to_msg()
                dmsg.measurements.append(md)

        return fmsg, dmsg

    # ── Estadísticas ────────────────────────────────────────────────────────

    def _log_stats(self):
        with self._stat_lock:
            p, nd, m = self._frames_proc, self._frames_nodet, self._markers_det
        self.get_logger().info(
            f'[BearingDetector/{self.estimator_type}] '
            f'proc={p}  no-det={nd}  markers={m}')
        for obs, shard in self.shards.items():
            with shard.lock:
                for tgt, est in shard.est_map.items():
                    if est.update_count > 0 or est.pred_count > 0:
                        self.get_logger().info(
                            f'  {obs}→{tgt}: '  f"upd={
                                est.update_count} '  f'pred={
                                est.pred_count} '  f'avg_dt={
                                est.avg_dt():.4f}s '  f'unc={
                                est.uncertainty():.4f} '  f'conf={
                                est.confidence(
                                    self.get_clock().now().nanoseconds *
                                    1e-9):.3f}")


# ═════════════════════════════════════════════════════════════════════════════
# MAIN
# ═════════════════════════════════════════════════════════════════════════════
def main():
    rclpy.init()
    node = BearingDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
