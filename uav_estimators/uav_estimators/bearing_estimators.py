#!/usr/bin/env python3
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""
bearing_estimators.py
=====================
Librería compartida con los dos estimadores de bearing en S².

Todos operan en el espacio tangente local T_{b_ref}S² y exponen
EXACTAMENTE la misma interfaz pública:

    est.bearing_and_dot(b_meas, ts, dist) → (b_3d, bdot_3d)
    est.predict_only(ts)                  → bool
    est.position                          → np.ndarray   bearing 3D
    est.velocity                          → np.ndarray   bearing_dot 3D
    est.confidence(now)                   → float  [0,1]
    est.uncertainty()                     → float
    est.avg_dt()                          → float
    est.initialized                       → bool
    est.update_count                      → int
    est.pred_count                        → int

Importar en cualquier nodo ROS2:
    from bearing_estimators import BearingKalmanFilter
    from bearing_estimators import BearingHinfFilter
    from bearing_estimators import make_estimator   # fábrica por nombre
"""

import numpy as np
from numpy.linalg import norm, inv, eigh
import threading
from collections import deque
from typing import Tuple

# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTES GLOBALES
# ─────────────────────────────────────────────────────────────────────────────
DEFAULT_PUB_HZ      = 30.0
KALMAN_MIN_DT       = 1e-4
KALMAN_MAX_DT       = 0.5
KALMAN_MAX_PRED_SEC = 2.0      # s sin medición antes de invalidar predicción
CONFIDENCE_TAU      = 1.5      # s, constante de tiempo de confianza

# Ruido del proceso — KF (optimizado Trial #89 Kalman)
KF_Q_VEL   = 0.07             # rad²/s³
KF_R_BASE  = 2e-5             # rad²
KF_R_SCALE = 7e-5             # rad²/m

# Ruido del proceso — H∞ (optimizado NSGA-II/CMA-ES H∞)
Q_BEARING_VEL = 0.023         # rad²/s³
R_BASE  = 6.6e-3              # rad²
R_SCALE = 5e-6                # rad²/m

# Compuerta de Mahalanobis (KF y H∞)
GATE_INIT   = 6.0
GATE_STEADY = 3.5
GATE_WARMUP = 8


# ═════════════════════════════════════════════════════════════════════════════
# BASE COMÚN: geometría S² en el espacio tangente
# ═════════════════════════════════════════════════════════════════════════════
class _TangentBase:
    """
    Geometría S² compartida por los tres estimadores.

    Vector de estado en R⁶  (mismo que el paper):
        x[0:2]  →  (δ₁, δ₂)   desplazamiento angular en plano tangente  [rad]
        x[2:4]  →  (ω₁, ω₂)   velocidad angular                         [rad/s]
        x[4:6]  →  (α₁, α₂)   aceleración angular                       [rad/s²]

    Marco ortonormal local {b_ref, t1, t2}:
        b_ref   →  punto de referencia sobre S² (origen del plano tangente)
        t1, t2  →  base ortonormal de T_{b_ref}S²  (Gram-Schmidt)
    """

    # Número de updates iniciales en que se devuelve la medición cruda en lugar
    # del estado estimado. Durante el warmup el filtro aprende b_ref pero zerear
    # velocidad/aceleración y los términos cruzados de P evita el pico al salir.
    _WARMUP_N: int = 5


    def __init__(self):
        self.lock = threading.Lock()

        # Estado y punto de referencia
        self.x     = np.zeros(6)
        self.b_ref = np.array([1.0, 0.0, 0.0])
        self.t1    = np.array([0.0, 1.0, 0.0])
        self.t2    = np.array([0.0, 0.0, 1.0])
        self.dist  = 1.0                       # distancia al marcador [m]

        # Contadores y tiempos
        self.initialized   = False
        self.last_meas_sec = 0.0
        self.last_pred_sec = 0.0
        self.update_count  = 0
        self.pred_count    = 0
        self._dt_hist      = deque(maxlen=60)

    # ── Geometría esférica ────────────────────────────────────────────────────

    def _update_tangent_basis(self):
        """
        Construye la base ortonormal {t1, t2} del plano tangente en b_ref
        mediante Gram-Schmidt (ec. 16-18 del paper).
        """
        b   = self.b_ref
        idx = int(np.argmin(np.abs(b)))        # eje menos alineado (ec. 16)
        e   = np.zeros(3); e[idx] = 1.0

        # t1: proyección de e sobre el plano tangente (ec. 17)
        t1 = e - b * (b @ e)
        n1 = norm(t1)
        if n1 < 1e-6:
            self.t1 = np.array([0.0, 1.0, 0.0])
            self.t2 = np.array([0.0, 0.0, 1.0])
        else:
            self.t1 = t1 / n1
            t2      = np.cross(b, self.t1)     # ec. 18
            self.t2 = t2 / max(norm(t2), 1e-6)

    def _bearing_3d(self) -> np.ndarray:
        """
        Reconstruye el bearing 3D desde el estado tangente (ec. 24 del paper):
            b = normalize(b_ref + δ₁·t1 + δ₂·t2)
        """
        b = self.b_ref + self.x[0] * self.t1 + self.x[1] * self.t2
        n = norm(b)
        return b / n if n > 1e-6 else self.b_ref.copy()

    def _bearing_dot_3d(self, b: np.ndarray) -> np.ndarray:
        """
        Derivada del bearing en R³ (ec. 32 del paper):
            ġ = P_g · (ω₁·t1 + ω₂·t2)
        donde P_g = I - g·gᵀ proyecta sobre T_g S² para garantizar g·ġ = 0.
        """
        bdot  = self.x[2] * self.t1 + self.x[3] * self.t2
        bdot -= b * (b @ bdot)                 # proyección geométrica
        return bdot

    def _project_meas(self, b_meas: np.ndarray) -> np.ndarray:
        """
        Innovación en el plano tangente (ec. 23 del paper):
            y = [b_meas·t1 - δ₁,  b_meas·t2 - δ₂]
        """
        return np.array([
            float(b_meas @ self.t1) - self.x[0],
            float(b_meas @ self.t2) - self.x[1],
        ])

    def _retract(self, P: np.ndarray = None):
        """
        Retracción: recentra el plano tangente en el bearing estimado actual
        (Sección 6 del paper — 'Recentrado del Plano Tangente').

        1. Reconstruye b_new desde el estado.
        2. Convierte ω, α a vectores 3D.
        3. Actualiza b_ref y recalcula {t1, t2}.
        4. Re-expresa ω y α en la nueva base; reinicia δ₁=δ₂=0.
        5. Estabiliza P[:2,:2] si se pasa (garantiza def. positiva).
        """
        b_new = self._bearing_3d()
        w_vec = self.x[2] * self.t1 + self.x[3] * self.t2   # ω en R³
        a_vec = self.x[4] * self.t1 + self.x[5] * self.t2   # α en R³

        self.b_ref = b_new.copy()
        self._update_tangent_basis()

        # Reiniciar posición tangente; re-expresar ω y α
        self.x[0] = 0.0;  self.x[1] = 0.0
        self.x[2] = float(w_vec @ self.t1)
        self.x[3] = float(w_vec @ self.t2)
        self.x[4] = float(a_vec @ self.t1)
        self.x[5] = float(a_vec @ self.t2)

        if P is not None:
            # Estabilizar P_pos para mantener definición positiva
            eigvals, eigvecs = eigh(P[:2, :2])
            eigvals = np.maximum(eigvals, 1e-5)
            P[:2, :2] = eigvecs @ np.diag(eigvals) @ eigvecs.T
            P[:] = (P + P.T) * 0.5

    def _warmup_reset(self):
        """
        Llamado después de cada update interno durante el warmup.
        Implementación base: solo zerear velocidad y aceleración.
        KF y H∞ sobreescriben esto para también resetear los términos
        cruzados de P, que de otro modo crecen en cada predicción y
        producen ganancias K enormes (especialmente K_hinf = P_inf·H^T·R^{-1})
        al salir del warmup.
        """
        self.x[2:] = 0.0

    def _init_base(self, b: np.ndarray, ts: float):
        """Inicialización común: fija b_ref y construye la base tangente."""
        self.b_ref = b.copy()
        self._update_tangent_basis()
        self.x[:] = 0.0
        self.last_meas_sec = self.last_pred_sec = ts
        self.initialized = True

    # ── Matrices del modelo cinemático ────────────────────────────────────────

    @staticmethod
    def _make_A(dt: float) -> np.ndarray:
        """
        Matriz de transición de estado (ec. 8 del paper).
        Modelo: aceleración constante en el plano tangente.
        """
        A = np.eye(6)
        A[0, 2] = dt;  A[0, 4] = 0.5 * dt**2
        A[1, 3] = dt;  A[1, 5] = 0.5 * dt**2
        A[2, 4] = dt
        A[3, 5] = dt
        return A

    @staticmethod
    def _make_Q(dt: float, q: float = Q_BEARING_VEL) -> np.ndarray:
        """
        Ruido de proceso (ec. 9 del paper).
        Modelo: aceleración angular como ruido blanco de intensidad q.
        """
        Q = np.zeros((6, 6))
        for i in range(2):
            pi, vi, ai = i, 2 + i, 4 + i
            Q[pi, pi] = q * dt**4 / 4.0
            Q[pi, vi] = q * dt**3 / 2.0
            Q[pi, ai] = q * dt**2 / 2.0
            Q[vi, pi] = Q[pi, vi]
            Q[vi, vi] = q * dt**2
            Q[vi, ai] = q * dt
            Q[ai, pi] = Q[pi, ai]
            Q[ai, vi] = Q[vi, ai]
            Q[ai, ai] = q
        return Q

    # ── API pública común ─────────────────────────────────────────────────────

    def bearing_and_dot(self, b_meas: np.ndarray, ts: float,
                        dist: float = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Punto de entrada principal. Procesa una medición y devuelve
        (bearing_3D, bearing_dot_3D) filtrados.

        Durante los primeros _WARMUP_N updates el filtro actualiza su estado
        internamente pero devuelve la medición cruda con bearing_dot=0.
        Esto evita los picos transitorios causados por la covarianza inicial
        grande y el acoplamiento posición→velocidad en la primera predicción.

        """
        with self.lock:
            # Capturar el contador ANTES del update: _update_internal lo incrementa,
            # y sin esto el último paso de warmup (count 4→5) nunca ejecuta el reset.
            count_before = self.update_count
            self._update_internal(b_meas, ts, dist)
            # Warmup: b_ref converge al bearing real; P y velocidad se resetean
            # para que la primera salida del filtro tenga vel≈0 y P en régimen
            # permanente (sin transitorio, especialmente para H∞).
            if count_before < self._WARMUP_N:
                self._warmup_reset()
                n = norm(b_meas)
                b_raw = b_meas / n if n > 1e-6 else b_meas.copy()
                return b_raw, np.zeros(3)
            b    = self._bearing_3d()
            bdot = self._bearing_dot_3d(b)
            return b, bdot

    def predict_only(self, ts: float) -> bool:
        """
        Propaga el estado sin medición (entre frames).
        Devuelve False si el filtro no está inicializado o lleva más de
        KALMAN_MAX_PRED_SEC sin medición.
        """
        with self.lock:
            if not self.initialized:
                return False
            if (ts - self.last_meas_sec) > KALMAN_MAX_PRED_SEC:
                return False
            dt = float(np.clip(ts - self.last_pred_sec,
                               KALMAN_MIN_DT, KALMAN_MAX_DT))
            self._predict(dt)
            self.last_pred_sec = ts
            self.pred_count   += 1
            return True

    @property
    def position(self) -> np.ndarray:
        with self.lock:
            return self._bearing_3d()

    @property
    def velocity(self) -> np.ndarray:
        with self.lock:
            return self._bearing_dot_3d(self._bearing_3d())

    def confidence(self, now: float) -> float:
        """Decaimiento exponencial desde la última medición aceptada."""
        with self.lock:
            if not self.initialized:
                return 0.0
            return float(np.exp(-(now - self.last_meas_sec) / CONFIDENCE_TAU))

    def avg_dt(self) -> float:
        with self.lock:
            return (float(np.mean(self._dt_hist))
                    if self._dt_hist else 1.0 / DEFAULT_PUB_HZ)

    # ── Métodos que cada subclase debe implementar ────────────────────────────

    def _update_internal(self, b_meas: np.ndarray, ts: float,
                         dist: float) -> None:
        raise NotImplementedError

    def _predict(self, dt: float) -> None:
        raise NotImplementedError

    def uncertainty(self) -> float:
        raise NotImplementedError


# ═════════════════════════════════════════════════════════════════════════════
# OPCIÓN 1 — Filtro de Kalman lineal en el espacio tangente (baseline)
# ═════════════════════════════════════════════════════════════════════════════
class BearingKalmanFilter(_TangentBase):
    """
    Filtro de Kalman Lineal en T_{b_ref}S²  (Secciones 5-7 del paper).

    Estado: x = [δ₁, δ₂, ω₁, ω₂, α₁, α₂]  ∈ R⁶
    Medición: z_tan = [b·t1, b·t2]           ∈ R²
    Ciclo: predicción → innovación → compuerta Mahalanobis → corrección
           (forma de Joseph) → retracción.

    Parámetros
    ──────────
    q_vel   : intensidad del ruido de proceso  [rad²/s³]  (ec. 9)
    r_base  : ruido de medición base           [rad²]     (ec. 10)
    r_scale : escala por distancia             [rad²/m]   (ec. 10)

    Ventajas
    ────────
    ✅ Óptimo bajo ruido gaussiano
    ✅ Covarianza P tiene interpretación probabilística exacta
    ✅ Un solo bloque de sintonía: Q (proceso) y R (medición)

    Limitaciones
    ────────────
    ⚠️  Sensible a perturbaciones impulsivas (outliers que pasan la compuerta)
    ⚠️  Convergencia asintótica (no garantiza tiempo finito)
    """

    def __init__(self,
                 q_vel:   float = KF_Q_VEL,
                 r_base:  float = KF_R_BASE,
                 r_scale: float = KF_R_SCALE):
        super().__init__()
        self.q_vel   = q_vel
        self.r_base  = r_base
        self.r_scale = r_scale

        # Covarianza inicial escalada a sqrt(q_vel) para que los acoplamientos
        # posición→velocidad en la primera predicción sean proporcionales al
        # ruido de proceso real y no causen picos transitorios.
        sq = float(np.sqrt(q_vel))
        s_pos = 0.5
        s_vel = float(np.clip(8.0 * sq, 0.05, 2.0))
        s_acc = float(np.clip(15.0 * sq, 0.10, 4.0))
        self.P = np.diag([s_pos, s_pos, s_vel, s_vel, s_acc, s_acc])
        self._p_vel_init = s_vel
        self._p_acc_init = s_acc

        # Matriz de observación H  ∈ R^{2×6}  (ec. 11 del paper)
        self.H = np.zeros((2, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0

    def _warmup_reset(self):
        """Durante warmup: resetear P completo y zerear vel/acc.
        Preservar solo P_pos crea K[2,0]=P[0,2]/(P[0,0]+r) enorme porque
        P_pos converge a ~0.001 pero P_vel se resetea a s_vel=2.0 →
        P[0,2]/P[0,0] >> 1. Resetear P_pos a s_pos=0.5 mantiene la razón
        P[0,2]/P[0,0] consistente con los valores iniciales de diseño."""
        self.x[2:] = 0.0
        self.P[:] = 0.0
        self.P[0, 0] = 0.5
        self.P[1, 1] = 0.5
        self.P[2, 2] = self._p_vel_init
        self.P[3, 3] = self._p_vel_init
        self.P[4, 4] = self._p_acc_init
        self.P[5, 5] = self._p_acc_init

    # ── Ciclo del filtro ──────────────────────────────────────────────────────

    def _predict(self, dt: float):
        """
        Paso de predicción (ec. 12-13 del paper):
            x_{k|k-1} = A·x_{k-1|k-1}
            P_{k|k-1} = A·P·Aᵀ + Q
        """
        A = self._make_A(dt)
        Q = self._make_Q(dt, self.q_vel)
        self.x = A @ self.x
        self.P = A @ self.P @ A.T + Q
        self.P = (self.P + self.P.T) * 0.5    # simetrizar

    def _update_internal(self, b_meas: np.ndarray, ts: float,
                         dist: float = None):
        n = norm(b_meas)
        if n < 1e-6:
            return
        b_meas = b_meas / n

        if dist is not None and dist > 1e-3:
            self.dist = dist

        if not self.initialized:
            self._init_base(b_meas, ts)
            return

        dt = float(np.clip(ts - self.last_meas_sec,
                            KALMAN_MIN_DT, KALMAN_MAX_DT))
        self._predict(dt)

        # Innovación (ec. 14)
        y = self._project_meas(b_meas)

        # Ruido de medición escalado por distancia (ec. 10)
        R = np.eye(2) * (self.r_base + self.r_scale * self.dist)

        # Covarianza de innovación (ec. 15)
        S = self.H @ self.P @ self.H.T + R

        # Compuerta de Mahalanobis (ec. 17-18 del paper)
        try:
            Si   = inv(S)
            maha = float(np.sqrt(max(0.0, y @ Si @ y)))
        except np.linalg.LinAlgError:
            maha = 0.0

        gate = GATE_INIT if self.update_count < GATE_WARMUP else GATE_STEADY
        if maha > gate:
            # Medición rechazada: solo actualizamos tiempos
            self.last_meas_sec = self.last_pred_sec = ts
            self._dt_hist.append(dt)
            return

        # Ganancia de Kalman (ec. 19)
        K = self.P @ self.H.T @ Si

        # Corrección del estado (ec. 20)
        self.x += K @ y

        # Corrección de covarianza — forma de Joseph (ec. 21)
        IKH    = np.eye(6) - K @ self.H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

        # Retracción a S² (Sección 6)
        self._retract(self.P)

        self.last_meas_sec = self.last_pred_sec = ts
        self._dt_hist.append(dt)
        self.update_count += 1
        self.pred_count    = 0

    def uncertainty(self) -> float:
        with self.lock:
            return float(np.sqrt(np.mean(np.diag(self.P[:2, :2]))))


# ═════════════════════════════════════════════════════════════════════════════
# OPCIÓN 2 — Filtro H-infinito en el espacio tangente
# ═════════════════════════════════════════════════════════════════════════════
class BearingHinfFilter(_TangentBase):
    """
    Filtro H∞ (H-infinito) en T_{b_ref}S².

    FUNDAMENTO MATEMÁTICO
    ─────────────────────
    El KF minimiza E[‖error‖²] asumiendo ruido gaussiano.
    El H∞ minimiza el *peor caso* de la razón de energías:

        sup_{w≠0}  ‖error‖² / ‖w‖²  <  γ²

    para CUALQUIER perturbación w de energía finita, sin asumir distribución.
    Esto lo hace robusto ante vibraciones de hélices, ráfagas de viento y
    cualquier perturbación acotada no modelada.

    DIFERENCIA CON KF: ECUACIÓN DE RICCATI MODIFICADA
    ──────────────────────────────────────────────────
    Dado P_{k|k-1} (predicción), la ganancia H∞ se obtiene resolviendo:

        M = P⁻¹  −  (1/γ²)·L  +  Hᵀ·R⁻¹·H

    donde el término  −(1/γ²)·L  es la corrección adversarial que hace
    el filtro "pesimista" y por eso robusto. Luego:

        P_hinf = M⁻¹
        K_hinf = P_hinf · Hᵀ · R⁻¹

    Cuando γ → ∞ el término adversarial desaparece y se recupera el KF.

    PARÁMETRO γ (gamma)
    ───────────────────
        γ grande  →  comportamiento cercano al KF  (menos conservador)
        γ pequeño →  más robusto, más lag
        Rango recomendado para UAVs: γ ∈ [1.5, 5.0]
        Valor por defecto: γ = 2.0

    CONDICIÓN DE EXISTENCIA
    ───────────────────────
    M debe ser definida positiva. Si no lo es (γ demasiado pequeño para
    la covarianza actual), el filtro hace fallback al KF estándar
    automáticamente.

    Ventajas
    ────────
    ✅ Robusto ante perturbaciones acotadas sin asumir gaussianidad
    ✅ Misma estructura que el KF: solo cambia el cálculo de la ganancia
    ✅ Un solo parámetro extra (γ) respecto al KF

    Limitaciones
    ────────────
    ⚠️  Más conservador que el KF en condiciones ideales (más lag)
    ⚠️  γ requiere sintonía manual; muy pequeño → fallback al KF
    ✅  L sintonizable — l_pos, l_vel, l_acc forman parte del espacio del optimizador
    """

    def __init__(self,
                 gamma:   float = 2.0,
                 q_vel:   float = Q_BEARING_VEL,
                 r_base:  float = R_BASE,
                 r_scale: float = R_SCALE,
                 l_pos:   float = 1.0,
                 l_vel:   float = 0.5,
                 l_acc:   float = 0.1):
        super().__init__()
        self.gamma   = gamma
        self.q_vel   = q_vel
        self.r_base  = r_base
        self.r_scale = r_scale

        # Covarianza inicial escalada a sqrt(q_vel) — misma lógica que KF.
        # Reduce el acoplamiento posición→velocidad en la primera predicción,
        # que es la causa de los picos iniciales del H∞.
        sq = float(np.sqrt(q_vel))
        s_pos = 0.5
        s_vel = float(np.clip(8.0 * sq, 0.05, 2.0))
        s_acc = float(np.clip(15.0 * sq, 0.10, 4.0))
        self.P = np.diag([s_pos, s_pos, s_vel, s_vel, s_acc, s_acc])
        self._p_vel_init = s_vel
        self._p_acc_init = s_acc

        # Matriz de observación (igual que KF)
        self.H = np.zeros((2, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0

        # Matriz de penalización L — ahora sintonizable
        # Penaliza qué errores de estado le importan minimizar al H∞
        self.L = np.diag([l_pos, l_pos, l_vel, l_vel, l_acc, l_acc])

        # Diagnóstico: fracción de pasos donde H∞ cae en KF fallback
        self.fallback_count = 0

        # Pre-computar P estacionaria iterando la ecuación de Riccati H∞
        # con dt nominal (1/15 s). De esta forma _warmup_reset arranca P en
        # su valor de régimen permanente y no hay transitorio al salir del warmup.
        self._P_steady = self._compute_steady_P(dt_nom=1.0 / 15.0, n_iter=200)

    # ── Precomputación de P estacionaria ─────────────────────────────────────

    def _compute_steady_P(self, dt_nom: float, n_iter: int) -> np.ndarray:
        """
        Itera la ecuación de Riccati H∞ discreta n_iter veces con dt fijo
        hasta obtener P_∞. Coste: n_iter multiplicaciones de matrices 6×6
        ejecutadas una sola vez en __init__ → sin impacto en tiempo real.

            P_{k+1} = A · (P_k^{-1} - γ^{-2}·L + H^T·R^{-1}·H)^{-1} · A^T + Q
        """
        A      = self._make_A(dt_nom)
        Q      = self._make_Q(dt_nom, self.q_vel)
        R_nom  = np.eye(2) * self.r_base          # distancia nominal → solo r_base
        R_inv  = inv(R_nom)
        gamma2 = self.gamma ** 2

        HtRiH  = self.H.T @ R_inv @ self.H        # 6×6, constante
        Lscale = (1.0 / gamma2) * self.L          # 6×6, constante

        # Partir de la covarianza inicial de diseño
        P = self.P.copy()

        for _ in range(n_iter):
            try:
                M = inv(P) - Lscale + HtRiH
                eigvals = np.linalg.eigvalsh(M)
                if np.any(eigvals <= 1e-10):
                    # Fallback KF un paso para mantener P acotada
                    S     = self.H @ P @ self.H.T + R_nom
                    K     = P @ self.H.T @ inv(S)
                    IKH   = np.eye(6) - K @ self.H
                    P_upd = IKH @ P @ IKH.T + K @ R_nom @ K.T
                else:
                    P_upd = inv(M)
            except np.linalg.LinAlgError:
                break
            P_upd = (P_upd + P_upd.T) * 0.5
            P     = A @ P_upd @ A.T + Q
            P     = (P + P.T) * 0.5

        return P

    def _warmup_reset(self):
        """Durante warmup: zerear vel/acc y restaurar P al valor estacionario.
        Usar P_∞ (precomputada) en lugar de la covarianza inicial de diseño
        elimina el transitorio de H∞ al salir del warmup: K_hinf = P_inf·H^T·R^{-1}
        ya está en régimen permanente desde el primer paso activo."""
        self.x[2:] = 0.0
        self.P[:] = self._P_steady

    # ── Predicción (idéntica al KF) ───────────────────────────────────────────

    def _predict(self, dt: float):
        A = self._make_A(dt)
        Q = self._make_Q(dt, self.q_vel)
        self.x = A @ self.x
        self.P = A @ self.P @ A.T + Q
        self.P = (self.P + self.P.T) * 0.5

    # ── Ganancia H∞ ───────────────────────────────────────────────────────────

    def _hinf_gain(self, R_meas: np.ndarray):
        """
        Resuelve la ecuación de Riccati modificada H∞:

            M = P⁻¹  −  (1/γ²)·L  +  Hᵀ·R⁻¹·H
            P_hinf = M⁻¹
            K_hinf = P_hinf · Hᵀ · R⁻¹

        Si M no es definida positiva → fallback al KF (γ demasiado pequeño).

        Returns
        ───────
        K       : ganancia  ∈ R^{6×2}
        P_new   : covarianza actualizada  ∈ R^{6×6}
        fallback: True si se usó KF en lugar de H∞
        """
        gamma2 = self.gamma ** 2
        R_inv  = inv(R_meas)

        # Riccati modificada: término adversarial -(1/γ²)·L
        try:
            M = inv(self.P) - (1.0 / gamma2) * self.L + self.H.T @ R_inv @ self.H
        except np.linalg.LinAlgError:
            M = None

        need_fallback = (M is None)
        if not need_fallback:
            eigvals = np.linalg.eigvalsh(M)
            if np.any(eigvals <= 1e-10):
                need_fallback = True

        if need_fallback:
            # KF fallback — se usa también la actualización de Joseph para que P
            # converja igual que un KF real (bug original: devolvía self.P sin actualizar)
            S = self.H @ self.P @ self.H.T + R_meas
            try:
                Si = inv(S)
                K  = self.P @ self.H.T @ Si
            except np.linalg.LinAlgError:
                K = np.zeros((6, 2))
                return K, self.P.copy(), True
            IKH   = np.eye(6) - K @ self.H
            P_kf  = IKH @ self.P @ IKH.T + K @ R_meas @ K.T
            P_kf  = (P_kf + P_kf.T) * 0.5
            return K, P_kf, True

        P_hinf = inv(M)
        P_hinf = (P_hinf + P_hinf.T) * 0.5    # simetrizar
        K      = P_hinf @ self.H.T @ R_inv

        # inv() puede producir NaN en matrices mal condicionadas sin lanzar
        # LinAlgError. Si K o P_hinf contienen NaN/inf → fallback al KF.
        if not (np.all(np.isfinite(K)) and np.all(np.isfinite(P_hinf))):
            S = self.H @ self.P @ self.H.T + R_meas
            try:
                Si = inv(S)
                K  = self.P @ self.H.T @ Si
            except np.linalg.LinAlgError:
                K = np.zeros((6, 2))
                return K, self.P.copy(), True
            IKH   = np.eye(6) - K @ self.H
            P_kf  = IKH @ self.P @ IKH.T + K @ R_meas @ K.T
            P_kf  = (P_kf + P_kf.T) * 0.5
            return K, P_kf, True

        return K, P_hinf, False

    # ── Corrección ────────────────────────────────────────────────────────────

    def _update_internal(self, b_meas: np.ndarray, ts: float,
                         dist: float = None):
        n = norm(b_meas)
        if n < 1e-6:
            return
        b_meas = b_meas / n

        if dist is not None and dist > 1e-3:
            self.dist = dist

        if not self.initialized:
            self._init_base(b_meas, ts)
            return

        dt = float(np.clip(ts - self.last_meas_sec,
                            KALMAN_MIN_DT, KALMAN_MAX_DT))
        self._predict(dt)

        y = self._project_meas(b_meas)
        R = np.eye(2) * (self.r_base + self.r_scale * self.dist)

        # Compuerta de Mahalanobis (misma que KF)
        S = self.H @ self.P @ self.H.T + R
        try:
            Si   = inv(S)
            maha = float(np.sqrt(max(0.0, y @ Si @ y)))
        except np.linalg.LinAlgError:
            maha = 0.0

        gate = GATE_INIT if self.update_count < GATE_WARMUP else GATE_STEADY
        if maha > gate:
            self.last_meas_sec = self.last_pred_sec = ts
            self._dt_hist.append(dt)
            return

        # Ganancia H∞ (con fallback automático al KF si γ es muy pequeño)
        K, P_new, _fallback = self._hinf_gain(R)
        if _fallback:
            self.fallback_count += 1

        x_new = self.x + K @ y
        # Guardia final: si la actualización produce NaN/inf descartarla
        if not np.all(np.isfinite(x_new)):
            self.last_meas_sec = self.last_pred_sec = ts
            self._dt_hist.append(dt)
            return
        self.x = x_new
        self.P = P_new

        self._retract(self.P)

        self.last_meas_sec = self.last_pred_sec = ts
        self._dt_hist.append(dt)
        self.update_count += 1
        self.pred_count    = 0

    def uncertainty(self) -> float:
        with self.lock:
            return float(np.sqrt(np.mean(np.diag(self.P[:2, :2]))))

    def fallback_rate(self) -> float:
        """Fracción de updates que cayeron en KF fallback (diagnóstico)."""
        with self.lock:
            total = self.update_count + self.fallback_count
            return self.fallback_count / total if total > 0 else 0.0


# ═════════════════════════════════════════════════════════════════════════════
# FÁBRICA — cambia estimador con una sola línea
# ═════════════════════════════════════════════════════════════════════════════
ESTIMATOR_KF   = 'kalman'
ESTIMATOR_HIN  = 'hinf'

# Rangos de búsqueda para el optimizador
PBOUNDS = {
    ESTIMATOR_KF: {
        "q_vel":   (0.01, 0.3),
        "r_base":  (1e-5, 0.01),
        "r_scale": (1e-6, 0.001),
    },
    ESTIMATOR_HIN: {
        "gamma":   (1.5,  5.0),
        "q_vel":   (0.01, 0.4),
        "r_base":  (1e-3, 0.1),   # >= 1e-3: término adversarial no trivial frente a H^T R⁻¹ H
        "r_scale": (1e-5, 1e-2),
        "l_pos":   (0.2,  3.0),
        "l_vel":   (0.05, 1.0),
        "l_acc":   (0.005, 0.8),
    },
}


_REGISTRY = {
    ESTIMATOR_KF:    BearingKalmanFilter,
    ESTIMATOR_HIN:   BearingHinfFilter,
}

def make_estimator(kind: str = ESTIMATOR_KF, **kwargs) -> _TangentBase:
    """
    Crea un estimador por nombre.

    Uso en cualquier nodo ROS2:
        from bearing_estimators import make_estimator, ESTIMATOR_HIN
        est = make_estimator(ESTIMATOR_HIN, gamma=2.5)

    Args:
        kind  : 'kalman' | 'hinf'
        kwargs: parámetros del estimador (ver docstring de cada clase)
    """
    cls = _REGISTRY.get(kind)
    if cls is None:
        raise ValueError(
            f"Estimador desconocido: '{kind}'. "
            f"Opciones: {list(_REGISTRY.keys())}")
    return cls(**kwargs)