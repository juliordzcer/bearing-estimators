# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Functional tests for the two bearing estimators (KF, H-inf).

Each estimator must satisfy:
  1. Starts un-initialized; initializes on the first update.
  2. Output bearing always lies on S² (unit norm).
  3. Output bearing_dot is tangent to S² (b · bdot ≈ 0).
  4. Converges to a static target within a reasonable number of steps.
  5. make_estimator factory returns the correct type.

"""

import math

import numpy as np
import pytest
from uav_estimators.bearing_estimators import (
    BearingKalmanFilter,
    BearingHinfFilter,
    make_estimator,
)

# ── Helpers ───────────────────────────────────────────────────────────────────
RNG = np.random.default_rng(seed=42)
DT = 0.033   # 30 Hz


# Match optimizer: hover_stable_duration=4 s → 120 steps; evaluation_duration=30 s → 909 steps.


def _unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def _static_sequence(b_true, n=100, noise_std=0.005):
    """Synthetic bearing measurements around a fixed direction."""
    bearings = []
    for _ in range(n):
        noise = RNG.standard_normal(3) * noise_std
        bearings.append(_unit(b_true + noise))
    return bearings


def _circular_sequence(n=80, noise_std=0.01):
    """Bearings tracing a small circle on S²."""
    t = np.linspace(0, 2 * np.pi, n)
    raw = np.column_stack([np.cos(t) * 0.6, np.sin(t) * 0.6, np.ones(n) * 0.8])
    noisy = raw + RNG.standard_normal(raw.shape) * noise_std
    return [_unit(row) for row in noisy]


def _dynamic_bearing_sequence(n: int, noise_std: float = 0.015):
    """Slow circular trajectory on S², mimicking Gazebo UAV formation flight.

    Returns (true_bearings, noisy_bearings, timestamps).

    Period ≈ 30 s, amplitude 0.5 rad — representative of a Crazyflie pair
    flying a circle at 1–2 m/s relative speed seen from 2–3 m distance.
    noise_std=0.015 rad ≈ 0.86° matches typical ArUco detection noise in Gazebo.
    """
    t = np.linspace(0.0, n * DT, n)
    omega = 2.0 * math.pi / 30.0
    raw = np.column_stack([
        0.5 * np.cos(omega * t),
        0.5 * np.sin(omega * t),
        0.7 * np.ones(n),
    ])
    true_b  = [_unit(r) for r in raw]
    noisy_b = [_unit(r) for r in noisy]
    return true_b, noisy_b, t


# ── Parametric fixture: run every test against all three estimators ────────────
@pytest.fixture(params=["kalman", "hinf"])
def estimator(request):
    return make_estimator(request.param)


# ── Test suite ────────────────────────────────────────────────────────────────

class TestInitialization:
    def test_starts_uninitialized(self, estimator):
        assert not estimator.initialized

    def test_first_update_initializes(self, estimator):
        b = _unit(np.array([1.0, 0.0, 0.0]))
        estimator.bearing_and_dot(b, ts=0.0, dist=1.0)
        assert estimator.initialized


class TestSphericalConstraint:
    def test_output_on_unit_sphere(self, estimator):
        """||b_out|| == 1 for every update step."""
        for i, b in enumerate(_circular_sequence()):
            b_out, _ = estimator.bearing_and_dot(b, ts=i * DT, dist=1.5)
            assert np.linalg.norm(b_out) == pytest.approx(1.0, abs=1e-4), (
                f"step {i}: ||b_out|| = {np.linalg.norm(b_out):.6f}"
            )


class TestTangentConstraint:
    def test_bdot_perpendicular_to_bearing(self, estimator):
        """b · bdot ≈ 0 (tangent space constraint)."""
        for i, b in enumerate(_circular_sequence()):
            b_out, bdot_out = estimator.bearing_and_dot(b, ts=i * DT, dist=1.5)
            if i > 5:   # allow initialization transient
                dot = float(np.dot(b_out, bdot_out))
                assert abs(dot) < 0.05, (
                    f"step {i}: b·bdot = {dot:.4f} (should be ≈ 0)"
                )


class TestConvergence:
    def test_converges_to_static_target(self, estimator):
        """After 80 steps at 30 Hz the estimate error must be < 5°."""
        b_true = _unit(np.array([0.6, 0.8, 0.0]))
        seq = _static_sequence(b_true, n=80)
        b_out = b_true   # fallback
        for i, b in enumerate(seq):
            b_out, _ = estimator.bearing_and_dot(b, ts=i * DT, dist=1.0)
        error_deg = float(np.degrees(np.arccos(
            np.clip(np.dot(b_out, b_true), -1.0, 1.0)
        )))
        assert error_deg < 5.0, (
            f"Did not converge: final error = {error_deg:.2f}°"
        )


class TestCounters:
    def test_update_count_increments(self, estimator):
        b = _unit(np.array([1.0, 0.0, 0.0]))
        for i in range(10):
            estimator.bearing_and_dot(b, ts=i * DT, dist=1.0)
        assert estimator.update_count >= 1

    def test_confidence_in_range(self, estimator):
        b = _unit(np.array([1.0, 0.0, 0.0]))
        for i in range(5):
            estimator.bearing_and_dot(b, ts=i * DT, dist=1.0)
        c = estimator.confidence(5 * DT)
        assert 0.0 <= c <= 1.0, f"confidence = {c} out of [0, 1]"


# ── Factory tests ─────────────────────────────────────────────────────────────

class TestMakeEstimator:
    def test_make_kalman(self):
        assert isinstance(make_estimator("kalman"), BearingKalmanFilter)

    def test_make_hinf(self):
        assert isinstance(make_estimator("hinf"), BearingHinfFilter)

    def test_invalid_raises(self):
        with pytest.raises((ValueError, KeyError)):
            make_estimator("unknown_estimator")


