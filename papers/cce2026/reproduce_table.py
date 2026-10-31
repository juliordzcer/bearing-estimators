#!/usr/bin/env python3
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""Reproduce Table II of the CCE 2026 paper from the recorded Gazebo flight.

Applies the KF and H-infinity filters offline to the raw ArUco bearings, with
the parameters of Table I, and prints RMSE_e and RMSE_gdot for the four pairs
that target CF7. Metrics follow the paper: the marker offset is estimated from
the first sample and removed from the ground truth, and the ground-truth rate
is a central difference projected on the tangent plane.
"""
import argparse
import os
import sys

import numpy as np
import scipy.io

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'uav_estimators'))
from uav_estimators.bearing_estimators import make_estimator  # noqa: E402

PARAMS = {
    'kalman': {'q_vel': 0.02120, 'r_base': 0.001122, 'r_scale': 3.7768e-05},
    'hinf': {'gamma': 4.6024, 'q_vel': 0.1986, 'r_base': 0.001122,
             'r_scale': 3.7768e-05, 'l_pos': 1.6759, 'l_vel': 0.2025,
             'l_acc': 0.5953},
}
# Tuned offline on simulated-time flights, with q_vel limited to at least 0.02
# because lower values keep the filter too slow to close the control loop.
TUNED = {
    'kalman': {'q_vel': 0.020001, 'r_base': 0.015301, 'r_scale': 0.0063895},
    'hinf': {'gamma': 2.3147, 'q_vel': 0.060555, 'r_base': 0.086687,
             'r_scale': 0.00033115, 'l_pos': 2.5844, 'l_vel': 0.99927,
             'l_acc': 0.0066723},
}
PARAM_SETS = {'paper': PARAMS, 'tuned': TUNED}
# Published values: (RMSE_e KF, RMSE_gdot KF, RMSE_e Hinf, RMSE_gdot Hinf).
PAPER = {2: (0.0382, 0.0419, 0.0381, 0.0370),
         3: (0.0380, 0.0472, 0.0378, 0.0407),
         5: (0.0365, 0.0416, 0.0364, 0.0346),
         6: (0.0363, 0.0431, 0.0361, 0.0362)}


def run_filter(kind, t, g_meas, dist, params=None):
    est = make_estimator(kind, **(params if params else PARAMS[kind]))
    g, gd = np.zeros_like(g_meas), np.zeros_like(g_meas)
    for k in range(len(t)):
        z = g_meas[:, k] / np.linalg.norm(g_meas[:, k])
        g[:, k], gd[:, k] = est.bearing_and_dot(z, float(t[k]), dist=float(dist[k]))
    return g, gd


def gt_rate(g_gt, t):
    gd = np.zeros_like(g_gt)
    for k in range(1, len(t) - 1):
        dt = t[k + 1] - t[k - 1]
        if dt >= 1e-9:
            b = g_gt[:, k]
            gd[:, k] = (np.eye(3) - np.outer(b, b)) @ ((g_gt[:, k + 1] - g_gt[:, k - 1]) / dt)
    gd[:, 0], gd[:, -1] = gd[:, 1], gd[:, -2]
    return gd


def rmse(x):
    return float(np.sqrt(np.mean(x ** 2)))


def metrics(t, g, gd, g_meas, g_gt, t_min):
    off = g_gt + (g_meas[:, 0] - g_gt[:, 0])[:, None]
    off /= np.linalg.norm(off, axis=0)
    e = np.arccos(np.clip(np.sum(g * off, axis=0), -1.0, 1.0))
    ed = np.linalg.norm(gd - gt_rate(g_gt, t), axis=0)
    i = t >= t_min
    return rmse(e[i]), rmse(ed[i]), int(i.sum())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mat', nargs='?',
                   default=os.path.join(HERE, 'data', 'bearing_raw_20260422_135803.mat'))
    p.add_argument('--t-min', type=float, default=0.0,
                   help='discard samples before this time [s]')
    a = p.parse_args()
    m = scipy.io.loadmat(a.mat, squeeze_me=True, struct_as_record=False)
    print(f'{"pair":10s}{"N":>6s} {"KF e":>8s}{"KF gdot":>9s}{"Hinf e":>9s}{"Hinf gdot":>10s}{"d_gdot%":>9s}')
    for i, ref in PAPER.items():
        d = m[f'pair_crazyflie{i}_to_crazyflie7']
        t = np.asarray(d.t, float)
        row = []
        for kind in ('kalman', 'hinf'):
            g, gd = run_filter(kind, t, d.g_meas, d.dists)
            er, ed, n = metrics(t, g, gd, d.g_meas, d.g_gt, a.t_min)
            row += [er, ed]
        imp = 100 * (row[1] - row[3]) / row[1]
        print(f'CF{i}->CF7  {n:6d} {row[0]:8.4f}{row[1]:9.4f}{row[2]:9.4f}{row[3]:10.4f}{imp:9.1f}')
        print(f'  paper          {ref[0]:8.4f}{ref[1]:9.4f}{ref[2]:9.4f}{ref[3]:10.4f}{100 * (ref[1] - ref[3]) / ref[1]:9.1f}')


if __name__ == '__main__':
    main()
