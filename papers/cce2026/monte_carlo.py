#!/usr/bin/env python3
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""Monte Carlo study of the Kalman and H-infinity filters in Gazebo.

Each flight uses different initial positions and its own wind realization. The
raw ArUco bearings of each flight are then filtered offline by both estimators,
so they are compared on the same data.

Usage, with the workspace sourced:
    python3 monte_carlo.py --runs 30 --jobs 4 --output results/monte_carlo
"""
import argparse
import csv
import glob
import os
import signal
import subprocess
import sys
import time

import numpy as np
import scipy.io

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import reproduce_table as table  # noqa: E402

PAIRS = (2, 3, 5, 6)
TIMEOUT_SEC = 300
MAX_RAW_ERROR_RAD = 0.5


def start_flight(index, base_dir, noise):
    """Start one isolated flight. Each job needs its own ROS domain and Gazebo partition."""
    out = os.path.join(base_dir, f'run_{index:03d}')
    os.makedirs(out, exist_ok=True)
    domain = 30 + index % 20
    env = dict(os.environ,
               ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
               GZ_PARTITION=f'mc{domain}', GZ_IP='127.0.0.1',
               UAV_IC_SEED=str(1000 + index), UAV_IC_NOISE_XY=str(noise))
    log = open(os.path.join(out, 'launch.log'), 'w')
    proc = subprocess.Popen(
        ['ros2', 'launch', 'uav_bringup', 'pid_7_comparison.launch.py',
         'headless:=true', 'run_gui:=false', 'auto_sequence:=true',
         f'logger_output_dir:={out}'],
        env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return {'proc': proc, 'dir': out, 'start': time.time(), 'index': index}


def stop_flight(flight):
    try:
        os.killpg(flight['proc'].pid, signal.SIGINT)
        flight['proc'].wait(timeout=20)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(flight['proc'].pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def analyze(mat_path):
    """Return one row per pair and parameter set, or None if the formation was lost.

    The distance in the log comes from the camera and freezes when a marker is
    lost, so the check uses the raw bearing against the Gazebo ground truth.
    """
    data = scipy.io.loadmat(mat_path, squeeze_me=True, struct_as_record=False)
    pair_keys = [k for k in data if k.startswith('pair_')]
    if len(pair_keys) < 8:
        return None
    for key in pair_keys:
        pair = data[key]
        raw = pair.g_meas / np.linalg.norm(pair.g_meas, axis=0)
        err = np.arccos(np.clip(np.sum(raw * pair.g_gt, axis=0), -1.0, 1.0))
        if np.mean(err > MAX_RAW_ERROR_RAD) > 0.01:
            return None
    rows = []
    for i in PAIRS:
        pair = data[f'pair_crazyflie{i}_to_crazyflie7']
        t = np.asarray(pair.t, float)
        for set_name, params in table.PARAM_SETS.items():
            for kind in ('kalman', 'hinf'):
                g, gd = table.run_filter(kind, t, pair.g_meas, pair.dists, params[kind])
                err, rate, _ = table.metrics(t, g, gd, pair.g_meas, pair.g_gt, 0.0)
                rows.append((i, set_name, kind, err, rate))
    return rows


def summarize(results, runs, lost, unfinished, output):
    with open(os.path.join(output, 'monte_carlo.csv'), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['run', 'observer', 'parameters', 'estimator', 'rmse_e_rad', 'rmse_gdot_rad_s'])
        writer.writerows(results)

    done = len({r[0] for r in results})
    print(f'\nflights: {runs}, valid: {done}, formation lost: {lost}, unfinished: {unfinished}')
    if not results:
        return
    print(f'{"parameters":12s}{"estimator":11s}{"RMSE_e [rad]":>20s}{"RMSE_gdot [rad/s]":>22s}')
    for set_name in table.PARAM_SETS:
        for kind in ('kalman', 'hinf'):
            sel = [r for r in results if r[2] == set_name and r[3] == kind]
            # Average the pairs of each flight first: pairs of a flight are not independent.
            per_run = {}
            for r in sel:
                per_run.setdefault(r[0], []).append((r[4], r[5]))
            vals = np.array([np.mean(v, axis=0) for v in per_run.values()])
            print(f'{set_name:12s}{kind:11s}'
                  f'{vals[:, 0].mean():12.4f} ± {vals[:, 0].std(ddof=1) if len(vals) > 1 else 0:.4f}'
                  f'{vals[:, 1].mean():14.4f} ± {vals[:, 1].std(ddof=1) if len(vals) > 1 else 0:.4f}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=int, default=30)
    parser.add_argument('--jobs', type=int, default=2,
                        help='flights at the same time; each one renders 7 cameras')
    parser.add_argument('--noise', type=float, default=0.01,
                        help='follower start position noise, +/- meters')
    parser.add_argument('--output', default='results/monte_carlo')
    args = parser.parse_args()
    os.makedirs(args.output, exist_ok=True)

    pending = list(range(args.runs))
    running = []
    results = []
    lost, unfinished = 0, 0
    while pending or running:
        while pending and len(running) < args.jobs:
            running.append(start_flight(pending.pop(0), args.output, args.noise))
        time.sleep(2)
        for flight in list(running):
            mats = glob.glob(os.path.join(flight['dir'], 'bearing_raw_*.mat'))
            timed_out = time.time() - flight['start'] > TIMEOUT_SEC
            if not mats and not timed_out:
                continue
            if mats:
                time.sleep(2)
            stop_flight(flight)
            running.remove(flight)
            if not mats:
                unfinished += 1
                print(f"run {flight['index']}: did not finish", flush=True)
                continue
            rows = analyze(sorted(mats)[0])
            if rows is None:
                lost += 1
                print(f"run {flight['index']}: formation lost", flush=True)
                continue
            results += [(flight['index'],) + row for row in rows]
            print(f"run {flight['index']}: ok", flush=True)

    summarize(results, args.runs, lost, unfinished, args.output)


if __name__ == '__main__':
    main()
