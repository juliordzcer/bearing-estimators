#!/usr/bin/env python3
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
"""Control panel of the CCE 2026 simulation.

Buttons send take off, automatic, land and emergency stop. The plots show the
formation in 3D and the geodesic error of the running estimator (Kalman or
H-infinity) for each observed pair.

ROS runs in a separate process. Poses arrive at 250 Hz per drone, and handling
them in the same process as the plots makes the display fall behind the flight.
"""

from enum import IntEnum
import multiprocessing as mp
import queue
import signal
import sys
import tkinter as tk
from tkinter import ttk

from geometry_msgs.msg import Pose
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from mpl_toolkits.mplot3d import Axes3D  # noqa: E402, F401
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from std_msgs.msg import Int32  # noqa: E402
from uav_msgs.msg import BearingMeasurementsArray  # noqa: E402

ROBOT_NAMES = [f'crazyflie{i}' for i in range(1, 8)]
CONNECTIONS = [(1, 0), (1, 6), (2, 3), (2, 6), (5, 0), (5, 6), (4, 3), (4, 6)]
COLORS = ['#e74c3c', '#e67e22', '#2ecc71', '#3498db',
          '#9b59b6', '#1abc9c', '#f39c12']
ESTIMATOR_NAMES = {'kalman': 'Kalman filter', 'hinf': 'H-infinity filter'}
ERROR_WINDOW_SEC = 60.0
TRAIL_SEC = 25.0
SNAPSHOT_SEC = 0.1
DRAW_MS = 250


class DroneState(IntEnum):
    IDLE = 0
    AUTOMATIC = 1
    TAKING_OFF = 2
    LANDING = 3
    EMERGENCY_STOP = 4


STATE_LABELS = {
    DroneState.IDLE: ('IDLE', '#555555'),
    DroneState.AUTOMATIC: ('AUTOMATIC', '#1e8449'),
    DroneState.TAKING_OFF: ('TAKING OFF', '#2471a3'),
    DroneState.LANDING: ('LANDING', '#b9770e'),
    DroneState.EMERGENCY_STOP: ('EMERGENCY STOP', '#c0392b'),
}


def quat_to_rot(q):
    """Rotation matrix from body to world, from a quaternion (x, y, z, w)."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class RosWorker(Node):
    """Reads the ROS topics and exchanges small snapshots with the window."""

    def __init__(self, to_gui, to_ros):
        super().__init__(
            'cce2026_gui',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self._to_gui = to_gui
        self._to_ros = to_ros
        self._state = DroneState.IDLE
        self._poses = {}
        self._quats = {}
        self._offsets = {}
        self._errors = []
        self._t0 = None

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(Int32, '/state', state_qos)
        self.create_subscription(Int32, '/state', self._on_state, state_qos)
        for name in ROBOT_NAMES:
            self.create_subscription(
                Pose, f'/{name}/pose',
                lambda msg, n=name: self._on_pose(msg, n),
                qos_profile_sensor_data)
        self.create_subscription(
            BearingMeasurementsArray, '/vision_bearings',
            self._on_bearings, qos_profile_sensor_data)
        self.create_timer(SNAPSHOT_SEC, self._exchange)

    def _on_state(self, msg):
        try:
            self._set_state(DroneState(msg.data))
        except ValueError:
            pass

    def _set_state(self, state):
        if state == DroneState.AUTOMATIC and self._state != state:
            self._t0 = self.get_clock().now().nanoseconds * 1e-9
            self._offsets.clear()
        self._state = state

    def _on_pose(self, msg, name):
        self._poses[name] = (msg.position.x, msg.position.y, msg.position.z)
        self._quats[name] = (msg.orientation.x, msg.orientation.y,
                             msg.orientation.z, msg.orientation.w)

    def _on_bearings(self, msg):
        if self._t0 is None or self._state != DroneState.AUTOMATIC:
            return
        t = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        for m in msg.measurements:
            if m.observer_id >= len(ROBOT_NAMES) or m.target_id >= len(ROBOT_NAMES):
                continue
            obs, tgt = ROBOT_NAMES[m.observer_id], ROBOT_NAMES[m.target_id]
            if obs not in self._poses or tgt not in self._poses:
                continue
            diff = np.array(self._poses[tgt]) - np.array(self._poses[obs])
            dist = np.linalg.norm(diff)
            g_hat = np.array([m.bearing_vector.x, m.bearing_vector.y, m.bearing_vector.z])
            if dist < 0.01 or np.linalg.norm(g_hat) < 1e-6:
                continue
            g_gt = quat_to_rot(self._quats[obs]).T @ (diff / dist)
            key = (m.observer_id, m.target_id)
            # The marker is not at the center of mass: remove that constant
            # offset with the first sample, as in the paper.
            offset = self._offsets.setdefault(key, g_hat - g_gt)
            g_ref = g_gt + offset
            g_ref /= np.linalg.norm(g_ref)
            err = float(np.arccos(np.clip(g_hat @ g_ref, -1.0, 1.0)))
            self._errors.append((key, t, err))

    def _exchange(self):
        try:
            while True:
                state = DroneState(self._to_ros.get_nowait())
                msg = Int32()
                msg.data = int(state)
                self._state_pub.publish(msg)
                self._set_state(state)
        except queue.Empty:
            pass
        snapshot = (int(self._state), dict(self._poses), self._errors)
        self._errors = []
        try:
            self._to_gui.put_nowait(snapshot)
        except queue.Full:
            pass


def ros_main(to_gui, to_ros):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    rclpy.init()
    node = RosWorker(to_gui, to_ros)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


class ControlPanel:
    """Tk window: control buttons, formation plot and estimation error plot."""

    def __init__(self, estimator, to_gui, to_ros):
        self._to_gui = to_gui
        self._to_ros = to_ros
        self._state = DroneState.IDLE
        self._poses = {}
        self._trails = {n: [] for n in ROBOT_NAMES}
        self._errors = {}

        self.root = tk.Tk()
        self.root.title('CCE 2026 | Bearing estimators')
        self.root.geometry('1200x800')
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TButton', font=('Arial', 11), padding=8)
        style.configure('Stop.TButton', font=('Arial', 11, 'bold'), foreground='#c0392b')

        panel = ttk.Frame(self.root, padding=12, relief='groove')
        panel.pack(side=tk.LEFT, fill=tk.Y, padx=(8, 4), pady=8)
        ttk.Label(panel, text='CCE 2026', font=('Arial', 16, 'bold')).pack()
        ttk.Label(panel, text=ESTIMATOR_NAMES.get(estimator, estimator),
                  font=('Arial', 11)).pack(pady=(2, 10))
        self._status = tk.Label(panel, text='IDLE', font=('Arial', 12, 'bold'),
                                fg=STATE_LABELS[DroneState.IDLE][1])
        self._status.pack(pady=8)
        ttk.Separator(panel).pack(fill='x', pady=6)
        for text, state, btn_style in (
                ('Take Off', DroneState.TAKING_OFF, 'TButton'),
                ('Automatic Mode', DroneState.AUTOMATIC, 'TButton'),
                ('Land', DroneState.LANDING, 'TButton'),
                ('Emergency Stop', DroneState.EMERGENCY_STOP, 'Stop.TButton')):
            ttk.Button(panel, text=text, width=20, style=btn_style,
                       command=lambda s=state: self.send(s)).pack(pady=6, padx=8)
        ttk.Separator(panel).pack(fill='x', pady=10)
        self._rms = ttk.Label(panel, text='RMS error: -', font=('Arial', 11))
        self._rms.pack(pady=4)
        for i in range(len(ROBOT_NAMES)):
            row = ttk.Frame(panel)
            row.pack(anchor='w', padx=28)
            tk.Label(row, text='●', fg=COLORS[i], font=('Arial', 11)).pack(side=tk.LEFT)
            ttk.Label(row, text=f'CF{i + 1}', font=('Arial', 10)).pack(side=tk.LEFT)

        plots = ttk.Frame(self.root, padding=4)
        plots.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(4, 8), pady=8)
        self._fig = Figure(figsize=(9, 8), dpi=100, constrained_layout=True)
        grid = self._fig.add_gridspec(5, 1)
        self._ax_form = self._fig.add_subplot(grid[:3, 0], projection='3d')
        self._ax_err = self._fig.add_subplot(grid[3:, 0])
        self._canvas = FigureCanvasTkAgg(self._fig, master=plots)
        self._canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.root.protocol('WM_DELETE_WINDOW', self.root.quit)
        self.root.after(DRAW_MS, self._tick)

    def send(self, state):
        self._to_ros.put(int(state))

    def _read_snapshots(self):
        try:
            while True:
                state, poses, errors = self._to_gui.get_nowait()
                self._state = DroneState(state)
                self._poses = poses
                for name, pos in poses.items():
                    trail = self._trails[name]
                    trail.append(pos)
                    del trail[:-int(TRAIL_SEC / SNAPSHOT_SEC)]
                for key, t, err in errors:
                    hist = self._errors.setdefault(key, ([], []))
                    if hist[0] and t < hist[0][-1]:
                        hist[0].clear()
                        hist[1].clear()
                    hist[0].append(t)
                    hist[1].append(err)
        except queue.Empty:
            pass

    def _draw_formation(self):
        ax = self._ax_form
        elev, azim = getattr(ax, 'elev', 25), getattr(ax, 'azim', -60)
        ax.cla()
        ax.view_init(elev=elev, azim=azim)
        ax.set_title('Formation', fontsize=11)
        ax.set_xlabel('x [m]', fontsize=8)
        ax.set_ylabel('y [m]', fontsize=8)
        ax.set_zlabel('z [m]', fontsize=8)
        ax.tick_params(labelsize=7)
        for i, name in enumerate(ROBOT_NAMES):
            trail = self._trails[name]
            if len(trail) > 1:
                xs, ys, zs = zip(*trail)
                ax.plot(xs, ys, zs, color=COLORS[i], linewidth=0.8, alpha=0.35)
        for obs, tgt in CONNECTIONS:
            a = self._poses.get(ROBOT_NAMES[obs])
            b = self._poses.get(ROBOT_NAMES[tgt])
            if a and b:
                ax.plot(*zip(a, b), color='#5dade2', linewidth=0.9, alpha=0.7)
        for i, name in enumerate(ROBOT_NAMES):
            pos = self._poses.get(name)
            if pos:
                ax.scatter(*pos, color=COLORS[i], s=40, edgecolors='gray', linewidths=0.4)
        if self._poses:
            pts = np.array(list(self._poses.values()))
            center = pts.mean(axis=0)
            half = max(0.8, float(np.abs(pts - center).max()) + 0.2)
            ax.set_xlim(center[0] - half, center[0] + half)
            ax.set_ylim(center[1] - half, center[1] + half)
            ax.set_zlim(max(0.0, center[2] - half), center[2] + half)

    def _draw_errors(self):
        ax = self._ax_err
        ax.cla()
        ax.set_title('Estimation error', fontsize=11)
        ax.set_xlabel('time in automatic mode [s]', fontsize=9)
        ax.set_ylabel('geodesic error [rad]', fontsize=9)
        ax.tick_params(labelsize=8)
        ax.grid(True, linewidth=0.5, linestyle='--', alpha=0.5)
        recent = []
        for (obs, tgt), (ts, es) in sorted(self._errors.items()):
            if len(ts) < 2:
                continue
            start = np.searchsorted(ts, ts[-1] - ERROR_WINDOW_SEC)
            style = '-' if tgt == 6 else '--'
            ax.plot(ts[start:], es[start:], style, color=COLORS[obs], linewidth=1.2,
                    label=f'CF{obs + 1}→CF{tgt + 1}')
            recent += es[-100:]
        if recent:
            ax.legend(loc='upper right', fontsize=8, ncol=4, framealpha=0.7)
            rms = float(np.sqrt(np.mean(np.square(recent))))
            self._rms.config(text=f'RMS error: {rms:.3f} rad')

    def _tick(self):
        self._read_snapshots()
        label, color = STATE_LABELS[self._state]
        self._status.config(text=label, fg=color)
        self._draw_formation()
        self._draw_errors()
        self._canvas.draw_idle()
        self.root.after(DRAW_MS, self._tick)


def main(args=None):
    argv = rclpy.utilities.remove_ros_args(sys.argv if args is None else args)
    estimator = argv[1] if len(argv) > 1 else 'hinf'
    # The worker is started before Tk exists, so it inherits no window state.
    context = mp.get_context('fork')
    to_gui, to_ros = context.Queue(maxsize=50), context.Queue()
    worker = context.Process(target=ros_main, args=(to_gui, to_ros), daemon=True)
    worker.start()
    panel = ControlPanel(estimator, to_gui, to_ros)
    signal.signal(signal.SIGINT, lambda *_: panel.root.quit())

    # Tk only handles signals while Python code runs.
    def keep_alive():
        panel.root.after(200, keep_alive)
    keep_alive()
    try:
        panel.root.mainloop()
    finally:
        worker.terminate()
        worker.join(timeout=2)


if __name__ == '__main__':
    main()
