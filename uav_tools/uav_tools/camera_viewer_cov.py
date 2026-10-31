#!/usr/bin/env python3
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
# camera_viewer_cov.py - camera_viewer con círculo de covarianza del estimador

import threading
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from uav_msgs.msg import BearingMeasurementsArray

# Máximo de frames a procesar por segundo (limita carga CPU del viewer)
DISPLAY_FPS = 15


class CameraViewerCov(Node):
    def __init__(self):
        super().__init__('camera_viewer_cov')

        self.bridge = CvBridge()
        self.latest_image = None
        self.lock = threading.Lock()
        self.processing = False
        self.fov_horizontal_deg = 66.0
        # focal length para Crazyflie 320×240 FOV_h=66°:  fx = (160/tan(33°))
        self.fx = 246.4
        self.frame_count = 0
        self.start_time = time.time()
        self._stop = threading.Event()
        self._last_proc_t = 0.0
        self._min_interval = 1.0 / DISPLAY_FPS

        # target_id (0-based, de BearingMeasurement) → std-dev angular [rad]
        self._uncert_lock = threading.Lock()
        self._uncert: dict = {}

        # ArUco/AprilTag ID (1-based) → target_id (0-based)
        self._id_map = {i: i - 1 for i in range(1, 10)}

        self._setup_detectors()

        self.image_sub = self.create_subscription(
            Image,
            '/world/crazyflie/model/crazyflie2/link/body/sensor/rgb_camera_sensor/image',
            self._image_callback,
            qos_profile_sensor_data,
        )

        best_effort = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.bearing_sub = self.create_subscription(
            BearingMeasurementsArray,
            '/vision_bearings',
            self._bearing_callback,
            best_effort,
        )

        self.get_logger().info(f'CameraViewerCov iniciado — OpenCV {cv2.__version__}')
        self._start_display_thread()

    # ------------------------------------------------------------------
    def _bearing_callback(self, msg: BearingMeasurementsArray):
        with self._uncert_lock:
            for m in msg.measurements:
                self._uncert[m.target_id] = m.uncertainty

    # ------------------------------------------------------------------
    def _setup_detectors(self):
        try:
            params = cv2.aruco.DetectorParameters()
            # CORNER_REFINE_NONE: suficiente para display, evita el coste de subpixel
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
            # Solo ArUco 4x4 — los markers del sim son ArUco, no AprilTag
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
            self.aruco_det = cv2.aruco.ArucoDetector(aruco_dict, params)
            self.tag_id_to_name = {i: f'{i}' for i in range(1, 10)}
            self.get_logger().info('✔ ArUco 4x4 listo')
        except Exception as e:
            self.get_logger().error(f'Error configurando detectores: {e}')
            self.aruco_det = None

    # ------------------------------------------------------------------
    def _image_callback(self, msg):
        now = time.monotonic()
        # Descarta frames que lleguen más rápido que DISPLAY_FPS
        if self.processing or (now - self._last_proc_t) < self._min_interval:
            return
        self.processing = True
        self._last_proc_t = now
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            processed = self._process_frame(cv_image)
            with self.lock:
                self.latest_image = processed
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f'image_callback: {e}')
        finally:
            self.processing = False

    # ------------------------------------------------------------------
    def _process_frame(self, image):
        if image is None or image.size == 0:
            return image

        out = image.copy()
        h, w = out.shape[:2]
        cx, cy = w // 2, h // 2
        gray = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)

        if self.aruco_det is not None:
            corners, ids, _ = self.aruco_det.detectMarkers(gray)
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(out, corners)
                # Un solo overlay para todos los círculos de covarianza
                with self._uncert_lock:
                    uncert_snap = dict(self._uncert)
                overlay = out.copy()
                drew_any = False
                for i, tag_id in enumerate(ids.flatten()):
                    self._draw_tag(out, overlay, corners[i], int(tag_id),
                                   cx, cy, w, uncert_snap)
                    drew_any = True
                if drew_any:
                    cv2.addWeighted(overlay, 0.12, out, 0.88, 0, out)

        elapsed = max(time.monotonic() - self.start_time, 1e-6)
        fps = self.frame_count / elapsed
        cv2.putText(out, f'FPS:{fps:.1f}', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.drawMarker(out, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        return out

    # ------------------------------------------------------------------
    def _draw_tag(self, image, overlay, corners, tag_id, cx, cy, img_w, uncert_snap):
        pts = corners[0]
        mx = int(np.mean(pts[:, 0]))
        my = int(np.mean(pts[:, 1]))

        color = (255, 120, 0)
        cv2.line(image, (cx, cy), (mx, my), color, 1, cv2.LINE_AA)
        cv2.circle(image, (mx, my), 5, color, -1)
        label = f'CF{self.tag_id_to_name.get(tag_id, tag_id)}'
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.putText(image, label, (mx - tw // 2, my - th - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)

        # Dibuja el círculo de covarianza sobre el overlay compartido
        tid = self._id_map.get(tag_id)
        if tid is not None:
            sigma = uncert_snap.get(tid)
            if sigma is not None and sigma > 1e-6:
                radius = max(4, int(self.fx * np.tan(2.0 * float(sigma))))
                cv2.circle(overlay, (mx, my), radius, (0, 220, 255), cv2.FILLED)
                cv2.circle(image,   (mx, my), radius, (0, 220, 255), 2, cv2.LINE_AA)

    # ------------------------------------------------------------------
    def _start_display_thread(self):
        def loop():
            cv2.namedWindow('Camera Viewer — Covarianza', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Camera Viewer — Covarianza', 800, 600)
            while not self._stop.is_set() and rclpy.ok():
                with self.lock:
                    frame = self.latest_image.copy() if self.latest_image is not None else None
                if frame is not None:
                    cv2.imshow('Camera Viewer — Covarianza', frame)
                key = cv2.waitKey(30) & 0xFF  # 30ms ≈ 33 FPS cap en display
                if key in (ord('q'), 27):
                    self._stop.set()
                    break
            cv2.destroyAllWindows()
            rclpy.shutdown()

        threading.Thread(target=loop, daemon=True, name='display').start()

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


# ----------------------------------------------------------------------
def main():
    rclpy.init()
    node = CameraViewerCov()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
