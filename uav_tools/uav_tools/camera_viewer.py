#!/usr/bin/env python3
# Copyright 2026 Julio Cesar Rodriguez <contact@juliordz.com>
# SPDX-License-Identifier: Apache-2.0
# camera_viewer.py - Compatible con OpenCV 4.7+ (4.10 incluido)

import threading
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer_apriltag')

        self.bridge = CvBridge()
        self.latest_image = None
        self.lock = threading.Lock()
        self.processing = False
        self.fov_horizontal_deg = 66.0
        self.frame_count = 0
        self.detection_count = 0
        self.start_time = time.time()
        self._stop = threading.Event()

        self._setup_detectors()

        self.image_sub = self.create_subscription(
            Image,
            '/world/crazyflie/model/crazyflie2/link/body/sensor/rgb_camera_sensor/image',
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f'Detector iniciado — OpenCV {cv2.__version__}')
        self._start_display_thread()

    # ------------------------------------------------------------------
    def _setup_detectors(self):
        try:
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE

            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
            self.aruco_det = cv2.aruco.ArucoDetector(aruco_dict, params)

            self.tag_id_to_name = {
                1: 'Crazyflie 1',
                2: 'Crazyflie 2',
                3: 'Crazyflie 3',
                4: 'Crazyflie 4',
                5: 'Crazyflie 5',
            }
            self.get_logger().info('✔ ArUco 4x4 listo')

        except Exception as e:
            self.get_logger().error(f'Error configurando detectores: {e}')
            self.aruco_det = None

    # ------------------------------------------------------------------
    def image_callback(self, msg):
        if self.processing:
            return
        self.processing = True
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

        # Cruz central
        cv2.drawMarker(out, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)

        if self.aruco_det is not None:
            corners, ids, _ = self.aruco_det.detectMarkers(gray)
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(out, corners, ids)
                self.detection_count += len(ids)
                for i, tag_id in enumerate(ids.flatten()):
                    self._draw_tag(out, corners[i], tag_id, cx, cy, w,
                                   color=(255, 120, 0), prefix='CF')

        self._draw_overlay(out, w, h)
        return out

    # ------------------------------------------------------------------
    def _draw_tag(self, image, corners, tag_id, cx, cy, img_w, color, prefix):
        pts = corners[0]
        mx = int(np.mean(pts[:, 0]))
        my = int(np.mean(pts[:, 1]))

        bearing = ((mx - cx) / img_w) * self.fov_horizontal_deg
        name = self.tag_id_to_name.get(tag_id, f'Tag_{tag_id}')

        cv2.line(image, (cx, cy), (mx, my), color, 2)
        cv2.circle(image, (mx, my), 6, color, -1)
        cv2.putText(image,
                    f'[{prefix}] {name}  {bearing:+.1f}deg',
                    (mx - 60, my - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    # ------------------------------------------------------------------
    def _draw_overlay(self, image, w, h):
        elapsed = max(time.time() - self.start_time, 1e-6)
        fps = self.frame_count / elapsed
        cv2.putText(
            image,
            f'OpenCV {cv2.__version__}  FPS:{fps:.1f}  Det:{self.detection_count}',
            (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (220, 220, 220),
            1,
            cv2.LINE_AA)
        cv2.putText(image, 'q / ESC para salir', (w - 180, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 200), 1)

    # ------------------------------------------------------------------
    def _start_display_thread(self):
        def loop():
            cv2.namedWindow('AprilTag / ArUco Viewer', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('AprilTag / ArUco Viewer', 800, 600)

            while not self._stop.is_set() and rclpy.ok():
                with self.lock:
                    frame = self.latest_image.copy() if self.latest_image is not None else None
                if frame is not None:
                    cv2.imshow('AprilTag / ArUco Viewer', frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    self._stop.set()
                    break
                time.sleep(0.008)

            cv2.destroyAllWindows()
            rclpy.shutdown()

        threading.Thread(target=loop, daemon=True, name='display').start()

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


# ----------------------------------------------------------------------
def main():
    rclpy.init()
    node = CameraViewer()
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
