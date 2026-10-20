import glob
import math
import os
import re
import socket
import time

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import pyzed.sl as sl
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node  # noqa: E402

from wauv_perception import perception_core as core  # noqa: E402

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_units = sl.UNIT.METER
        # x right, y down, z forward: the optical frame perception_core uses

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Camera Open Error: {err}')


        info = self.zed.get_camera_information()
        conf = getattr(info, 'camera_configuration', info)
        cam = conf.calibration_parameters.left_cam
        self.k = (cam.fx, cam.fy, cam.cx, cam.cy)

        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.timer = self.create_timer(0.05, self.timer_callback)

        package_share = get_package_share_directory('wauv_perception')
        # gate model trained on the Krill gate labels; see models/krill_gate_v1.md
        file_path = os.path.join(package_share, 'models', 'krill_gate_v1.pt')
        self.model = YOLO(file_path)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    
    def timer_callback(self):
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            return
        self.zed.retrieve_image(self.image, sl.VIEW.LEFT, sl.MEM.CPU)
        self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH, sl.MEM.CPU)
        frame = cv2.cvtColor(self.image.get_data(), cv2.COLOR_BGRA2BGR)
        depth = self.depth.get_data()
        fx, fy, cx, cy = self.k
        # results is a list because you can pass multiple images at once
        results = self.model(frame)
        for result in results:
            for det in core.yolo_result_to_detections(result):
                # nearest surface in the box
                z = core.estimate_depth(depth, det)
                x1, y1, x2, y2 = int(det.x1), int(det.y1), int(det.x2), int(det.y2)
                if z is None:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), thickness=2)
                    continue
                u, v = det.center
                x, y, z = core.pixel_to_optical(u, v, z, fx, fy, cx, cy)
                self.get_logger().info(
                    f'{det.class_id} {det.score:.2f}: x={x:.2f} y={y:.2f} z={z:.2f}')
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), thickness=2)
                cv2.putText(frame, f'{x:.2f} {y:.2f} {z:.2f}m', (x1, max(24, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)

        stream_frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", stream_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if len(buf) >= 65000:
            print(f"frame too big: {len(buf)} bytes, dropped")
        else:
            self.sock.sendto(buf.tobytes(), ("192.168.137.1", 5000))

def main(args=None):
    rclpy.init(args=args)

    yolo_detector = YoloDetector()

    rclpy.spin(yolo_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    yolo_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

