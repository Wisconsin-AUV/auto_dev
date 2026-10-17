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

        self.image = sl.Mat()
        self.timer = self.create_timer(0.05, self.timer_callback)

        package_share = get_package_share_directory('wauv_perception')
        file_path = os.path.join(package_share, 'models', 'role_signs_v3', 'role_signs_v3.pt')
        self.model = YOLO(file_path)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    
    def timer_callback(self):
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            return
        self.zed.retrieve_image(self.image, sl.VIEW.LEFT, sl.MEM.CPU)
        frame = cv2.cvtColor(self.image.get_data(), cv2.COLOR_BGRA2BGR)
        # results is a list because you can pass multiple images at once
        results = self.model(frame)
        for result in results:
            boxes = result.boxes # Bounding boxes object

            for box in boxes:
                confidence = box.conf[0].item()
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), thickness=2)

        
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

