import glob
import math
import os
import re
import socket
import time
from unittest import result

os.environ['OPENCV_IO_ENABLE_OPENEXR'] = '1'  # before cv2 is imported, for exr_dir replay

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
import pyzed.sl as sl
from utralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node  # noqa: E402
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose  # noqa: E402

from wauv_perception import perception_core as core  # noqa: E402

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.depth_mode = getattr(sl.DEPTH_MODE, depth_mode)
        init_params.coordinate_units = sl.UNIT.METER
        # x right, y down, z forward: the optical frame perception_core uses

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Camera Open Error: {err}')

        self.image = sl.Mat()
        self.timer = self.create_timer(0.05, self.timer_callback)

        package_share = get_package_share_directory('wauv_perception')
        file_path = os.path.join(package_share, 'models', 'role_signs_v3.pt')
    
        self.model = YOLO(file_path)


    
    def timer_callback(self):
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

        cv2.imshow('YOLO Detection', frame)
        cv2.waitKey(1)

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

