import os
import socket

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pyzed.sl as sl
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from ament_index_python.packages import get_package_share_directory


class YoloDetectorZedNative(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.zed = sl.Camera()

        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        # IMAGE coordinate system = x right, y down, z forward (optical frame)

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Camera Open Error: {err}')

        # 2. Native ZED Custom YOLO ONNX Object Detection (No Positional Tracking)
        # which model in share/wauv_perception/models to run, e.g. -p model:=bins_v1.onnx
        model_name = self.declare_parameter('model', 'krill_gate_v1.onnx').value
        package_share = get_package_share_directory('wauv_perception')
        onnx_path = os.path.join(package_share, 'models', model_name)
        if not os.path.exists(onnx_path):
            raise FileNotFoundError(f'ONNX file not found at: {onnx_path}')

        self.get_logger().info(
            f'Loading {onnx_path} into ZED TensorRT...'
        )
        obj_param = sl.ObjectDetectionParameters()
        obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_YOLOLIKE_BOX_OBJECTS
        obj_param.custom_onnx_file = onnx_path
        obj_param.enable_tracking = False
        obj_param.enable_segmentation = False

        err = self.zed.enable_object_detection(obj_param)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'Object Detection Enable Error: {err}')
        self.get_logger().info('ZED YOLO TensorRT Engine Ready!')

        self.runtime_params = sl.RuntimeParameters()
        self.od_runtime_params = sl.CustomObjectDetectionRuntimeParameters()
        self.od_runtime_params.object_detection_properties.detection_confidence_threshold = 30

        self.image = sl.Mat()
        self.objects = sl.Objects()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return

        # Runs YOLO + 3D depth extraction on GPU
        status = self.zed.retrieve_custom_objects(self.objects, self.od_runtime_params)

        self.zed.retrieve_image(self.image, sl.VIEW.LEFT, sl.MEM.CPU)
        frame = cv2.cvtColor(self.image.get_data(), cv2.COLOR_BGRA2BGR)

        if status == sl.ERROR_CODE.SUCCESS:
            for obj in self.objects.object_list:
                bbox_2d = obj.bounding_box_2d
                x1, y1 = int(bbox_2d[0][0]), int(bbox_2d[0][1])
                x2, y2 = int(bbox_2d[2][0]), int(bbox_2d[2][1])

                # 3D position [x, y, z] in meters (x right, y down, z forward)
                pos = obj.position
                x, y, z = float(pos[0]), float(pos[1]), float(pos[2])

                class_id = obj.raw_label
                score = obj.confidence / 100.0

                if not np.isfinite(z) or z <= 0.0:
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), thickness=2)
                    continue

                self.get_logger().info(
                    f'{class_id} {score:.2f}: x={x:.2f} y={y:.2f} z={z:.2f}'
                )
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), thickness=2)
                cv2.putText(
                    frame, f'{x:.2f} {y:.2f} {z:.2f}m', (x1, max(24, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA
                )

        # Resize & stream via UDP
        stream_frame = cv2.resize(frame, (640, 360), interpolation=cv2.INTER_LINEAR)
        ok, buf = cv2.imencode(".jpg", stream_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            if len(buf) >= 65000:
                self.get_logger().warn(f"frame too big: {len(buf)} bytes, dropped")
            else:
                try:
                    self.sock.sendto(buf.tobytes(), ("192.168.137.1", 5000))
                except OSError:
                    pass


def main(args=None):
    rclpy.init(args=args)
    yolo_detector = YoloDetectorZedNative()
    try:
        rclpy.spin(yolo_detector)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_detector.zed.disable_object_detection()
        yolo_detector.zed.close()
        yolo_detector.sock.close()
        yolo_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()