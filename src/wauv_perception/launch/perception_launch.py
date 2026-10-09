from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Frames come straight from the ZED SDK (pyzed), not from ROS image topics.
    # plumbing check with no weights:
    #   ros2 launch wauv_perception perception_launch.py backend:=hough
    # competition model:               ros2 launch wauv_perception perception_launch.py \
    #                                    yolo_weights:=/path/best.pt yolo_conf:=0.5
    # replay record.py EXR frames on a laptop (no camera):
    #   ros2 launch wauv_perception perception_launch.py exr_dir:=/path/to/data \
    #     stream_host:=127.0.0.1 yolo_weights:=/path/best.pt
    # Watch the annotated image on the laptop with WAUV-tools' streamer.py.
    args = {
        'backend': 'yolo',
        'yolo_weights': '',
        'yolo_conf': '0.25',
        'depth_mode': 'ULTRA',
        'max_depth': '8.0',
        'stream': 'true',
        'stream_host': '192.168.137.1',
        'stream_port': '5000',
        'exr_dir': '',
    }
    declared = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]

    return LaunchDescription([
        *declared,
        Node(
            package='wauv_perception',
            executable='perception',
            name='perception',
            output='screen',
            parameters=[{k: LaunchConfiguration(k) for k in args}],
        ),
    ])
