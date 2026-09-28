from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # sim plumbing check with no weights:  ros2 launch wauv_perception perception_launch.py backend:=hough
    # competition model:                    ros2 launch wauv_perception perception_launch.py yolo_weights:=/path/best.pt
    backend = DeclareLaunchArgument('backend', default_value='yolo')
    weights = DeclareLaunchArgument('yolo_weights', default_value='')

    return LaunchDescription([
        backend,
        weights,
        Node(
            package='wauv_perception',
            executable='perception',
            name='perception',
            output='screen',
            parameters=[{
                'backend': LaunchConfiguration('backend'),
                'yolo_weights': LaunchConfiguration('yolo_weights'),
            }],
        ),
    ])
