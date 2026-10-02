from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # sim plumbing check with no weights:  ros2 launch wauv_perception perception_launch.py backend:=hough
    # competition model:                    ros2 launch wauv_perception perception_launch.py yolo_weights:=/path/best.pt
    # real sub (ZED X Mini, depth already registered to the left image); check the topic
    # names with `ros2 topic list` first, they depend on the ZED wrapper's camera_name:
    #   ros2 launch wauv_perception perception_launch.py yolo_weights:=/path/best.pt yolo_conf:=0.5     #     color_in_depth_x:=0.0     #     image_topic:=/zed/zed_node/rgb/image_rect_color image_info_topic:=/zed/zed_node/rgb/camera_info     #     depth_topic:=/zed/zed_node/depth/depth_registered depth_info_topic:=/zed/zed_node/depth/camera_info
    backend = DeclareLaunchArgument('backend', default_value='yolo')
    weights = DeclareLaunchArgument('yolo_weights', default_value='')
    conf = DeclareLaunchArgument('yolo_conf', default_value='0.25')
    # Stonefish default; the real ZED needs 0.0
    color_x = DeclareLaunchArgument('color_in_depth_x', default_value='-0.0725')
    # Stonefish defaults
    topics = {
        'image_topic': '/bluerov2/left/image_color',
        'image_info_topic': '/bluerov2/left/camera_info',
        'depth_topic': '/bluerov2/depth_image/image_depth',
        'depth_info_topic': '/bluerov2/depth_image/camera_info',
    }
    topic_args = [DeclareLaunchArgument(k, default_value=v) for k, v in topics.items()]

    return LaunchDescription([
        backend,
        weights,
        conf,
        color_x,
        *topic_args,
        Node(
            package='wauv_perception',
            executable='perception',
            name='perception',
            output='screen',
            parameters=[{
                'backend': LaunchConfiguration('backend'),
                'yolo_weights': LaunchConfiguration('yolo_weights'),
                'yolo_conf': LaunchConfiguration('yolo_conf'),
                'color_in_depth_x': LaunchConfiguration('color_in_depth_x'),
                **{k: LaunchConfiguration(k) for k in topics},
            }],
        ),
    ])
