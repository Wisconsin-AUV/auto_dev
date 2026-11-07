import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
 
    return LaunchDescription([
        ExecuteProcess(
            cmd=['bash', '-c',
                'source /opt/ros/humble/setup.bash && '
                'ros2 run mavros mavros_node --ros-args '
                '-p fcu_url:=/dev/ttyACM0:57600 '
                '-p system_id:=255 '
                '-p tgt_system:=1 '
                '-p tgt_component:=1 '
                '-p use_sim_time:=false'],
            output='screen'
        ),

        # start ROS nodes
        Node(
            package='wauv_perception',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen'
        ),

 
    ])