import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
 
    return LaunchDescription([
        # mavros launch
        # mavros launch
        ExecuteProcess(
            cmd=['bash', '-c',
                'source /opt/ros/humble/setup.bash && '
                'ros2 run mavros mavros_node --ros-args '
                '-p fcu_url:="udp://127.0.0.1:14551@" '
                '-p system_id:=255 '
                '-p tgt_system:=1 '
                '-p tgt_component:=1 '
                '-p use_sim_time:=false'],
            output='screen'
        ),

        # start ROS nodes
        Node(
            package='wauv_controls',
            executable='vehicle_manager',
            name='vehicle_manager',
            output='screen'
        ),

        Node(
            package='wauv_controls',
            executable='xbox_controller',
            name='xbox_controller',
            output='screen'
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
 
    ])