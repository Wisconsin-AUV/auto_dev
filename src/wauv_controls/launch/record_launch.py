"""
Records everything needed to score a hold / alignment run into one rosbag, so runs
can be compared after the fact.

This should be ran alongside the sub's normal launch, Ctrl + C to stop and close the bag:
    ros2 launch wauv_controls record_launch.py label:=trim_v1

The ArduSub dataflash log (.bin) is on the flight controller, so pull it separately after the run.
"""
import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

TOPICS = [
    '/wauv/.*',                         # detections, vehicle_ready, hold monitor
    '/mavros/state',                    # mode, armed
    '/mavros/imu/data',                 # attitude + rates
    '/mavros/imu/data_raw',
    '/mavros/imu/mag',                  # compass, for coilgun interference ;)
    '/mavros/global_position/rel_alt',  # depth
    '/mavros/local_position/odom',        # pose/velocity once the DVL is in
    '/joystick',
]


def record(context):
    bag_dir = os.path.expanduser(LaunchConfiguration('bag_dir').perform(context))
    label = LaunchConfiguration('label').perform(context)
    storage = LaunchConfiguration('storage').perform(context)

    name = datetime.now().strftime('%Y%m%d_%H%M%S')
    if label:
        name += '_' + label
    os.makedirs(bag_dir, exist_ok=True)

    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 '-e', '^(' + '|'.join(TOPICS) + ')$',
                 '-s', storage,
                 '-o', os.path.join(bag_dir, name)],
            output='screen'
        ),
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('bag_dir', default_value='~/wauv_bags'),
        DeclareLaunchArgument('label', default_value='',
                              description='tag for the run, e.g. trim_v1 or gains_b'),
        DeclareLaunchArgument('storage', default_value='sqlite3',
                              description='mcap if ros-humble-rosbag2-storage-mcap is installed'),
        OpaqueFunction(function=record),
    ])
