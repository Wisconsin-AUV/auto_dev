import rclpy
from rclpy.node import Node

from mavros_msgs.msg import PositionTarget

class LookAround(Node) :
    def __init__(self) :
        super().init("Look Around")
        self.cmd_pub = self.create_publisher(
            PositionTarget,
            "mavros/setpoint_raw/local",
            10
        )
        self.yaw = 1

    def command_loop(self) :
        cmd = PositionTarget()
        cmd.velocity.x = 0
        cmd.velocity.y = 0
        cmd.velocity.z = 0
        cmd.yaw_rate = self.yaw

        self.cmd_pub.publish(cmd)