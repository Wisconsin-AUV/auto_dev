import rclpy
from rclpy.node import Node

from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import Vector3

class GateScript(Node) :
    def __init__(self) :
        super().init("Gate Script")
        self.cmd_pub = self.create_publisher(
            PositionTarget, 
            "mavros/setpoint_raw/local",
            10
        )

        self.cmd_sub = self.create_subscription(
            Vector3, 
            "vision/gate_raw",
            self.gate_cb,
            10
        )

        self.x = 0
        self.y = 0
        self.z = 0

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Gate Script Started")



    def gate_cb(self, msg) :
        self.get_logger().info("Gate Coordinates Received")
        self.x = msg._x
        self.y = msg._y
        self.z = msg._z

    def control_loop(self) :
        cmd = PositionTarget()
        cmd.position._x = self.x
        cmd.position._y = self.y
        cmd.position._z = self.z
        self.cmd_pub.publish(cmd)


    def main(args=None) :
        rclpy.init(args=args)
        node = GateScript()
        rclpy.spin(node)
        node.destroyNode()
        rclpy.shutdown()

if __name__ == "__main__" :
    main()