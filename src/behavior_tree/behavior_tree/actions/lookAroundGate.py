import rclpy
import py_trees
from rclpy.node import Node

from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import Vector3


class LookAroundGate(Node, py_trees.behaviour.Behaviour):
    def __init__(self) :
        Node.__init__(self, "Look Around")
        py_trees.behaviour.Behaviour.__init__(self, "Look Around")

        self.blackboard = py_trees.blackboard.Client(name="Look Around")
        self.blackboard.register_key("gate_found", access=py_trees.common.Access.READ)

        self.updated = False

        #Publisher for mavros
        self.cmd_pub = self.create_publisher(
            PositionTarget,
            "mavros/setpoint_raw/local",
            10
        )

        #Subscriber for vision
        self.cmd_sub = self.create_subscription(
            Vector3,
            "vision/gate_raw",
            self.vision_cb,
            10
        )

        self.get_logger().info("Looking Around")

    def initialize(self) :
        main()

    def update(self) :
        self.control_loop()
        if(self.blackboard.gateFound) :
            return py_trees.common.Status.SUCCESS
        else :
            return py_trees.common.Status.RUNNING

    def vision_cb(self, msg) :
        self.get_logger().info("Gate Coordinates Received")
        self.updated = True

    def control_loop(self) :
        cmd = PositionTarget()
        cmd.coordinate_frame = 8
        cmd.type_mask = 2040 #ignores eveerything but xyz and yawrate

        #Stop moving and spin, assumes robot is always at 0,0,0
        cmd.yaw_rate = 0.3 #range 0 to 1
        cmd.position.x = 0
        cmd.position.y = 0
        cmd.position.z = 0

        self.get_logger().info("Looking for Gate")
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LookAround()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()