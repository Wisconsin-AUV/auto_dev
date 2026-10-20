import rclpy
from rclpy.node import Node

from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import Vector3

class GateScript(Node) :
    def __init__(self) :
        #Publisher for mavros
        super().init("Gate Script")
        self.cmd_pub = self.create_publisher(
            PositionTarget, 
            "mavros/setpoint_raw/local",
            10
        )

        #Subscriber for vision
        self.cmd_sub = self.create_subscription(
            Vector3, 
            "vision/gate_raw",
            self.gate_cb,
            10
        )

        self.x = 0
        self.y = 0
        self.z = 0

        self.blind_ticks = 0
        self.max_blind_ticks = 40
        self.updated = False

        self.at_Gate = False
        self.arrived_deadband = 0.01

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Gate Script Started")



    def gate_cb(self, msg) :
        self.get_logger().info("Gate Coordinates Received")
        self.updated = True
        #Assuming meters
        self.x = msg._x
        self.y = msg._y
        self.z = msg._z

    def control_loop(self) :
        cmd = PositionTarget()
        if(not self.updated) :
            #No new vision input
            self.get_logger.info("No New Vision Input")
            self.blind_ticks = self.blind_ticks + 1
            if(self.blind_ticks > self.max_blind_ticks) :
                #Stop moving and spin, assumes robot is always at 0,0,0
                cmd.yaw_rate = 1
                cmd.position._x = 0
                cmd.position._y = 0
                cmd.position._z = 0
                self.get_logger.info("Looking for Gate")
        else :
            #Has seen new vision input
            self.blind_ticks = 0
            #If within deadband
            if(self.x < self.arrived_deadband and self.y < self.arrived_deadband and self.z < self.arrived_deadband and not self.at_Gate) :
                self.get_logger.info("At Gate")
                #Has not before set a post gate position
                cmd.position._x = 1 + self.x
            else :
                #normal operation
                cmd.position._x = self.x
            #normal operation
            cmd.position._y = self.y
            cmd.position._z = self.z
        self.cmd_pub.publish(cmd)
        self.get_logger.info("Position Updated")



def main(args=None) :
    rclpy.init(args=args)
    node = GateScript()
    rclpy.spin(node)
    node.destroyNode()
    rclpy.shutdown()

if __name__ == "__main__" :
    main()