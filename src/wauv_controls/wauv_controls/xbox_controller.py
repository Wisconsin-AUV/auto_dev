import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Float32MultiArray

class XboxController(Node):

    def __init__(self):
        # Initialise the ROS2 node with name 'manual_controller'
        super().__init__('xbox_controller')

        # Publisher sends velocity commands (Twist messages)
        # Topic: MAVROS velocity command topic
        self.cmd_pub = self.create_publisher(
            ManualControl,
            'mavros/manual_control/send',
            10
        )

        self.xbox_sub = self.create_subscription(Float32MultiArray, '/joystick', self.xbox_cb ,10)

        # Timer creates a loop running at 20 Hz (every 0.05 seconds)
        # This repeatedly calls command_loop()
        self.timer = self.create_timer(0.05, self.command_loop)

        # Current velocity state (these get updated by keyboard input)
        self.linear_x = 0.0  # forward/backward
        self.linear_y = 0.0  # left/right
        self.linear_z = 0.0  # up/down
        self.yaw = 0.0

        # Log message to confirm node has started
        self.get_logger().info("XBOX controller started")

    def xbox_cb(self, msg):
        self.linear_y = msg.data[0]
        self.linear_x = -1 * msg.data[1]
        self.linear_z = ((1 - msg.data[5]) / 2) - ((1 - msg.data[4]) / 2)
        self.yaw = msg.data[2]

        #[0] = left stick left -1 right +1
        #[1] = left stick up -1 down +1
        #[2] = right stick left -1 right +1
        #[4][5] is lt and rt and -1.0 on rest and 1.0 on full press

    def command_loop(self):
        """
        Runs at 20 Hz.
        Publishes the current velocity as a Twist message.
        """
        cmd = ManualControl()
        cmd.x = self.linear_x * 1000 *0.3
        cmd.y = self.linear_y * 1000 *0.3
        cmd.z = ((self.linear_z * 500) + 500) *0.3
        cmd.r = self.yaw * 1000 *0.3
        # Publish command to MAVROS
        self.cmd_pub.publish(cmd)


def main(args=None):
    # Initialise ROS2 communication
    rclpy.init(args=args)

    # Create the node
    node = XboxController()

    # Keep node running and processing callbacks
    rclpy.spin(node)

    # Cleanup when shutting down
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
