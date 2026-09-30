import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import json
import socket


class joy_node(Node):

    def __init__(self):
        super().__init__('joy_node')

        self.cmd_pub = self.create_publisher(Float32MultiArray, '/joystick', 10)

        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.PORT = 6767
        self.receiver.bind(('0.0.0.0', self.PORT))
        self.receiver.setblocking(False)
        self.get_logger().info(f"Jetson is listening to xbox on port {self.PORT}...")

        # Poll a bit faster than the sender's 50 Hz
        self.timer = self.create_timer(0.01, self.command_loop)

    def command_loop(self):
        latest = None

        # only the newest packet
        while True:
            try:
                data, _ = self.receiver.recvfrom(1024)
                latest = data
            except BlockingIOError:
                break

        if latest is None:
            return
        
        try:
            joy_msg = json.loads(latest.decode('utf-8'))
            joy = Float32MultiArray()
            joy.data = [float(x) for x in joy_msg["axes"]]
            self.cmd_pub.publish(joy)

        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().warn(f"Bad packet: {e}")

    def destroy_node(self):
        self.receiver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = joy_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()