import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import time
import serial

# /joystick layout: data[0:6] = axes, data[6:] = buttons
# Button index in data -> solenoid number on the Pico
SOL_BUTTONS = {6: 1, 7: 2, 8: 3, 9: 4}   # A, B, X, Y 


class XboxController(Node):

    def __init__(self):
        super().__init__('xbox_controller')

        self.cmd_pub = self.create_publisher(ManualControl, 'mavros/manual_control/send', 10)
        self.xbox_sub = self.create_subscription(Float32MultiArray, '/joystick', self.xbox_cb, 10)
        self.timer = self.create_timer(0.05, self.command_loop)

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.yaw = 0.0

        self.btn_prev = {i: False for i in SOL_BUTTONS}
        self.sol_state = {n: 0 for n in SOL_BUTTONS.values()}
        self.rx_buf = b''

        # timeout=0 -> reads never block the executor
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
        self.ser.dtr = True
        self.ser.write(b'\x03\x03')   # Ctrl-C: interrupt whatever is running
        time.sleep(0.2)
        self.ser.write(b'\x04')       # Ctrl-D: soft reboot -> runs main.py
        time.sleep(1.5)
        self.ser.reset_input_buffer() # drop REPL/boot banner
        self.send('OFF')

        self.get_logger().info("XBOX controller started")

    # ---------- serial ----------
    def send(self, line):
        self.ser.write((line + '\n').encode())
        self.get_logger().info(f"-> {line}")

    def read_replies(self):
        """Drain the Pico's OK/ERR replies without blocking.
        If they're never read, its USB buffer fills up and print() blocks the Pico."""
        n = self.ser.in_waiting
        if not n:
            return
        self.rx_buf += self.ser.read(n)
        *lines, self.rx_buf = self.rx_buf.split(b'\n')
        for raw in lines:
            reply = raw.decode(errors='ignore').strip()
            if reply.startswith('ERR'):
                self.get_logger().warn(f"Pico: {reply}")

    # ---------- joystick ----------
    def xbox_cb(self, msg):
        d = msg.data
        if len(d) <= max(SOL_BUTTONS):
            self.get_logger().warn(f"/joystick too short: {len(d)}")
            return

        self.linear_y = d[0]
        self.linear_x = -d[1]
        self.linear_z = ((1 - d[5]) / 2) - ((1 - d[4]) / 2)
        self.yaw = d[2]

        # toggle a solenoid on button release
        for idx, sol in SOL_BUTTONS.items():
            pressed = (d[idx] == 1.0)
            if self.btn_prev[idx] and not pressed:
                self.sol_state[sol] ^= 1
                self.send(f"SOL {sol} {self.sol_state[sol]}")
            self.btn_prev[idx] = pressed

    def command_loop(self):
        self.read_replies()

        cmd = ManualControl()
        cmd.x = self.linear_x * 1000 * 0.3
        cmd.y = self.linear_y * 1000 * 0.3
        cmd.z = ((self.linear_z * 500) + 500) * 0.3
        cmd.r = self.yaw * 1000 * 0.3
        # Publish command to MAVROS
        self.cmd_pub.publish(cmd)

    def destroy_node(self):
        try:
            self.send('OFF')
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = XboxController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()