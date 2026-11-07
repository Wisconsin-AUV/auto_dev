import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Float32MultiArray
from collections import deque
import time
import serial

# /joystick layout: data[0:6] = axes, then buttons, then hats [x, y]
SOL_BUTTONS = {6: 1, 7: 2, 8: 3, 9: 4}   # A, B, X, Y
HAT_X = -2               # hat x is second-to-last
STEP_CHUNK = 100         # steps per MOVE; smaller = faster stop on release, more stutter
STEP_SPEED = 1600        # steps/s sent once at startup
JOYSTICK_TIMEOUT = 0.3   # s; treat hat as released if /joystick goes stale
MOVE_TIMEOUT = 2.0       # s; give up waiting for a MOVE's OK


class XboxController(Node):

    def __init__(self):
        super().__init__('xbox_controller')

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.yaw = 0.0

        self.btn_prev = {i: False for i in SOL_BUTTONS}
        self.sol_state = {n: 0 for n in SOL_BUTTONS.values()}
        self.rx_buf = b''

        self.jog_dir = 0
        self.last_joy_time = 0.0
        self.pending = deque()        # commands awaiting a reply, in send order
        self.move_pending = False
        self.move_sent_at = 0.0

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)
        self.ser.dtr = True
        self.ser.write(b'\x03\x03')
        time.sleep(0.2)
        self.ser.write(b'\x04')
        time.sleep(1.5)
        self.ser.reset_input_buffer()
        self.send('OFF')
        self.send(f'SPEED {STEP_SPEED}')

        self.cmd_pub = self.create_publisher(ManualControl, 'mavros/manual_control/send', 10)
        self.xbox_sub = self.create_subscription(Float32MultiArray, '/joystick', self.xbox_cb, 10)
        self.timer = self.create_timer(0.05, self.command_loop)
        self.step_timer = self.create_timer(0.01, self.stepper_loop)  # fast so chunks run back-to-back

        self.get_logger().info("XBOX controller started")

    # ---------- serial ----------
    def send(self, line, log=True):
        self.ser.write((line + '\n').encode())
        self.pending.append(line.split()[0])
        if log:
            self.get_logger().info(f"-> {line}")

    def read_replies(self):
        n = self.ser.in_waiting
        if not n:
            return
        self.rx_buf += self.ser.read(n)
        *lines, self.rx_buf = self.rx_buf.split(b'\n')
        for raw in lines:
            reply = raw.decode(errors='ignore').strip()
            if not reply:
                continue
            cmd = self.pending.popleft() if self.pending else None
            if cmd == 'MOVE':
                self.move_pending = False
            if reply.startswith('ERR'):
                self.get_logger().warn(f"Pico ({cmd}): {reply}")

    # ---------- joystick ----------
    def xbox_cb(self, msg):
        d = msg.data
        if len(d) <= max(SOL_BUTTONS) + 2:
            self.get_logger().warn(f"/joystick too short: {len(d)}")
            return

        self.linear_y = d[0]
        self.linear_x = -d[1]
        self.linear_z = ((1 - d[5]) / 2) - ((1 - d[4]) / 2)
        self.yaw = d[2]

        for idx, sol in SOL_BUTTONS.items():
            pressed = (d[idx] == 1.0)
            if self.btn_prev[idx] and not pressed:
                self.sol_state[sol] ^= 1
                self.send(f"SOL {sol} {self.sol_state[sol]}")
            self.btn_prev[idx] = pressed

        hx = d[HAT_X]
        self.jog_dir = -1 if hx < -0.5 else (1 if hx > 0.5 else 0)   # left=-1, right=+1
        self.last_joy_time = time.monotonic()

    # ---------- stepper ----------
    def stepper_loop(self):
        self.read_replies()
        now = time.monotonic()

        if now - self.last_joy_time > JOYSTICK_TIMEOUT:
            self.jog_dir = 0

        if self.move_pending and now - self.move_sent_at > MOVE_TIMEOUT:
            self.get_logger().warn("MOVE reply timed out")
            self.move_pending = False
            self.pending.clear()

        if self.jog_dir and not self.move_pending:
            self.send(f"MOVE {self.jog_dir * STEP_CHUNK}", log=False)
            self.move_pending = True
            self.move_sent_at = now

    def command_loop(self):
        cmd = ManualControl()
        cmd.x = self.linear_x * 1000 * 0.3
        cmd.y = self.linear_y * 1000 * 0.3
        cmd.z = ((self.linear_z * 500) + 500) * 0.3
        cmd.r = self.yaw * 1000 * 0.3
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