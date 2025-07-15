import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys, tty, termios, select

class TeleopAckermannDual(Node):
    def __init__(self):
        super().__init__('teleop_ackermann_dual')

        self.speed = 0.0
        self.steering = 0.0
        self.speed_step = 0.1
        self.steering_step = 0.05
        self.max_steering = 0.5

        self.steer_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.timer = self.create_timer(0.1, self.publish_commands)
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_commands(self):
        key = self.get_key()

        if key == 'w':
            self.speed += self.speed_step
        elif key == 's':
            self.speed -= self.speed_step
        elif key == 'a':
            self.steering += self.steering_step
        elif key == 'd':
            self.steering -= self.steering_step
        elif key == 'x':
            self.speed = 0.0
            self.steering = 0.0

        self.steering = max(min(self.steering, self.max_steering), -self.max_steering)

        # Direksiyon açılarını ayarla (ön iki teker için)
        steer_msg = Float64MultiArray()
        steer_msg.data = [self.steering, self.steering]
        self.steer_pub.publish(steer_msg)

        # Hızları ayarla (arka iki teker için)
        vel_msg = Float64MultiArray()
        vel_msg.data = [self.speed, self.speed]
        self.vel_pub.publish(vel_msg)

        print(f"🔄 Steering: {self.steering:.2f} rad | Speed: {self.speed:.2f} m/s")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopAckermannDual()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
