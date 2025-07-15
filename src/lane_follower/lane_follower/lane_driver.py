#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, Float64MultiArray, String # String ekledik!

class LaneControlDirect(Node):
    def __init__(self):
        super().__init__('lane_control_direct')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('steering_gain', 0.5),
                ('base_speed', 0.5),
                ('max_steering', 0.5),
                ('timeout_sec', 1.0),
                ('min_speed', 0.2)
            ]
        )
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Subscribers/Publishers
        self.offset_sub = self.create_subscription(
            Float32, '/lane_offset', self.offset_callback, 10)
        self.steer_pub = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)
        self.vel_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)

        # YENİ EKLENEN: Robot durumuna abone ol
        self.robot_state_sub = self.create_subscription(
            String, # RobotStateController'dan String mesajı bekliyoruz
            '/robot_current_state',
            self.robot_state_callback,
            10
        )
        self.current_robot_state = "unknown" # Başlangıç durumu

        # Control variables
        self.timer = self.create_timer(0.05, self.send_commands)  # 20 Hz
        self.current_offset = 0.0
        self.filtered_offset = 0.0
        self.last_offset_time = self.get_clock().now()

        self.get_logger().info("Lane Control Direct Node Initialized.")

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f"Parameter {param.name} changed to {param.value}")
        return SetParametersResult(successful=True)

    def offset_callback(self, msg):
        self.current_offset = msg.data
        self.filtered_offset = 0.8 * self.filtered_offset + 0.2 * self.current_offset
        self.last_offset_time = self.get_clock().now()

    def robot_state_callback(self, msg: String):
        """Robotun mevcut durumunu alır ve buna göre davranışı ayarlar."""
        old_state = self.current_robot_state
        self.current_robot_state = msg.data
        if old_state != self.current_robot_state:
            self.get_logger().info(f"Robot State changed from {old_state} to {self.current_robot_state}")
            if self.current_robot_state != "lane_following": # Eğer şerit takibi değilse, durdur
                self.get_logger().info("Not in LANE_FOLLOWING state, stopping lane control commands.")
                # Şerit takibi devredışı kaldığında robotu durdurma komutları gönder
                vel_msg = Float64MultiArray()
                vel_msg.data = [0.0, 0.0]
                self.vel_pub.publish(vel_msg)
                steer_msg = Float64MultiArray()
                steer_msg.data = [0.0, 0.0]
                self.steer_pub.publish(steer_msg)

    def send_commands(self):
        # YALNIZCA ROBOT DURUMU 'lane_following' OLDUĞUNDA KONTROL ET
        if self.current_robot_state != "lane_following":
            # self.get_logger().info(f"Lane control inactive (state: {self.current_robot_state})", throttle_duration_sec=1.0)
            return # Komut yayınlamayı durdur

        # Check for timeout
        if (self.get_clock().now() - self.last_offset_time).nanoseconds > \
           self.get_parameter('timeout_sec').value * 1e9:
            self.get_logger().warn("No offset updates - stopping vehicle for lane control", throttle_duration_sec=1.0)
            vel_msg = Float64MultiArray()
            vel_msg.data = [0.0, 0.0]
            self.vel_pub.publish(vel_msg)
            return

        # Calculate steering
        steering_gain = self.get_parameter('steering_gain').value
        max_steering = self.get_parameter('max_steering').value
        steering_angle = self.filtered_offset * steering_gain
        steering_angle = max(min(steering_angle, max_steering), -max_steering)

        # Calculate speed (reduce when turning sharply)
        base_speed = self.get_parameter('base_speed').value
        min_speed = self.get_parameter('min_speed').value
        speed_factor = 1.0 - 0.4 * abs(steering_angle)/max_steering
        adjusted_speed = max(min_speed, base_speed * speed_factor)

        # Publish commands
        steer_msg = Float64MultiArray()
        steer_msg.data = [steering_angle, steering_angle]
        self.steer_pub.publish(steer_msg)

        vel_msg = Float64MultiArray()
        vel_msg.data = [adjusted_speed, adjusted_speed]
        self.vel_pub.publish(vel_msg)

        self.get_logger().info(
            f"Direct: offset={self.filtered_offset:.2f} | steer={steering_angle:.2f} | speed={adjusted_speed:.2f} | State: {self.current_robot_state}",
            throttle_duration_sec=0.5)

def main(args=None):
    rclpy.init(args=args)
    node = LaneControlDirect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean stop
        vel_msg = Float64MultiArray()
        vel_msg.data = [0.0, 0.0]
        node.vel_pub.publish(vel_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()