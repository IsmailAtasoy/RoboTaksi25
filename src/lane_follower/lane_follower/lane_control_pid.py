#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, Float64MultiArray, String # String ekledik!
import time

class LaneControlPID(Node):
    def __init__(self):
        super().__init__('lane_control_pid')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 0.8),
                ('ki', 0.02),
                ('kd', 0.1),
                ('max_output', 0.5),
                ('base_speed', 0.8),
                ('min_speed', 0.5),
                ('timeout_sec', 1.0)
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
            String, # RobotStateController'dan String bekliyoruz
            '/robot_current_state',
            self.robot_state_callback,
            10
        )
        self.current_robot_state = "unknown" # Başlangıç durumu
        self.get_logger().info(f"LaneControlPID: Initial robot state set to {self.current_robot_state}.") # Yeni log

        # Control variables
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.offset = 0.0
        self.filtered_offset = 0.0
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.last_offset_time = self.get_clock().now()

        self.get_logger().info("Lane Control PID Node Initialized.")


    def parameter_callback(self, params):
        for param in params:
            if param.name == 'kp': self.get_logger().info(f"kp changed to {param.value}")
            elif param.name == 'ki': self.get_logger().info(f"ki changed to {param.value}")
            elif param.name == 'kd': self.get_logger().info(f"kd changed to {param.value}")
            elif param.name == 'max_output': self.get_logger().info(f"max_output changed to {param.value}")
            elif param.name == 'base_speed': self.get_logger().info(f"base_speed changed to {param.value}")
            elif param.name == 'min_speed': self.get_logger().info(f"min_speed changed to {param.value}")
            elif param.name == 'timeout_sec': self.get_logger().info(f"timeout_sec changed to {param.value}")
        return SetParametersResult(successful=True)

    def offset_callback(self, msg):
        self.offset = msg.data
        self.filtered_offset = 0.7 * self.filtered_offset + 0.3 * self.offset
        self.last_offset_time = self.get_clock().now()

    def robot_state_callback(self, msg: String):
        """Robotun mevcut durumunu alır ve buna göre davranışı ayarlar."""
        old_state = self.current_robot_state
        self.current_robot_state = msg.data
        if old_state != self.current_robot_state:
            self.get_logger().info(f"Robot State changed from {old_state} to {self.current_robot_state}")
            if self.current_robot_state != "lane_following": # Eğer şerit takibi değilse, durdur
                self.get_logger().info("Not in LANE_FOLLOWING state, stopping lane control commands.")
                vel_msg = Float64MultiArray()
                vel_msg.data = [0.0, 0.0]
                self.vel_pub.publish(vel_msg)
                steer_msg = Float64MultiArray()
                steer_msg.data = [0.0, 0.0]
                self.steer_pub.publish(steer_msg)
            else:
                self.get_logger().info("LaneControlPID: Now in LANE_FOLLOWING state. PID control should resume if offset data is available.")

    def control_loop(self):
        # YALNIZCA ROBOT DURUMU 'LANE_FOLLOWING' OLDUĞUNDA KONTROL ET
        if self.current_robot_state != "lane_following":
            # self.get_logger().info(f"Lane control inactive (state: {self.current_robot_state})", throttle_duration_sec=1.0)
            return # Komut yayınlamayı durdur

        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        if dt <= 0.0:
            return

        # Check for timeout
        if (self.get_clock().now() - self.last_offset_time).nanoseconds > \
           self.get_parameter('timeout_sec').value * 1e9:
            self.get_logger().warn("No offset updates - stopping vehicle for lane control", throttle_duration_sec=1.0)
            vel_msg = Float64MultiArray()
            vel_msg.data = [0.0, 0.0]
            self.vel_pub.publish(vel_msg)
            return

        # PID calculation
        error = self.filtered_offset
        self.integral += error * dt
        
        # Anti-windup
        if abs(self.integral) > 2.0:  # Arbitrary large value
            self.integral *= 0.99  # Leaky integration

        derivative = (error - self.last_error) / dt

        # Get parameters
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        max_output = self.get_parameter('max_output').value

        steer_output = kp * error + ki * self.integral + kd * derivative
        steer_output = max(min(steer_output, max_output), -max_output)

        # Speed adjustment based on steering
        base_speed = self.get_parameter('base_speed').value
        min_speed = self.get_parameter('min_speed').value
        speed_factor = 1.0 - 0.5 * abs(steer_output)/max_output
        adjusted_speed = max(min_speed, base_speed * speed_factor)

        # Publish commands
        steer_msg = Float64MultiArray()
        steer_msg.data = [steer_output, steer_output]
        self.steer_pub.publish(steer_msg)

        vel_msg = Float64MultiArray()
        vel_msg.data = [adjusted_speed, adjusted_speed]
        self.vel_pub.publish(vel_msg)

        # Update state
        self.last_error = error
        self.last_time = current_time

        self.get_logger().info(
            f"PID: offset={error:.2f} | steer={steer_output:.2f} | speed={adjusted_speed:.2f} | State: {self.current_robot_state}",
            throttle_duration_sec=0.5)
        
    

def main(args=None):
    rclpy.init(args=args)
    node = LaneControlPID()
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