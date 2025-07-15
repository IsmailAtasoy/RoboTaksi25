#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64MultiArray
import numpy as np

class TwistFilterNode(Node):
    def __init__(self):
        super().__init__('twist_filter_node')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_velocity_max', 1.0),
                ('angular_velocity_max', 2.5),
                ('linear_acceleration_max', 0.5),
                ('angular_acceleration_max', 1.5),
                ('linear_jerk_max', 2.0),
                ('angular_jerk_max', 5.0),
                ('twist_topic', '/cmd_vel'),
                ('ackermann_topic', '/ackermann_steering_controller/reference'),
                ('diag_topic', '/filter_diagnostics'),
                ('publish_rate', 50.0),
                ('command_timeout', 0.5)
            ]
        )
        self.add_on_set_parameters_callback(self.parameter_callback)

        # State variables
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.last_linear_accel = 0.0
        self.last_angular_accel = 0.0
        self.last_cmd_time = self.get_clock().now()

        # Publishers/Subscribers
        twist_topic = self.get_parameter('twist_topic').value
        self.subscription = self.create_subscription(
            Twist, twist_topic, self.cmd_callback, 10)
        
        ackermann_topic = self.get_parameter('ackermann_topic').value
        self.publisher_ = self.create_publisher(
            TwistStamped, ackermann_topic, 10)
        
        diag_topic = self.get_parameter('diag_topic').value
        self.diag_pub = self.create_publisher(
            Float64MultiArray, diag_topic, 10)

        # Control loop
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0/publish_rate, self.publish_filtered_cmd)
        self.get_logger().info("🚀 Twist Filter Node Initialized")

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f"Parameter {param.name} changed to {param.value}")
        return SetParametersResult(successful=True)

    def cmd_callback(self, msg):
        self.target_linear_velocity = msg.linear.x
        self.target_angular_velocity = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def publish_filtered_cmd(self):
        now = self.get_clock().now()
        dt = self.timer.timer_period_ns / 1e9

        # Check timeout
        if (now - self.last_cmd_time).nanoseconds > \
           self.get_parameter('command_timeout').value * 1e9:
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0
            self.get_logger().warn("Command timeout - zeroing commands", throttle_duration_sec=1.0)

        # Clip to max values
        lin_vel_max = self.get_parameter('linear_velocity_max').value
        ang_vel_max = self.get_parameter('angular_velocity_max').value
        self.target_linear_velocity = np.clip(
            self.target_linear_velocity, -lin_vel_max, lin_vel_max)
        self.target_angular_velocity = np.clip(
            self.target_angular_velocity, -ang_vel_max, ang_vel_max)

        # Apply acceleration and jerk limits
        linear_accel = (self.target_linear_velocity - self.current_linear_velocity)/dt
        linear_accel_max = self.get_parameter('linear_acceleration_max').value
        linear_jerk_max = self.get_parameter('linear_jerk_max').value
        
        linear_accel = np.clip(linear_accel,
                             self.last_linear_accel - linear_jerk_max*dt,
                             self.last_linear_accel + linear_jerk_max*dt)
        linear_accel = np.clip(linear_accel, -linear_accel_max, linear_accel_max)
        
        self.current_linear_velocity += linear_accel * dt
        self.last_linear_accel = linear_accel

        # Same for angular
        angular_accel = (self.target_angular_velocity - self.current_angular_velocity)/dt
        angular_accel_max = self.get_parameter('angular_acceleration_max').value
        angular_jerk_max = self.get_parameter('angular_jerk_max').value
        
        angular_accel = np.clip(angular_accel,
                              self.last_angular_accel - angular_jerk_max*dt,
                              self.last_angular_accel + angular_jerk_max*dt)
        angular_accel = np.clip(angular_accel, -angular_accel_max, angular_accel_max)
        
        self.current_angular_velocity += angular_accel * dt
        self.last_angular_accel = angular_accel

        # Publish filtered command
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.current_linear_velocity
        msg.twist.angular.z = self.current_angular_velocity
        self.publisher_.publish(msg)

        # Publish diagnostics
        diag_msg = Float64MultiArray()
        diag_msg.data = [
            self.target_linear_velocity,
            self.current_linear_velocity,
            self.target_angular_velocity,
            self.current_angular_velocity,
            linear_accel,
            angular_accel
        ]
        self.diag_pub.publish(diag_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero command before shutdown
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        node.publisher_.publish(msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()