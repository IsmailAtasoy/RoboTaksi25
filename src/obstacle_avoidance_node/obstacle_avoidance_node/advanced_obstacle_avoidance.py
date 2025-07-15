#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import math

class AdvancedObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('advanced_obstacle_avoidance_node')
        
        # Publishers
        self.position_pub = self.create_publisher(
            Float64MultiArray, 
            '/position_controller/commands', 
            10
        )
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, 
            '/velocity_controller/commands', 
            10
        )
        
        # Parameters
        self.declare_parameter('front_velocity', 0.5)
        self.declare_parameter('rear_velocity', 0.5)
        self.declare_parameter('left_angle', 1.5)
        self.declare_parameter('right_angle', 1.5)
        self.declare_parameter('step_distance', 2.0)  # metre
        self.declare_parameter('step_duration', 15.0)  # saniye
        self.declare_parameter('wobble_frequency', 2.0)  # Sallanma frekansı (Hz)
        self.declare_parameter('wobble_amplitude', 0.1)  # Sallanma genliği (radyan)
        
        # Get parameters
        self.front_velocity = self.get_parameter('front_velocity').value
        self.rear_velocity = self.get_parameter('rear_velocity').value
        self.left_angle = self.get_parameter('left_angle').value
        self.right_angle = self.get_parameter('right_angle').value
        self.step_distance = self.get_parameter('step_distance').value
        self.step_duration = self.get_parameter('step_duration').value
        self.wobble_frequency = self.get_parameter('wobble_frequency').value
        self.wobble_amplitude = self.get_parameter('wobble_amplitude').value
        
        # Timer for movement sequence
        self.timer = self.create_timer(0.1, self.movement_sequence)
        
        # Movement state
        self.current_step = 0
        self.start_time = time.time()
        self.movement_complete = False
        
        # Movement steps definition - sadece sola dönüş
        self.movement_steps = [
            {
                'name': 'Sola dönüp engelden kaçınma',
                'left_angle': -self.left_angle,
                'right_angle': -self.right_angle,
                'front_vel': self.front_velocity,
                'rear_vel': self.rear_velocity,
                'duration': self.step_duration,
                'description': 'Sola dönüp engelden kaçınana kadar devam et',
                'wobble_factor': 1.0  # Tam sallanma
            }
        ]
        
        self.get_logger().info('Advanced Obstacle Avoidance Node başlatıldı!')
        self.get_logger().info(f'Parametreler: Ön Hız={self.front_velocity} m/s, Arka Hız={self.rear_velocity} m/s')
        self.get_logger().info(f'Açılar: Sol={self.left_angle} rad, Sağ={self.right_angle} rad')
        self.get_logger().info(f'Sallanma: Frekans={self.wobble_frequency} Hz, Genlik={self.wobble_amplitude} rad')
        self.get_logger().info(f'Engelden kaçınma süresi: {self.step_duration} saniye')
        
    def movement_sequence(self):
        if self.movement_complete:
            return
            
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        if self.current_step < len(self.movement_steps):
            step = self.movement_steps[self.current_step]
            
            if elapsed_time < step['duration']:
                # Doğal sallanma efekti
                wobble = self.wobble_amplitude * math.sin(2 * math.pi * self.wobble_frequency * elapsed_time)
                wobble = wobble * step['wobble_factor']  # Adıma göre sallanma faktörü
                
                # Açı ve hız komutlarını gönder
                left_angle_with_wobble = step['left_angle'] + wobble
                right_angle_with_wobble = step['right_angle'] + wobble
                
                self.send_position_command(left_angle_with_wobble, right_angle_with_wobble)
                self.send_velocity_command(step['front_vel'], step['rear_vel'])
                
                # Progress bilgisi
                progress = (elapsed_time / step['duration']) * 100
                self.get_logger().info(f'{step["name"]}: {progress:.1f}% tamamlandı ({elapsed_time:.1f}s/{step["duration"]}s)')
                self.get_logger().info(f'  Açılar: Sol={left_angle_with_wobble:.3f}, Sağ={right_angle_with_wobble:.3f}')
                self.get_logger().info(f'  Hız: Ön={step["front_vel"]:.2f}, Arka={step["rear_vel"]:.2f}')
            else:
                # Adım tamamlandı
                self.get_logger().info(f'✓ {step["name"]} tamamlandı: {step["description"]}')
                self.current_step += 1
                self.start_time = current_time
                
                if self.current_step >= len(self.movement_steps):
                    # Tüm hareketler tamamlandı
                    self.send_position_command(0.0, 0.0)
                    self.send_velocity_command(0.0, 0.0)
                    self.get_logger().info('🎉 Engelden kaçınma hareketi başarıyla tamamlandı!')
                    self.movement_complete = True
                    self.timer.cancel()
        else:
            # Durdur
            self.send_position_command(0.0, 0.0)
            self.send_velocity_command(0.0, 0.0)
            self.movement_complete = True
            self.timer.cancel()
    
    def send_velocity_command(self, front_vel, rear_vel):
        """Hız komutunu gönder (ön tekerler, arka tekerler)"""
        msg = Float64MultiArray()
        msg.data = [front_vel, rear_vel]
        self.velocity_pub.publish(msg)
    
    def send_position_command(self, left_angle, right_angle):
        """Açı komutunu gönder (sol teker, sağ teker)"""
        msg = Float64MultiArray()
        msg.data = [left_angle, right_angle]
        self.position_pub.publish(msg)
    
    def reset_movement(self):
        """Hareketi sıfırla"""
        self.current_step = 0
        self.start_time = time.time()
        self.movement_complete = False
        self.get_logger().info('Hareket sıfırlandı, yeniden başlatılıyor...')

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedObstacleAvoidanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node durduruldu (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 