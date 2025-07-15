#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
import time
import math

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
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
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',  # LIDAR verisi
            self.scan_callback,
            10
        )
        
        # Timer for movement sequence
        self.timer = self.create_timer(0.1, self.movement_sequence)
        
        # Movement state
        self.current_step = 0  # 0: Bekleme, 1: Sol, 2: Düz, 3: Sağ, 4: Hafif Sol, 5: Tamamlandı
        self.start_time = time.time()
        self.step_duration = 3.0  
        
        # Movement parameters
        self.front_velocity = 2.0 # Ön tekerler hızı (m/s)
        self.rear_velocity = 2.0   # Arka tekerler hızı (m/s)
        self.left_angle = 1.5   # Sol teker açısı (radyan)
        self.right_angle = 1.5  # Sağ teker açısı (radyan)
        
        # Doğal hareket için parametreler
        self.wobble_frequency = 2.0  # Sallanma frekansı (Hz)
        self.wobble_amplitude = 0.06  # Sallanma genliği (radyan)
        
        # Engel algılama parametreleri
        self.obstacle_distance = 1.7 # Engel algılama mesafesi (metre)
        self.detection_angle = 30.0   # Engel algılama açısı (derece)
        self.obstacle_detected = False
        self.movement_started = False
        self.last_scan_time = 0
        
        # Yumuşak geçiş için değişkenler
        self.target_left_angle = 0.0
        self.target_right_angle = 0.0
        self.current_left_angle = 0.0
        self.current_right_angle = 0.0
        self.angle_smoothness = 0.1  # Açı geçiş yumuşaklığı (0.1 = yavaş, 0.5 = hızlı)
        
        self.get_logger().info('Obstacle Avoidance Node başlatıldı!')
        self.get_logger().info('Engel algılandığında: 2m Sol → 2m Düz → 2m Sağ → Hafif Sol')
        self.get_logger().info(f'Engel algılama mesafesi: {self.obstacle_distance} metre')
        self.get_logger().info(f'Engel algılama açısı: ±{self.detection_angle} derece')
        
    def smooth_angle_transition(self, target_left, target_right):
        """Yumuşak açı geçişi"""
        self.target_left_angle = target_left
        self.target_right_angle = target_right
        
        # Mevcut açıları hedef açılara doğru yumuşak geçiş
        self.current_left_angle += (self.target_left_angle - self.current_left_angle) * self.angle_smoothness
        self.current_right_angle += (self.target_right_angle - self.current_right_angle) * self.angle_smoothness
        
        return self.current_left_angle, self.current_right_angle
        
    def scan_callback(self, msg):
        """LIDAR verilerini işle"""
        self.last_scan_time = time.time()
        
        # Ön ±30 derecelik alanı kontrol et
        front_angle_rad = math.radians(self.detection_angle)
        center_index = len(msg.ranges) // 2
        angle_increment = msg.angle_increment
        
        # Tarama aralığı
        start_index = center_index - int(front_angle_rad / angle_increment)
        end_index = center_index + int(front_angle_rad / angle_increment)
        
        # Geçerli indeksleri kontrol et
        start_index = max(0, start_index)
        end_index = min(len(msg.ranges) - 1, end_index)
        
        # En yakın engeli bul
        min_distance = float('inf')
        
        for i in range(start_index, end_index + 1):
            if msg.ranges[i] > msg.range_min and msg.ranges[i] < msg.range_max:
                distance = msg.ranges[i]
                if distance < min_distance:
                    min_distance = distance
        
        # Engel durumunu güncelle
        if min_distance < self.obstacle_distance and not self.movement_started:
            self.obstacle_detected = True
            self.movement_started = True
            self.current_step = 1  # Hareket sırasını başlat
            self.start_time = time.time()
            self.get_logger().info(f'🚨 Engel algılandı! Mesafe: {min_distance:.2f} metre')
            self.get_logger().info('🔄 Engelden kaçınma hareketi başlatılıyor: 2m Sol → 2m Düz → 2m Sağ')
        
    def movement_sequence(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Doğal sallanma efekti
        wobble = self.wobble_amplitude * math.sin(2 * math.pi * self.wobble_frequency * elapsed_time)
        
        if self.current_step == 0:  # Engel bekleniyor
            # Düz devam et, hafif sallanma ile
            left_angle, right_angle = self.smooth_angle_transition(0.0, 0.0)
            left_angle_with_wobble = left_angle + wobble * 0.3
            right_angle_with_wobble = right_angle + wobble * 0.3
            
            self.send_position_command(left_angle_with_wobble, right_angle_with_wobble)
            self.send_velocity_command(self.front_velocity, self.rear_velocity)
            
            # Sensör verisi varsa durumu göster
            if current_time - self.last_scan_time < 1.0:  # Son 1 saniyede veri geldiyse
                self.get_logger().info(f'👀 Engel aranıyor... Düz devam ediliyor')
                
        elif self.current_step == 1:  # 2 metre sola hareket
            if elapsed_time < self.step_duration+2:
                # Sola hareket için açı komutu + sallanma
                left_angle, right_angle = self.smooth_angle_transition(-self.left_angle, -self.right_angle)
                
                # Hafif sallanma ekle
                left_angle_with_wobble = left_angle + wobble
                right_angle_with_wobble = right_angle + wobble
                
                self.send_position_command(-left_angle_with_wobble, -right_angle_with_wobble)
                self.send_velocity_command(self.front_velocity, self.rear_velocity)
                self.get_logger().info(f'Adım 1/4: 2m Sola hareket - Açı: [{left_angle_with_wobble:.3f}, {right_angle_with_wobble:.3f}] ({elapsed_time:.1f}s)')
            else:
                self.current_step = 2
                self.start_time = current_time
                self.get_logger().info('✅ 2m Sola hareket tamamlandı! Şimdi düz hareket...')
                
        elif self.current_step == 2:  # 2 metre düz hareket
            if elapsed_time < self.step_duration-1:
                # Düz hareket için açı komutu + hafif sağa yönlendirme
                left_angle, right_angle = self.smooth_angle_transition(self.left_angle * 0.1, self.right_angle * 0.1)
                
                # Hafif sallanma ekle
                left_angle_with_wobble = left_angle + wobble * 0.1
                right_angle_with_wobble = right_angle + wobble * 0.1
                
                self.send_position_command(-left_angle_with_wobble, -right_angle_with_wobble)
                self.send_velocity_command(self.front_velocity, self.rear_velocity)
                self.get_logger().info(f'Adım 2/4: 2m Düz hareket (hafif sağa) - Açı: [{left_angle_with_wobble:.3f}, {right_angle_with_wobble:.3f}] ({elapsed_time:.1f}s)')
            else:
                self.current_step = 3
                self.start_time = current_time
                self.get_logger().info('✅ 2m Düz hareket (hafif sağa) tamamlandı! Şimdi sağa hareket...')
                
        elif self.current_step == 3:  # 2 metre sağa hareket
            if elapsed_time < self.step_duration+12:
                # Sağa hareket için açı komutu + hafif sola yönlendirme
                left_angle, right_angle = self.smooth_angle_transition(self.left_angle * 0.3, self.right_angle * 0.3)
                
                # Hafif sallanma ekle
                left_angle_with_wobble = left_angle + wobble *0.1
                right_angle_with_wobble = right_angle + wobble*0.1
                
                self.send_position_command(-left_angle_with_wobble, -right_angle_with_wobble)
                self.send_velocity_command(self.front_velocity, self.rear_velocity)
                self.get_logger().info(f'Adım 3/4: 2m Sağa hareket (hafif sola) - Açı: [{left_angle_with_wobble:.3f}, {right_angle_with_wobble:.3f}] ({elapsed_time:.1f}s)')
            else:
                self.current_step = 4
                self.start_time = current_time
                self.get_logger().info('✅ 2m Sağa hareket (hafif sola) tamamlandı! Şimdi hafif sola hareket...')
                
        elif self.current_step == 4:  # Hafif sola hareket
            if elapsed_time < self.step_duration:
                # Hafif sola hareket için açı komutu
                left_angle, right_angle = self.smooth_angle_transition(-self.left_angle * 0.1, -self.right_angle * 0.1)
                
                # Hafif sallanma ekle
                left_angle_with_wobble = left_angle + wobble * 0.1
                right_angle_with_wobble = right_angle + wobble * 0.1
                
                self.send_position_command(-left_angle_with_wobble, -right_angle_with_wobble)
                self.send_velocity_command(self.front_velocity, self.rear_velocity)
                self.get_logger().info(f'Adım 4/4: Hafif Sola Hareket - Açı: [{left_angle_with_wobble:.3f}, {right_angle_with_wobble:.3f}] ({elapsed_time:.1f}s)')
            else:
                self.current_step = 5
                self.start_time = current_time
                self.get_logger().info('✅ Hafif Sola Hareket tamamlandı! Engelden kaçınma tamamlandı!')
                
        elif self.current_step == 5:  # Hareket tamamlandı
            # Durdur - açıları sıfırla
            left_angle, right_angle = self.smooth_angle_transition(0.0, 0.0)
            self.send_position_command(0.0, 0.0)
            self.send_velocity_command(0.0, 0.0)
            self.get_logger().info('🎉 Engelden kaçınma hareketi tamamlandı! Araç durduruldu.')
            # Timer'ı durdur
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

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node durduruldu (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 