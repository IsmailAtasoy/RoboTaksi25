import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool # TrafficSignDetector'dan gelen park algılama için
from std_msgs.msg import Float64MultiArray # Konum ve Hız komutları için
import math
import time

class LParkNode(Node):

    def __init__(self):
        super().__init__('l_park_node')

        # Yeni Publisher'lar: Direksiyon Pozisyonu ve Tekerlek Hızı için
        self.position_publisher_ = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.velocity_publisher_ = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # L-Park algoritması tamamlandığında yayınlamak için publisher
        self.l_park_completion_publisher = self.create_publisher(Bool, '/l_park_algorithm_completed', 10)

        # Timer'ı başlangıçta oluşturma, tetiklendiğinde başlatacağız
        self.timer = None
        self.step = 0
        self.start_time = 0.0
        self.phase_start_time = 0.0
        self.get_logger().info('L Park Node started!')

        # Robotun hareket parametreleri
        self.base_linear_speed = 0.4  # m/s (Önceki Twist hızlarına benzer bir taban hız)
        self.max_steering_angle = math.radians(30) # rad (Direksiyonun maksimum açısı, ayarlayın)
        self.wheel_radius = 0.1 # Metre cinsinden tekerlek yarıçapı. ROBOT MODELİNİZE GÖRE AYARLAYIN!
                                # Yanlışsa, tekerlek hızı yanlış hesaplanır.

        # Rampalama için sabitler (isteğe bağlı olarak daha yumuşak geçişler için eklenebilir)
        self.ramp_steps = 10  # Rampalama için adım sayısı
        self.ramp_interval = 0.1 # Saniye cinsinden her adım arasındaki bekleme süresi

        # --- PARK LEVHASI ALGILAMA SUBSCRIBER'I ---
        self.park_sign_subscription_ = self.create_subscription(
            Bool,
            '/park_sign_detected_signal',
            self.park_sign_callback,
            10
        )
        self.get_logger().info('LParkNode subscribed to /park_sign_detected_signal.')

        # Park sekansının zaten çalışıp çalışmadığını kontrol etmek için bayrak
        self.park_sequence_active = False
        self.has_executed_park_sequence = False

    def publish_steering_position(self, left_angle, right_angle):
        """Direksiyon eklemlerinin pozisyonunu yayınlar (radyan cinsinden)."""
        msg = Float64MultiArray()
        # Genellikle robotlarda ön tekerlekler aynı açıda döner
        # Eğer direksiyon mekanizmanız farklı ise burayı ayarlamanız gerekebilir.
        msg.data = [float(left_angle), float(right_angle)]
        self.position_publisher_.publish(msg)
        self.get_logger().info(f'Publishing Steering Positions: Left={math.degrees(left_angle):.2f} deg, Right={math.degrees(right_angle):.2f} deg')

    def publish_wheel_velocity(self, left_velocity, right_velocity):
        """Arka tekerleklerin açısal hızını (rad/s) yayınlar."""
        msg = Float64MultiArray()
        msg.data = [float(left_velocity), float(right_velocity)]
        self.velocity_publisher_.publish(msg)
        self.get_logger().info(f'Publishing Wheel Velocities: Left={left_velocity:.2f} rad/s, Right={right_velocity:.2f} rad/s')

    def stop_robot(self):
        """Robotu durdurur."""
        self.get_logger().info('Stopping robot.')
        self.publish_steering_position(0.0, 0.0) # Direksiyonu sıfırla
        self.publish_wheel_velocity(0.0, 0.0) # Tekerlek hızlarını sıfırla
        time.sleep(0.5) # Komutların işlenmesi için kısa bekleme

    def park_sign_callback(self, msg: Bool):
        """
        TrafficSignDetector'dan gelen park levhası algılama mesajlarını işler.
        'True' mesajı alındığında ve sekans aktif değilse, L park sekansını başlatır.
        """
        if msg.data and not self.park_sequence_active and not self.has_executed_park_sequence:
            self.get_logger().info('Park levhası algılandı! L park sekansı başlatılıyor (TERS YÖN)...')
            self.park_sequence_active = True
            self.has_executed_park_sequence = True

            self.step = 0
            self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
            self.phase_start_time = self.start_time
            # Timer'ı başlat
            self.timer = self.create_timer(0.1, self.timer_callback)
            self.get_logger().info('L Park sekansı timer başlatıldı.')

        elif msg.data and self.park_sequence_active:
            self.get_logger().info('Park levhası algılandı, ancak L park sekansı zaten aktif. Şimdilik yok sayılıyor.')
        elif msg.data and self.has_executed_park_sequence:
            self.get_logger().info('Park levhası algılandı, ancak L park sekansı zaten bir kez çalıştırıldı. Yok sayılıyor.')

    def timer_callback(self):
        # NOT: Bu L-Park sekansı adımları, robotunuzun fiziksel dinamiklerine ve PID kontrolörlerinize göre
        # hassas ayar gerektirecektir. Belirtilen süreler ve açılar sadece başlangıç noktalarıdır.

        now = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed = now - self.phase_start_time

        # Lineer hızı açısal tekerlek hızına çevir: v = w * r => w = v / r
        target_wheel_velocity = self.base_linear_speed / self.wheel_radius

        if self.step == 0:
            # 1. Biraz ileri git (1.2 saniye) - İlk L-Park adımı gibi
            self.publish_steering_position(0.0, 0.0) # Düz
            self.publish_wheel_velocity(target_wheel_velocity, target_wheel_velocity)
            if elapsed > 1.2:
                self.step += 1
                self.phase_start_time = now
                self.get_logger().info(f'L Park Step {self.step}: Moving forward slightly.')
        elif self.step == 1:
            # 2. Sol ön tekeri park alanına sokacak şekilde sola keskin dönüş (5.0 saniye)
            steering_angle = self.max_steering_angle # Tam sola kır (pozitif açı)
            self.publish_steering_position(steering_angle, steering_angle)
            self.publish_wheel_velocity(target_wheel_velocity * 0.8, target_wheel_velocity * 0.8) # Dönüş için hızı biraz düşür
            if elapsed > 5.0: # Bu süreyi denemelerle ayarlayın
                self.step += 1
                self.phase_start_time = now
                self.get_logger().info(f'L Park Step {self.step}: Sharp left turn.')
        elif self.step == 2:
            # 3. Araç park alanına girerken sağa çevirerek hizala (0.7 saniye)
            steering_angle = -self.max_steering_angle * 0.5 # Yarı sağa kır (negatif açı)
            self.publish_steering_position(steering_angle, steering_angle)
            self.publish_wheel_velocity(target_wheel_velocity * 0.7, target_wheel_velocity * 0.7) # Hızı biraz daha düşür
            if elapsed > 0.7: # Bu süreyi denemelerle ayarlayın
                self.step += 1
                self.phase_start_time = now
                self.get_logger().info(f'L Park Step {self.step}: Aligning right into spot.')
        elif self.step == 3:
            # 4. Düz ilerleyerek park alanına tamamen gir (2.0 saniye)
            self.publish_steering_position(0.0, 0.0) # Düz
            self.publish_wheel_velocity(target_wheel_velocity * 0.6, target_wheel_velocity * 0.6) # Hızı düşür
            if elapsed > 2.0: # Bu süreyi denemelerle ayarlayın
                self.step += 1
                self.phase_start_time = now
                self.get_logger().info(f'L Park Step {self.step}: Entering park spot straight.')
        elif self.step == 4:
            # 5. Küçük bir düzeltme: sağa veya sola çok hafif dönerek paralel hizala (3.0 saniye)
            steering_angle = math.radians(5) # Çok hafif sola düzeltme (deneyerek ayarla)
            self.publish_steering_position(steering_angle, steering_angle)
            self.publish_wheel_velocity(target_wheel_velocity * 0.3, target_wheel_velocity * 0.3) # Çok yavaş hareket
            if elapsed > 3.0: # Bu süreyi denemelerle ayarlayın
                self.step += 1
                self.phase_start_time = now
                self.get_logger().info(f'L Park Step {self.step}: Minor alignment (towards left).')
        elif self.step == 5:
            # 6. Son düz ilerleme (0.5 saniye)
            self.publish_steering_position(0.0, 0.0) # Düz
            self.publish_wheel_velocity(target_wheel_velocity * 0.2, target_wheel_velocity * 0.2) # Çok yavaş hareket
            if elapsed > 0.5: # Bu süreyi denemelerle ayarlayın
                self.step += 1
                self.phase_start_time = now
                self.get_logger().info(f'L Park Step {self.step}: Final forward move.')
        # --- YENİ EKLENEN ADIMLAR BURADAN BAŞLIYOR ---
        elif self.step == 6:
            # 7. AŞAMA – Sola dönerek yaklaş (Arka tekerleklerin hizası park çizgilerine yaklaşmalı)
            # Açı: Yaklaşık -35° ila -40° (saat yönünün tersine, yani sağa doğru)
            # Mesafe: Yaklaşık 1.2 - 1.5 metre
            # Yeni eklediğiniz ters park mantığına göre, bu adımda direksiyonu sağa kırmalıyız.
            steering_angle = -math.radians(37.5) # Yaklaşık -35 ile -40 arası orta değer
            # 1.2m mesafe için gerekli süre (hız 0.4 m/s ise 1.2 / 0.4 = 3 saniye)
            duration_for_distance = 1.3 / self.base_linear_speed # Ortalama mesafe / hız
            
            self.publish_steering_position(steering_angle, steering_angle)
            self.publish_wheel_velocity(target_wheel_velocity * 0.6, target_wheel_velocity * 0.6) # Daha yavaş
            if elapsed > duration_for_distance:
                self.step += 1
                self.phase_start_time = now
                self.get_logger().info(f'L Park Step {self.step}: Approaching with right turn (for reverse park).')
        elif self.step == 7:
            steering_angle = math.radians(0.42) # Yaklaşık +35 ile +40 arası orta değer
            duration_for_distance = 0.8 / (self.base_linear_speed * 0.5) # 1m mesafe, daha yavaş hızla
            
            self.publish_steering_position(steering_angle, steering_angle)
            self.publish_wheel_velocity(target_wheel_velocity * 0.5, target_wheel_velocity * 0.5) # Yavaş
            if elapsed > duration_for_distance:
                self.step += 1
                self.phase_start_time = now
                self.get_logger().info(f'L Park Step {self.step}: Straightening with left turn (for reverse park).')
        elif self.step == 8:
            steering_angle = 0.0 # Düz
            duration_for_distance = 0.05 / (self.base_linear_speed * 0.4) # 0.7m mesafe, daha da yavaş hızla

            self.publish_steering_position(steering_angle, steering_angle)
            self.publish_wheel_velocity(target_wheel_velocity * 0.4, target_wheel_velocity * 0.4) # Çok yavaş
            if elapsed > duration_for_distance:
                self.step += 1
                self.phase_start_time = now
                self.get_logger().info(f'L Park Step {self.step}: Final straight entry and centering (for reverse park).')
        # --- YENİ EKLENEN ADIMLAR BURADA BİTİYOR ---
        else:
            # Durdur ve sekansı tamamla
            self.stop_robot()
            self.get_logger().info('L park tamamlandı ve araç çizgilere paralel!')
            
            # Timer'ı durdur ve yok et
            if self.timer is not None:
                self.destroy_timer(self.timer)
                self.timer = None
            
            self.park_sequence_active = False
            # has_executed_park_sequence'ı True bırakıyoruz ki aynı park işareti tekrar tetiklemesin.

            # L-Park algoritmasının tamamlandığını RobotStateController'a bildir
            completion_msg = Bool()
            completion_msg.data = True
            self.l_park_completion_publisher.publish(completion_msg)
            self.get_logger().info("L-Park algoritması tamamlandı sinyali yayınlandı.")


def main(args=None):
    rclpy.init(args=args)
    node = LParkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()