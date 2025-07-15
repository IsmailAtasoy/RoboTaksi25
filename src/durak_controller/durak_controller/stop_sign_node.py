import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool # Detector node'dan gelen durak algılama için
# from robotaksi_interfaces.msg import RobotState # BU İMPORTU KALDIRIYORUZ!

import time
import math

class StopSignAlgorithm(Node):

    def __init__(self):
        super().__init__('stop_sign_algorithm')

        # Publisher'ları kontrolörlere göre ayarla
        self.position_publisher_ = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.velocity_publisher_ = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # RobotStateController'a stop sign algoritmasının tamamlandığını bildirmek için publisher
        # '/stop_sign_algorithm_completed' topic'ine Bool mesajı yayınlayacağız
        self.stop_algorithm_completion_publisher = self.create_publisher(Bool, '/stop_sign_algorithm_completed', 10)


        # --- DURAK ALGILAMA SUBSCRIBER'I ---
        # detector_node'un durak algıladığında yayınladığı topic'i dinle.
        self.bus_stop_subscription_ = self.create_subscription(
            Bool,
            '/bus_stop_detected_signal',  # TrafficSignDetector'daki yeni topic adı
            self.bus_stop_callback, # Mesaj geldiğinde çağrılacak fonksiyon
            10
        )
        self.get_logger().info('Stop Sign Algorithm Node Started with separate controllers and subscribed to /bus_stop_detected_signal.')

        # Robotun hareket parametreleri
        self.base_linear_speed = 0.5  # m/s (İleri hız)
        self.base_angular_position = math.radians(15) # rad (Direksiyonun maksimum açısı, yumuşak dönüş için)
        self.wheel_radius = 0.1 # Metre cinsinden tekerlek yarıçapı. **KENDİ ROBOT MODELİNİZE GÖRE AYARLAYIN!**
                                # Yanlışsa, tekerlek hızı yanlış hesaplanır.

        # Rampalama için sabitler
        self.ramp_steps = 20  # Rampalama için adım sayısı (daha fazla adım = daha yumuşak geçiş)
        self.ramp_interval = 0.05 # Saniye cinsinden her adım arasındaki bekleme süresi

        # Durak sekansının zaten çalışıp çalışmadığını kontrol etmek için bayraklar
        self.stop_sequence_active = False # Dizinin şu an çalışıp çalışmadığını gösterir
        self.has_executed_bus_stop_sequence = False # Durak sekansını bir kez çalıştırdığımızı işaretlemek için

    def bus_stop_callback(self, msg: Bool):
        """
        Detector node'dan gelen durak algılama mesajlarını işler.
        'True' mesajı alındığında ve sekans aktif değilse, durak sekansını başlatır.
        """
       
        # Sadece durak algılandığında (msg.data == True) ve sekans aktif değilse başla
        if msg.data and not self.stop_sequence_active:
            # Sekansın daha önce çalışıp çalışmadığını kontrol et
            # Bu, tek bir durak işareti için sadece bir kez çalışmasını sağlar.
            if not self.has_executed_bus_stop_sequence:
                self.get_logger().info('Durak tabelası algılandı! Sekans başlatılıyor...')
                self.stop_sequence_active = True        # Sekansın aktif olduğunu işaretle
                self.has_executed_bus_stop_sequence = True # Sekansı bir kez tetiklediğimizi işaretle

                # RobotStateController'a durma algoritmasının başladığını bildirmek için bir sinyal GÖNDERMİYORUZ.
                # RobotStateController, 'bus_stop_detected_signal' True olduğunda kendi durumunu değiştiriyor.
                # Biz sadece stop sign algoritmasını tamamladığımızda sinyal göndereceğiz.
                # Aksi takdirde, Race condition (yarış durumu) oluşabilir.

                # Durak tabelası sekansını başlat
                self.execute_stop_sign_sequence()

                # Sekans tamamlandığında bayrakları sıfırla ve robotun şerit takibine dönmesini sağla
                self.stop_sequence_active = False # Sekans bitti
                # has_executed_bus_stop_sequence'ı sıfırlamayız, bu durak işareti için bitti demek.
                # Eğer aynı durak işaretini tekrar geçerken tetiklenmesini istemiyorsak böyle kalmalı.
                # Eğer bir sonraki durak işareti için tekrar tetiklenmeli ise, bu mantık daha karmaşık olur.

                # RobotStateController'a algoritmanın tamamlandığını bildir (std_msgs/Bool kullanıyoruz)
                completion_msg = Bool()
                completion_msg.data = True # Algoritma tamamlandı
                self.stop_algorithm_completion_publisher.publish(completion_msg)
                self.get_logger().info("Stop sign algorithm completion signal (True) sent to RobotStateController.")
                self.get_logger().info("Durak sekansı tamamlandı. Robot şerit takibine dönmeli.")

            else:
                self.get_logger().info('Durak tabelası algılandı, ancak sekans zaten bir kez çalıştırıldı. Yok sayılıyor.')
        elif msg.data and self.stop_sequence_active:
            self.get_logger().info('Durak tabelası algılandı, ancak sekans zaten aktif. Şimdilik yok sayılıyor.')
        # msg.data == False durumunu burada özel olarak ele almıyoruz, çünkü bu sadece sinyalin yok olduğunu gösterir.
        # Bizim için önemli olan True sinyali ve sekansın başlatılması.

    # publish_robot_state fonksiyonu KALDIRILIYOR, çünkü artık StopSignAlgorithm bu görevi yapmıyor.
    # def publish_robot_state(self, state: str):
    #     """RobotStateController'a robotun durumunu yayınlar."""
    #     msg = RobotState()
    #     msg.state = state
    #     self.robot_state_publisher.publish(msg)
    #     self.get_logger().info(f"Robot Durumu Yayınlandı: {state}")


    def publish_steering_position(self, left_angle, right_angle):
        """Direksiyon eklemlerinin pozisyonunu yayınlar."""
        msg = Float64MultiArray()
        msg.data = [float(left_angle), float(right_angle)]
        self.position_publisher_.publish(msg)
        self.get_logger().info(f'Publishing Steering Positions: Left={math.degrees(left_angle):.2f} deg, Right={math.degrees(right_angle):.2f} deg')

    def publish_wheel_velocity(self, left_velocity, right_velocity):
        """Arka tekerleklerin açısal hızını (rad/s) yayınlar."""
        msg = Float64MultiArray()
        msg.data = [float(left_velocity), float(right_velocity)]
        self.velocity_publisher_.publish(msg)
        self.get_logger().info(f'Publishing Wheel Velocities: Left={left_velocity:.2f} rad/s, Right={right_velocity:.2f} rad/s')

    def move_forward(self, distance_meters):
        """Belirtilen mesafe kadar ileri gider."""
        self.get_logger().info(f'Moving forward {distance_meters} meters...')

        # Direksiyonu düz tut
        self.publish_steering_position(0.0, 0.0)
        time.sleep(0.1) # Direksiyonun düzelmesi için kısa bekleme

        # Lineer hızı açısal tekerlek hızına çevir: v = w * r => w = v / r
        target_wheel_velocity = self.base_linear_speed / self.wheel_radius
        duration = distance_meters / self.base_linear_speed

        self._ramp_wheel_velocity(target_wheel_velocity, duration)
        self.stop_robot()

    def turn(self, target_angle_degrees, turn_distance_meters=0.0, ramp_duration_s=1.0):
        """
        Belirtilen açıya direksiyonu döndürür ve bu sırada ileri hareket eder.
        Sağ için negatif açı, sol için pozitif açı.
        turn_distance_meters: Dönüş sırasında katedilecek yaklaşık lineer mesafe.
        ramp_duration_s: Direksiyonun hedeflenen açıya ulaşması için geçen süre.
        """
        self.get_logger().info(f'Turning to {target_angle_degrees} degrees while moving {turn_distance_meters} meters with ramp_duration={ramp_duration_s}s...')

        target_angle_radians = math.radians(target_angle_degrees)

        # Direksiyonu hedeflenen açıya yumuşakça getir
        self._ramp_steering_angle(target_angle_radians, target_angle_radians, ramp_duration_s)

        # Tekerlekleri döndürerek ileri hareket et
        target_wheel_velocity = self.base_linear_speed / self.wheel_radius
        duration_for_turn = 0.0
        if self.base_linear_speed != 0:
            duration_for_turn = turn_distance_meters / self.base_linear_speed

        # Direksiyon hedeflenen açıda tutulurken tekerlekleri döndür.
        self._ramp_wheel_velocity(target_wheel_velocity, duration_for_turn, initial_ramp_done=True)

        self.stop_robot() # Dönüş bitince durdur

    def _ramp_steering_angle(self, target_left_angle, target_right_angle, ramp_duration_s):
        """Direksiyon eklemlerini belirli bir süre içinde kademeli olarak hedeflenen açıya getirir."""
        start_left_angle = 0.0 # Varsayılan olarak düz pozisyondan başla
        start_right_angle = 0.0

        num_steps = int(ramp_duration_s / self.ramp_interval)
        if num_steps == 0: num_steps = 1 # En az bir adım

        for i in range(num_steps + 1):
            factor = i / num_steps
            current_left_angle = start_left_angle + (target_left_angle - start_left_angle) * factor
            current_right_angle = start_right_angle + (target_right_angle - start_right_angle) * factor
            self.publish_steering_position(current_left_angle, current_right_angle)
            time.sleep(self.ramp_interval)
        # Son pozisyonda kalması için bir kez daha yayınla
        self.publish_steering_position(target_left_angle, target_right_angle)

    def _ramp_wheel_velocity(self, target_velocity, total_duration_s, initial_ramp_done=False):
        """Tekerleklerin hızını belirli bir süre içinde kademeli olarak artırır ve sonra yavaşça durdurur."""
        start_velocity = 0.0 # Rampanın her zaman 0'dan başlamasını sağlar.
        
        num_steps = int(total_duration_s / self.ramp_interval)
        if num_steps == 0: num_steps = 1

        # Eğer initial_ramp_done true ise, sadece durma rampasını uygula
        if initial_ramp_done:
            # Hızı azaltma rampası (sadece duruş)
            for i in range(num_steps + 1): # Tam sürede yavaşlama
                factor = i / num_steps if num_steps > 0 else 1.0
                current_velocity = target_velocity - (target_velocity - 0.0) * factor
                self.publish_wheel_velocity(current_velocity, current_velocity)
                time.sleep(self.ramp_interval)
        else:
            # Hızı artırma rampası ve sonra yavaşça durdurma
            for i in range(num_steps + 1):
                factor = i / num_steps if num_steps > 0 else 1.0
                current_velocity = start_velocity + (target_velocity - start_velocity) * factor
                self.publish_wheel_velocity(current_velocity, current_velocity)
                time.sleep(self.ramp_interval)

        self.publish_wheel_velocity(0.0, 0.0) # Son duruş


    def stop_robot(self):
        """Robotu durdurur."""
        self.get_logger().info('Stopping robot.')
        self.publish_steering_position(0.0, 0.0) # Direksiyonu sıfırla
        self.publish_wheel_velocity(0.0, 0.0) # Tekerlek hızlarını sıfırla
        time.sleep(0.5) # Durma komutlarının işlenmesi için yeterli bekleme

    def wait_for_seconds(self, seconds):
        """Belirtilen saniye kadar bekler."""
        self.get_logger().info(f'Waiting for {seconds} seconds...')
        self.stop_robot() # Beklerken robotun durduğundan emin ol
        time.sleep(seconds)
        self.get_logger().info('Wait finished.')

    def execute_stop_sign_sequence(self):
        """Durak tabelası görüldükten sonraki hareket dizisini çalıştırır."""
        self.get_logger().info('Executing stop sign sequence...')

        # 1. 5.0 metre ilerle
        self.move_forward(3.5)

        self.turn(-15.0, turn_distance_meters=3.5, ramp_duration_s=0.7) 

        self.move_forward(2.8)

        self.turn(25.0, turn_distance_meters=3.0, ramp_duration_s=0.7)

        #self.move_forward(0.5)

        self.wait_for_seconds(3)

        self.turn(30.0, turn_distance_meters=4.5, ramp_duration_s=0.7)
        
        self.move_forward(1.6)

        self.turn(-20.0, turn_distance_meters=3.0, ramp_duration_s=0.7)


        self.move_forward(1.0)

        # 10. Devam et
        self.get_logger().info('Stop sign sequence completed and robot stopped.')
        self.stop_robot() # Sekans sonunda robotun durduğundan emin ol


def main(args=None):
    rclpy.init(args=args)
    node = StopSignAlgorithm()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()