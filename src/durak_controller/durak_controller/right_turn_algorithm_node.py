import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool # Detector node'dan gelen sağa dön algılama ve tamamlanma sinyali için

import time
import math

class RightTurnAlgorithm(Node):

    def __init__(self):
        super().__init__('right_turn_algorithm_node')

        # Publisher'ları kontrolörlere göre ayarla
        self.position_publisher_ = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.velocity_publisher_ = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # RobotStateController'a sağa dön algoritmasının tamamlandığını bildirmek için publisher
        # '/right_turn_algorithm_completed' topic'ine Bool mesajı yayınlayacağız
        self.turn_algorithm_completion_publisher = self.create_publisher(Bool, '/right_turn_algorithm_completed', 10)

        # --- SAĞA DÖN ALGILAMA SUBSCRIBER'I ---
        # detector_node'un sağa dön algıladığında yayınladığı topic'i dinle.
        self.right_turn_subscription_ = self.create_subscription(
            Bool,
            '/right_turn_detected_signal',  # TrafficSignDetector'daki topic adı
            self.right_turn_callback, # Mesaj geldiğinde çağrılacak fonksiyon
            10
        )
        self.get_logger().info('Right Turn Algorithm Node Started with separate controllers and subscribed to /right_turn_detected_signal.')

        # Robotun hareket parametreleri
        self.declare_parameter('base_linear_speed', 0.5) # m/s
        self.declare_parameter('wheel_radius', 0.1)     # Metre cinsinden tekerlek yarıçapı
        self.declare_parameter('ramp_steps', 20)        # Rampalama için adım sayısı
        self.declare_parameter('ramp_interval', 0.05)   # Saniye cinsinden her adım arasındaki bekleme süresi

        # *** AYARLANABİLİR DÖNÜŞ PARAMETRELERİ ***
        # Daha keskin dönüş için ana direksiyon açısını daha negatif yapın.
        self.declare_parameter('turn_angle_degrees', -270.0) # Sağa dönüş için direksiyonun ana açısı (negatif sağa)
        
        # Ackermann benzeri direksiyon için ayarlama faktörleri.
        # Bu değerler, fiziksel robot modelinize ve istenen dönüş yumuşaklığına göre ayarlanmalıdır.
        # Sağ dönüş için (turn_angle_degrees negatif):
        # Sol tekerlek (dış): |turn_angle_degrees| * steering_outer_factor
        # Sağ tekerlek (iç): |turn_angle_degrees| * steering_inner_factor
        # Genellikle steering_inner_factor > 1.0 ve steering_outer_factor < 1.0 olmalıdır.
        self.declare_parameter('steering_inner_factor', 1.15)  # İç tekerlek için çarpan (sağ dönüşte sağ tekerlek)
        self.declare_parameter('steering_outer_factor', 0.85)  # Dış tekerlek için çarpan (sağ dönüşte sol tekerlek)

        # Dönüş sırasında katedilecek lineer mesafeyi azaltarak dönüşü daha kompakt hale getirin.
        self.declare_parameter('turn_forward_distance', 5.0) # Dönüş sırasında ilerlenecek mesafe

        # Algoritma başladığında direkt dönüşe geçmek için bu değeri sıfıra yakın yapın.
        self.declare_parameter('pre_turn_forward_distance', 6.5) # Dönüşten önce ilerlenecek mesafe (çok az)
        
        self.declare_parameter('post_turn_forward_distance', 1.5) # Dönüşten sonra ilerlenecek mesafe
        self.declare_parameter('turn_ramp_duration_s', 0.8) # Direksiyon dönüş rampası süresi (daha hızlı dönüş için azaltılabilir)
        # *** PARAMETRE AYARLAMALARI SONU ***


        self.base_linear_speed = self.get_parameter('base_linear_speed').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.ramp_steps = self.get_parameter('ramp_steps').get_parameter_value().integer_value
        self.ramp_interval = self.get_parameter('ramp_interval').get_parameter_value().double_value
        self.turn_angle_degrees = self.get_parameter('turn_angle_degrees').get_parameter_value().double_value
        
        self.steering_inner_factor = self.get_parameter('steering_inner_factor').get_parameter_value().double_value
        self.steering_outer_factor = self.get_parameter('steering_outer_factor').get_parameter_value().double_value

        self.turn_forward_distance = self.get_parameter('turn_forward_distance').get_parameter_value().double_value
        self.pre_turn_forward_distance = self.get_parameter('pre_turn_forward_distance').get_parameter_value().double_value
        self.post_turn_forward_distance = self.get_parameter('post_turn_forward_distance').get_parameter_value().double_value
        self.turn_ramp_duration_s = self.get_parameter('turn_ramp_duration_s').get_parameter_value().double_value


        # Dönüş sekansının zaten çalışıp çalışmadığını kontrol etmek için bayraklar
        self.turn_sequence_active = False # Dizinin şu an çalışıp çalışmadığını gösterir
        self.has_executed_right_turn_sequence = False # Sağ dönüş sekansını bir kez çalıştırdığımızı işaretlemek için

    def right_turn_callback(self, msg: Bool):
        """
        Detector node'dan gelen sağa dön algılama mesajlarını işler.
        'True' mesajı alındığında ve sekans aktif değilse, sağa dön sekansını başlatır.
        """
        # Sadece sağa dön algılandığında (msg.data == True) ve sekans aktif değilse başla
        if msg.data and not self.turn_sequence_active:
            # Sekansın daha önce çalışıp çalışmadığını kontrol et
            if not self.has_executed_right_turn_sequence:
                self.get_logger().info('Sağa Dön tabelası algılandı! Dönüş sekansı başlatılıyor...')
                self.turn_sequence_active = True        # Sekansın aktif olduğunu işaretle
                self.has_executed_right_turn_sequence = True # Sekansı bir kez tetiklediğimizi işaretle

                # Sağa dönüş sekansını başlat
                self.execute_right_turn_sequence()

                # Sekans tamamlandığında bayrakları sıfırla
                self.turn_sequence_active = False # Sekans bitti

                # RobotStateController'a algoritmanın tamamlandığını bildir (std_msgs/Bool kullanıyoruz)
                completion_msg = Bool()
                completion_msg.data = True # Algoritma tamamlandı
                self.turn_algorithm_completion_publisher.publish(completion_msg)
                self.get_logger().info("Right turn algorithm completion signal (True) sent.")
                self.get_logger().info("Sağa dönüş sekansı tamamlandı. Robot şerit takibine dönmeli.")

            else:
                self.get_logger().info('Sağa Dön tabelası algılandı, ancak sekans zaten bir kez çalıştırıldı. Yok sayılıyor.')
        elif msg.data and self.turn_sequence_active:
            self.get_logger().info('Sağa Dön tabelası algılandı, ancak sekans zaten aktif. Şimdilik yok sayılıyor.')
        # msg.data == False durumu, TrafficSignDetector'da bayrağın sıfırlanmasıyla ilgilidir.
        # Burada özel olarak ele almaya gerek yok, çünkü biz sadece True sinyaliyle ilgileniyoruz.


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

    def turn_and_move(self, target_angle_degrees, turn_distance_meters, ramp_duration_s):
        """
        Direksiyonu belirtilen açıya döndürür ve bu sırada belirli bir mesafe ileri hareket eder.
        Sağ için negatif açı, sol için pozitif açı.
        """
        self.get_logger().info(f'Turning steering to {target_angle_degrees} degrees and moving {turn_distance_meters} meters over {ramp_duration_s}s.')

        # Sol ve sağ tekerlek açılarını hesapla
        # target_angle_degrees bir dönüşün ana açısıdır.
        # Sağa dönüş için (target_angle_degrees negatif):
        # Sağ tekerlek (iç) daha büyük bir negatif açıya, sol tekerlek (dış) daha küçük bir negatif açıya sahip olmalı.
        if target_angle_degrees < 0: # Sağa dönüş
            target_right_angle_radians = math.radians(target_angle_degrees * self.steering_inner_factor)
            target_left_angle_radians = math.radians(target_angle_degrees * self.steering_outer_factor)
        elif target_angle_degrees > 0: # Sola dönüş
            target_left_angle_radians = math.radians(target_angle_degrees * self.steering_inner_factor)
            target_right_angle_radians = math.radians(target_angle_degrees * self.steering_outer_factor)
        else: # Düz gitme
            target_left_angle_radians = 0.0
            target_right_angle_radians = 0.0


        # Direksiyonu hedeflenen açıya yumuşakça getir
        self._ramp_steering_angle(target_left_angle_radians, target_right_angle_radians, ramp_duration_s)

        # Tekerlekleri döndürerek ileri hareket et
        target_wheel_velocity = self.base_linear_speed / self.wheel_radius
        duration_for_turn = 0.0
        if self.base_linear_speed != 0:
            duration_for_turn = turn_distance_meters / self.base_linear_speed

        # Direksiyon hedeflenen açıda tutulurken tekerlekleri döndür.
        self._ramp_wheel_velocity(target_wheel_velocity, duration_for_turn, initial_ramp_done=False)

        self.stop_robot() # Hareket bitince durdur


    def _ramp_steering_angle(self, target_left_angle, target_right_angle, ramp_duration_s):
        """Direksiyon eklemlerini belirli bir süre içinde kademeli olarak hedeflenen açıya getirir."""
        start_left_angle = 0.0 
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

        # Hızı artırma rampası
        for i in range(num_steps + 1):
            factor = i / num_steps if num_steps > 0 else 1.0
            current_velocity = start_velocity + (target_velocity - start_velocity) * factor
            self.publish_wheel_velocity(current_velocity, current_velocity)
            time.sleep(self.ramp_interval)

        # Hedef hıza ulaştıktan sonra durma rampası
        self.publish_wheel_velocity(target_velocity, target_velocity) # Hızı hedefte tut

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

    def execute_right_turn_sequence(self):
        """Sağa dön tabelası görüldükten sonraki 90 derecelik dönüş dizisini çalıştırır."""
        self.get_logger().info('Executing 90-degree right turn sequence...')

        # 1. Dönüşten önce çok az ileri git (veya hiç gitme)
        # pre_turn_forward_distance değerini çok düşük (örn: 0.05) veya 0.0 yaparak direkt dönüşe başla
        self.move_forward(self.pre_turn_forward_distance)
        self.stop_robot() # Kısa bir duraklama, ani dönüşler için bazen yararlı olabilir
        self.get_logger().info(f'Moved {self.pre_turn_forward_distance}m forward before turn to initiate.')

        # 2. Direksiyonu sağa çevir ve dönüş sırasında ileri git
        # turn_angle_degrees daha negatif, turn_forward_distance daha kısa olmalı
        self.turn_and_move(self.turn_angle_degrees, self.turn_forward_distance, self.turn_ramp_duration_s)
        self.stop_robot()
        self.get_logger().info(f'Completed turn with steering {self.turn_angle_degrees} deg and moved {self.turn_forward_distance}m.')

        # 3. Direksiyonu düzelt
        self.publish_steering_position(0.0, 0.0)
        time.sleep(0.5) # Direksiyonun düzelmesi için bekleme
        self.get_logger().info('Steering straightened.')

        # 4. Dönüşü tamamlamak ve yeni yola hizalanmak için biraz daha ileri git
        self.move_forward(self.post_turn_forward_distance) 
        self.stop_robot()
        self.get_logger().info(f'Moved {self.post_turn_forward_distance}m forward after turn.')

        self.get_logger().info('90-degree right turn sequence completed.')
        self.stop_robot() # Sekans sonunda robotun durduğundan emin ol


def main(args=None):
    rclpy.init(args=args)
    node = RightTurnAlgorithm()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()