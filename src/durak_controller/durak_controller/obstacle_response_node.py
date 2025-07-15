import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool # TrafficSignDetector'dan gelen engel sinyali için
from std_msgs.msg import Float64MultiArray # Kontrolcülere komut göndermek için
import time
import math

class ObstacleResponseNode(Node):
    def __init__(self):
        super().__init__('obstacle_response_node')

        self.get_logger().info("Obstacle Response Node Başlatılıyor...")

        # --- PARAMETRELERİN TANIMLANMASI VE ALINMASI ---
        self.declare_parameter('position_commands_topic', '/position_controller/commands')
        self.declare_parameter('velocity_commands_topic', '/velocity_controller/commands')

        # İlk koddaki 'step_duration' ve 'front_velocity' değerlerinden türetilen yeni parametreler
        self.declare_parameter('avoidance_linear_speed', 0.5) # Kaçınma sırasında kullanılacak hız (m/s)
        self.declare_parameter('wheel_radius', 0.3)           # Tekerlek yarıçapı (metre) - Robot modelinize göre ayarlayın!

        # İlk koddaki hareket süreleri ve açıları baz alınarak yeni kaçınma parametreleri
        self.declare_parameter('avoid_left_duration_s', 6.5)   # Sola hareket süresi tekrar 6.0 saniyeye çıkarıldı
        self.declare_parameter('avoid_straight_duration_s', 4.0) # Düz hareket süresi (step_duration - 1)
        self.declare_parameter('avoid_right_duration_s', 8.0) # Sağa hareket süresi (step_duration + 12)
        self.declare_parameter('avoid_slight_left_duration_s', 3.0) # Hafif sola hareket süresi (step_duration)

        self.declare_parameter('steer_angle_full_rad', 1.5) # İlk koddaki 'left_angle' (radyan)
        self.declare_parameter('steer_angle_slight_right_factor', 0.1) # Düz harekette hafif sağa dönüş faktörü
        self.declare_parameter('steer_angle_slight_left_factor', 0.1)  # Hafif sola harekette faktör
        self.declare_parameter('steer_angle_hard_right_factor', 0.3)  # Sağa harekette faktör (ilk koddaki 0.3)


        self.declare_parameter('turn_ramp_duration_s', 0.75) # Direksiyon dönüş rampası süresi

        # Parametre değerlerini al
        self.position_commands_topic = self.get_parameter('position_commands_topic').get_parameter_value().string_value
        self.velocity_commands_topic = self.get_parameter('velocity_commands_topic').get_parameter_value().string_value
        self.avoidance_linear_speed = self.get_parameter('avoidance_linear_speed').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        self.avoid_left_duration_s = self.get_parameter('avoid_left_duration_s').get_parameter_value().double_value
        self.avoid_straight_duration_s = self.get_parameter('avoid_straight_duration_s').get_parameter_value().double_value
        self.avoid_right_duration_s = self.get_parameter('avoid_right_duration_s').get_parameter_value().double_value
        self.avoid_slight_left_duration_s = self.get_parameter('avoid_slight_left_duration_s').get_parameter_value().double_value

        self.steer_angle_full_rad = self.get_parameter('steer_angle_full_rad').get_parameter_value().double_value
        self.steer_angle_slight_right_factor = self.get_parameter('steer_angle_slight_right_factor').get_parameter_value().double_value
        self.steer_angle_slight_left_factor = self.get_parameter('steer_angle_slight_left_factor').get_parameter_value().double_value
        self.steer_angle_hard_right_factor = self.get_parameter('steer_angle_hard_right_factor').get_parameter_value().double_value
        
        self.turn_ramp_duration_s = self.get_parameter('turn_ramp_duration_s').get_parameter_value().double_value


        # --- YAYINCILAR (PUBLISHERS) ---
        self.position_publisher_ = self.create_publisher(Float64MultiArray, self.position_commands_topic, 10)
        self.velocity_publisher_ = self.create_publisher(Float64MultiArray, self.velocity_commands_topic, 10)
        self.get_logger().info(f"Yayıncılar oluşturuldu: '{self.position_commands_topic}', '{self.velocity_commands_topic}'")

        # --- ABONELER (SUBSCRIBERS) ---
        self.straight_obstacle_subscription_ = self.create_subscription(
            Bool,
            '/straight_obstacle_detected_signal', # TrafficSignDetector'daki topic adı
            self.obstacle_detected_callback,
            10
        )
        # RobotStateController'a tamamlanma sinyali göndermek için yeni publisher
        self.obstacle_response_completed_publisher = self.create_publisher(
            Bool,
            '/obstacle_avoidance_completed', # RobotStateController'dakiyle aynı olmalı
            10
        )
        self.get_logger().info('Engel Yanıt Düğümü /straight_obstacle_detected_signal konusuna abone oldu.')
        self.get_logger().info('Engel tepkisi tamamlanma sinyali /obstacle_avoidance_completed konusuna yayımlanacak.')

        # Engel tepkisinin aktif olup olmadığını takip eden bayrak
        self.is_obstacle_handling_active = False
        self.get_logger().info('Engel Yanıt Düğümü Başlatıldı.')

    # --- YARDIMCI KONTROL FONKSİYONLARI ---
    def publish_steering_position(self, left_angle_rad, right_angle_rad):
        """
        Direksiyon eklemlerinin hedef pozisyonunu (radyan cinsinden) yayınlar.
        """
        msg = Float64MultiArray()
        msg.data = [float(left_angle_rad), float(right_angle_rad)]
        self.position_publisher_.publish(msg)
        # self.get_logger().info(f'Direksiyon Pozisyonları Yayınlanıyor: Sol={math.degrees(left_angle_rad):.2f} derece, Sağ={math.degrees(right_angle_rad):.2f} derece')

    def publish_wheel_velocity(self, left_velocity_rad_s, right_velocity_rad_s):
        """
        Arka (veya tahrik) tekerleklerin açısal hızını (radyan/s) yayınlar.
        """
        msg = Float64MultiArray()
        msg.data = [float(left_velocity_rad_s), float(right_velocity_rad_s)]
        self.velocity_publisher_.publish(msg)
        # self.get_logger().info(f'Tekerlek Hızları Yayınlanıyor: Sol={left_velocity_rad_s:.2f} rad/s, Sağ={right_velocity_rad_s:.2f} rad/s')

    def stop_robot(self):
        """Robotu durdurur ve direksiyonu düzeltir."""
        self.get_logger().info('Robot durduruluyor.')
        self.publish_steering_position(0.0, 0.0) # Direksiyonu sıfırla (düz)
        self.publish_wheel_velocity(0.0, 0.0) # Tekerlek hızlarını sıfırla
        time.sleep(0.5) # Komutların alt seviye kontrolörler tarafından işlenmesi için kısa bekleme
        

    def move_with_steer(self, target_left_angle_rad, target_right_angle_rad, duration_s, speed=None):
        """
        Belirtilen açılarda belirtilen süre boyunca ileri hareket eder.
        """
        if speed is None:
            speed = self.avoidance_linear_speed

        self.get_logger().info(f'Hareket başlatılıyor: Sol Açı={math.degrees(target_left_angle_rad):.2f} derece, Sağ Açı={math.degrees(target_right_angle_rad):.2f} derece, Süre={duration_s:.2f} s, Hız={speed:.2f} m/s')

        target_wheel_velocity = speed / self.wheel_radius
        
        # Direksiyonu hedef açıya yumuşakça getir (ramp_duration_s içinde)
        start_left_angle = 0.0 
        start_right_angle = 0.0

        num_ramp_steps = int(self.turn_ramp_duration_s / 0.05) + 1
        if num_ramp_steps == 0: num_ramp_steps = 1

        for i in range(num_ramp_steps):
            factor = (i + 1) / num_ramp_steps
            current_left_angle = start_left_angle + (target_left_angle_rad - start_left_angle) * factor
            current_right_angle = start_right_angle + (target_right_angle_rad - start_right_angle) * factor
            self.publish_steering_position(current_left_angle, current_right_angle)
            #rclpy.spin_once(self, timeout_sec=0.05) # Küçük bir spin ile mesajların gitmesini sağla
            time.sleep(0.05)
        
        # Ana hareket süresi boyunca açıları ve hızı sabit tut
        # Rampalama süresini ana süreden çıkar
        remaining_duration = duration_s - self.turn_ramp_duration_s
        if remaining_duration < 0: remaining_duration = 0 # Süre negatif olmasın

        start_time = time.time()
        while (time.time() - start_time) < remaining_duration: 
            self.publish_steering_position(target_left_angle_rad, target_right_angle_rad)
            self.publish_wheel_velocity(target_wheel_velocity, target_wheel_velocity)
            #rclpy.spin_once(self, timeout_sec=0.05) # Küçük bir spin ile mesajların gitmesini sağla
            time.sleep(0.05) # Küçük adımlarla yayınla

        self.get_logger().info('Hareket adım tamamlandı.')

    # --- ANA CALLBACK VE MANEVRA MANTIĞI ---

    def obstacle_detected_callback(self, msg: Bool):
        """
        /straight_obstacle_detected_signal topic'inden gelen mesajları işler.
        """
        if msg.data: # Eğer True sinyali gelirse (düz engel algılandı)
            if not self.is_obstacle_handling_active:
                self.get_logger().info('Düz Engel Algılandı! Engelden Kaçınma Algoritması Başlatılıyor...')
                self.is_obstacle_handling_active = True # Manevranın aktif olduğunu işaretle
                # Manevrayı yeni bir iş parçacığında başlatmak, ROS spin döngüsünü bloke etmez.
                # Ancak, burada sadece bir ROS düğümü olduğu varsayılarak doğrudan çağrı yapılıyor.
                # Daha karmaşık sistemlerde Concurrency (Multi-threaded Executor) kullanmak daha iyidir.
                self.execute_obstacle_avoidance_maneuver() 
            else:
                self.get_logger().info('Düz Engel Algılandı, ancak kaçınma manevrası zaten aktif. Tekrar tetiklenmiyor.')
        else: # Eğer False sinyali gelirse (engel artık yok veya uzaklaştı)
            # is_obstacle_handling_active, manevra adımları bittiğinde False olacak.
            # Dedektörden gelen False sinyali, manevra bitmeden gelirse,
            # RobotStateController bu durumu kendi is_obstacle_response_active bayrağıyla yönetecek.
            if self.is_obstacle_handling_active:
                self.get_logger().info('Düz Engel dedektörden kalktı, ancak kaçınma manevrası hala devam ediyor.')
            else:
                self.get_logger().info('Düz Engel dedektörden kalktı ve kaçınma manevrası zaten tamamlanmıştı/aktif değildi.')
                # Bu durumda ek bir işlem yapmaya gerek yok, çünkü RobotStateController tamamlanma sinyalini bekler.


    def execute_obstacle_avoidance_maneuver(self):
        """
        Engelden kaçınma manevrasını (sol şeride geç, engeli geç, sağ şeride dön) uygular.
        """
        self.get_logger().info("Robot durduruluyor ve engelden kaçınma manevrasına başlanıyor...")
        self.stop_robot() # İlk olarak robotu durdur
        # time.sleep(0.5) # Stop komutlarının işlenmesi için kısa bekleme (stop_robot içinde zaten var)

        # *** MANEVRA ADIMLARI (İlk koddaki prensiplere göre) ***

        # 1. Adım: Sola hareket (İlk koddaki "2m Sola hareket" -> Süre: 5s, Açı: -1.5 rad)
        self.get_logger().info(f"Manevra 1/4: Sola hareket ({self.avoid_left_duration_s:.1f}s) başlatılıyor...")
        # Sola dönmek için pozitif açılar (sağ tekerlek biraz daha küçük pozitif açı alabilir)
        # Not: Genellikle pozitif açı sağa, negatif açı sola döner (robot kinematik modeline bağlı)
        # Mevcut kodda -1.5 rad sol olarak yorumlandığından, sol dönüş için pozitif açıları kullanıyoruz.
        # Eğer direksiyon modelinizde pozitif açı sağa, negatif açı sola dönüyorsa, bu değerleri buna göre ayarlayın.
        target_left_angle = self.steer_angle_full_rad # Sol tekerlek içe doğru döner (ön aksta)
        target_right_angle = self.steer_angle_full_rad # Sağ tekerlek dışa doğru döner (ön aksta)
        self.move_with_steer(target_left_angle, target_right_angle, self.avoid_left_duration_s, self.avoidance_linear_speed)
        self.stop_robot()
        time.sleep(0.5)

        # 2. Adım: Düz hareket (hafif sağa yönlendirme) (İlk koddaki "2m Düz hareket (hafif sağa)" -> Süre: 2s, Açı: 0.15 rad)
        self.get_logger().info(f"Manevra 2/4: Düz hareket (hafif sağa) ({self.avoid_straight_duration_s:.1f}s) başlatılıyor...")
        # Hafif sağa dönmek için negatif açılar
        target_left_angle = -self.steer_angle_full_rad * self.steer_angle_slight_right_factor 
        target_right_angle = -self.steer_angle_full_rad * self.steer_angle_slight_right_factor 
        self.move_with_steer(target_left_angle, target_right_angle, self.avoid_straight_duration_s, self.avoidance_linear_speed)
        self.stop_robot()
        time.sleep(0.5)

        # 3. Adım: Sağa hareket (hafif sola yönlendirme) (İlk koddaki "2m Sağa hareket (hafif sola)" -> Süre: 15s, Açı: 0.45 rad)
        self.get_logger().info(f"Manevra 3/4: Sağa hareket ({self.avoid_right_duration_s:.1f}s) başlatılıyor...")
        # Sağa dönmek için negatif açılar
        target_left_angle = -self.steer_angle_full_rad * self.steer_angle_hard_right_factor 
        target_right_angle = -self.steer_angle_full_rad * self.steer_angle_hard_right_factor 
        self.move_with_steer(target_left_angle, target_right_angle, self.avoid_right_duration_s, self.avoidance_linear_speed)
        self.stop_robot()
        time.sleep(0.5)

        # 4. Adım: Hafif Sola Hareket (İlk koddaki "Hafif Sola Hareket" -> Süre: 3s, Açı: -0.15 rad)
        self.get_logger().info(f"Manevra 4/4: Hafif Sola Hareket ({self.avoid_slight_left_duration_s:.1f}s) başlatılıyor...")
        # Hafif sola dönmek için pozitif açılar
        target_left_angle = self.steer_angle_full_rad * self.steer_angle_slight_left_factor 
        target_right_angle = self.steer_angle_full_rad * self.steer_angle_slight_left_factor 
        self.move_with_steer(target_left_angle, target_right_angle, self.avoid_slight_left_duration_s, self.avoidance_linear_speed)
        self.stop_robot()
        time.sleep(0.5)

        self.get_logger().info("Engelden kaçınma manevrası tamamlandı.")
        
        # Manevra bittiğinde tamamlanma sinyalini yayımla
        completed_msg = Bool()
        completed_msg.data = True
        self.obstacle_response_completed_publisher.publish(completed_msg)
        self.get_logger().info("Engel tepkisi bitti sinyali RobotStateController'a yayınlandı.")
        
        # Manevranın artık aktif olmadığını işaretle
        self.is_obstacle_handling_active = False 
        self.get_logger().info("Robotun normal seyrine dönmesi bekleniyor.")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleResponseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Engel Yanıt Düğümü durduruldu (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()