import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # Robotun hız komutları için (artık doğrudan yayınlamayacağız, ama hala Twist objesiyle çalışacağız)
from nav_msgs.msg import Odometry # Robotun pozisyon bilgisi için (isteğe bağlı)
import time # sleep için
from std_msgs.msg import Bool # Durum bilgisi göndermek için (isteğe bağlı)
from robotaksi_interfaces.msg import TrafficSignDetection # Kendi özel mesajınız
from robotaksi_interfaces.msg import RobotState # YENİ: RobotState mesajını içe aktar

class StopSignAlgorithm(Node):
    def __init__(self):
        super().__init__('stop_sign_algorithm')

        # --- Parametre Bildirimleri ---
        # self.declare_parameter('cmd_vel_topic', '/cmd_vel') # BU SATIRI ARTIK KULLANMAYACAĞIZ, ÇÜNKÜ DOĞRUDAN HIZ YAYINLAMIYORUZ
        self.declare_parameter('detection_msg_topic', '/robotaksi/traffic_sign_detections') # Abone olacağımız topic
        self.declare_parameter('stop_duration', 5.0) # Saniye cinsinden durma süresi
        self.declare_parameter('stop_distance_threshold', 1.0) # Yakınlık mesafesi (örnek)

        # --- Parametre Değerlerini Alın ---
        # self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value # BU SATIRI DA KULLANMAYACAĞIZ
        self.detection_msg_topic = self.get_parameter('detection_msg_topic').get_parameter_value().string_value
        self.stop_duration = self.get_parameter('stop_duration').get_parameter_value().double_value
        self.stop_distance_threshold = self.get_parameter('stop_distance_threshold').get_parameter_value().double_value

        # --- Yayınlayıcılar (Publishers) ---
        # self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10) # BU YAYINLAYICI ARTIK KALDIRILIYOR

        # YENİ: Robotun durumunu değiştirmek için bir yayınlayıcı
        self.state_command_publisher = self.create_publisher(RobotState, '/robotaksi/robot_state_command', 10)
        
        # --- Aboneler (Subscribers) ---
        # TrafficSignDetection mesajlarına abone ol
        self.detection_subscriber_ = self.create_subscription(
            TrafficSignDetection,
            self.detection_msg_topic,
            self.traffic_sign_callback, # Mesaj geldiğinde çağrılacak fonksiyon
            10
        )
        self.get_logger().info(f'Trafik Levhası Algılama topicine abone olundu: {self.detection_msg_topic}')

        # --- Durum Değişkenleri ---
        # Robotun şu anda durma dizisinde olup olmadığını gösteren bayrak
        self.is_stopping_sequence_active = False
        self.stop_timer = None # Zamanlayıcı referansını başlangıçta None olarak ayarlayın

        self.get_logger().info('StopSignAlgorithm düğümü başlatıldı.')

    # --- Mesaj Geri Çağırma Fonksiyonu (Callback) ---
    def traffic_sign_callback(self, msg: TrafficSignDetection):
        """
        TrafficSignDetection mesajları geldiğinde çağrılan fonksiyon.
        Yaya geçidi algılandığında durma dizisini başlatır.
        """
        self.get_logger().info(f"TrafficSignDetection mesajı alındı: Label='{msg.label}', Confidence={msg.confidence:.2f}, IsPedestrianCrossing={msg.is_pedestrian_crossing}, Distance={msg.distance:.2f}m")

        # Eğer yaya geçidi algılandıysa (is_pedestrian_crossing True ise)
        # VE mesafe belirli bir eşiğin altındaysa (örneğin 10m, bu parametre olabilir)
        # VE durma dizisi şu an aktif değilse:
        # Not: StopSignAlgorithm'da bu eşiği yeniden tanımlamak yerine TrafficSignDetector'daki trigger_distance'ı kullanabilirsiniz.
        # Basitlik için şimdilik kendi stop_distance_threshold parametrenizi kullanmaya devam edelim.
        if msg.is_pedestrian_crossing and msg.distance <= self.stop_distance_threshold and not self.is_stopping_sequence_active:
            self.get_logger().info(f"Yaya Geçidi Algılandı ve Yakın! Güven: {msg.confidence:.2f}, Mesafe: {msg.distance:.2f}m. Durma dizisi başlatılıyor...")
            
            # Durma dizisini başlat
            self.is_stopping_sequence_active = True # Dizinin aktif olduğunu işaretle

            # YENİ: Durum yöneticisine durma komutu gönder
            state_msg = RobotState()
            state_msg.state = "STOPPING_AT_PEDESTRIAN_CROSSING"
            self.state_command_publisher.publish(state_msg)
            self.get_logger().info(f"Durum Yöneticisine '{state_msg.state}' komutu gönderildi.")
            
            self.get_logger().info(f"Robot {self.stop_duration} saniye boyunca durduruluyor...")
            
            # Robotu durdurmak için hız yayınlamasına gerek kalmadı, durum yöneticisi bunu yapacak.
            # self.publish_cmd_vel(0.0, 0.0) 

            # Önceki bir zamanlayıcı varsa iptal edin ve yok edin (güvenlik için)
            if self.stop_timer is not None:
                self.stop_timer.cancel()
                self.destroy_timer(self.stop_timer)
                self.stop_timer = None

            # Belirli bir süre durmak için tek seferlik yeni bir zamanlayıcı oluştur
            self.stop_timer = self.create_timer(self.stop_duration, self.finish_stop_sequence)
            
        elif msg.is_pedestrian_crossing and self.is_stopping_sequence_active:
            # Yaya geçidi algılandı ama zaten durma dizisi aktif.
            self.get_logger().info(f"Yaya Geçidi hala algılanıyor ({msg.distance:.2f}m). Robot durma dizisini sürdürüyor.")
        elif msg.is_pedestrian_crossing and msg.distance > self.stop_distance_threshold:
            # Yaya geçidi algılandı ama henüz tetikleme mesafesinde değil.
            self.get_logger().info(f"Yaya Geçidi algılandı ({msg.distance:.2f}m), ancak tetikleme mesafesinden ({self.stop_distance_threshold:.2f}m) daha uzakta.")
        else:
            # Diğer trafik işaretleri veya yaya geçidi algılanmadıysa (is_pedestrian_crossing false ise)
            self.get_logger().info(f"Diğer trafik işareti algılandı veya yaya geçidi algılanmadı: {msg.label}")


    # BU FONKSİYON ARTIK KULLANILMIYOR ÇÜNKÜ HIZ KOMUTLARINI DOĞRUDAN YAYINLAMIYORUZ
    # def publish_cmd_vel(self, linear_x, angular_z):
    #     """Robotun hız komutlarını yayınlar."""
    #     twist_msg = Twist()
    #     twist_msg.linear.x = float(linear_x)
    #     twist_msg.angular.z = float(angular_z)
    #     self.cmd_vel_publisher.publish(twist_msg)

    def finish_stop_sequence(self):
        """Durma süresi sona erdiğinde çağrılır."""
        self.get_logger().info(f"{self.stop_duration} saniyelik durma süresi tamamlandı. Robot hareket etmeye devam edebilir.")
        self.is_stopping_sequence_active = False # Diziyi pasif hale getir
        
        # YENİ: Durum yöneticisine şerit takibi moduna geri dönme komutu gönder
        state_msg = RobotState()
        state_msg.state = "LANE_FOLLOWING"
        self.state_command_publisher.publish(state_msg)
        self.get_logger().info(f"Durum Yöneticisine '{state_msg.state}' komutu gönderildi.")

        # Robotu tekrar hareket ettirme veya bir sonraki adıma geçme mantığı buraya gelir
        # Ancak bunu artık durum yöneticisi yapacak.
        # Örneğin, 0.5 m/s ileri gitmeye devam et:
        # self.publish_cmd_vel(0.5, 0.0) 

        # Zamanlayıcıyı yok et
        if self.stop_timer is not None:
            self.destroy_timer(self.stop_timer)
            self.stop_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = StopSignAlgorithm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()