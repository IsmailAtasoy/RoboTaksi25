import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String
import time

# Define states for clarity
class RobotState:
    LANE_FOLLOWING = "lane_following"
    STOP_SIGN_ALGORITHM = "stop_sign_algorithm"
    L_PARK_ALGORITHM = "l_park_algorithm"
    STOPPED_AFTER_PARK = "stopped_after_park"
    RIGHT_TURN_ALGORITHM = "right_turn_algorithm" 
    OBSTACLE_AVOIDANCE = "obstacle_avoidance" # Engelden kaçınma durumu
    
class RobotStateController(Node):
    def __init__(self):
        super().__init__('robot_state_controller')

        self.get_logger().info("Robot State Controller Node Initialized.")

        # Parametreleri tanımlama ve alma
        self.declare_parameter('bus_stop_signal_topic', '/bus_stop_detected_signal')
        self.declare_parameter('stop_sign_algorithm_completion_topic', '/stop_sign_algorithm_completed')
        self.declare_parameter('robot_current_state_topic', '/robot_current_state')
        self.declare_parameter('park_sign_signal_topic', '/park_sign_detected_signal')
        self.declare_parameter('l_park_algorithm_completion_topic', '/l_park_algorithm_completed')
        self.declare_parameter('right_turn_signal_topic', '/right_turn_detected_signal')
        self.declare_parameter('right_turn_algorithm_completion_topic', '/right_turn_algorithm_completed')
        
        # Düz engel algılama için
        self.declare_parameter('straight_obstacle_signal_topic', '/straight_obstacle_detected_signal')
        # ObstacleResponseNode'un tamamlandığını bildiren sinyal (ARTIK YORUM SATIRI DEĞİL)
        self.declare_parameter('obstacle_avoidance_completion_topic', '/obstacle_avoidance_completed')


        self.bus_stop_signal_topic = self.get_parameter('bus_stop_signal_topic').get_parameter_value().string_value
        self.stop_sign_algorithm_completion_topic = self.get_parameter('stop_sign_algorithm_completion_topic').get_parameter_value().string_value
        self.robot_current_state_topic = self.get_parameter('robot_current_state_topic').get_parameter_value().string_value
        self.park_sign_signal_topic = self.get_parameter('park_sign_signal_topic').get_parameter_value().string_value
        self.l_park_algorithm_completion_topic = self.get_parameter('l_park_algorithm_completion_topic').get_parameter_value().string_value
        self.right_turn_signal_topic = self.get_parameter('right_turn_signal_topic').get_parameter_value().string_value
        self.right_turn_algorithm_completion_topic = self.get_parameter('right_turn_algorithm_completion_topic').get_parameter_value().string_value
        
        # Yeni parametre değerlerini alma
        self.straight_obstacle_signal_topic = self.get_parameter('straight_obstacle_signal_topic').get_parameter_value().string_value
        self.obstacle_avoidance_completion_topic = self.get_parameter('obstacle_avoidance_completion_topic').get_parameter_value().string_value


        # Current state of the robot
        self.current_state = RobotState.LANE_FOLLOWING
        # Engel tepkisinin devam edip etmediğini takip eden bayrak (YENİ)
        self.is_obstacle_response_active = False 

        self.get_logger().info(f"Initial State Set Internally: {self.current_state}")

        # Publishers
        self.state_publisher = self.create_publisher(String, self.robot_current_state_topic, 10)
        self.get_logger().info(f"State Publisher created for topic: '{self.robot_current_state_topic}'")

        # ÖNEMLİ: Yayıncı hazır olana kadar bekle ve ilk durumu yayınla
        self.get_logger().info(f"Waiting for {self.robot_current_state_topic} publisher to become ready and publishing initial state...")
        time.sleep(0.5) # Yarım saniye bekle
        self.publish_current_state() # İlk durumu hemen yayınla
        self.get_logger().info("Initial state published immediately after short delay.")


        # Subscribers
        self.create_subscription(
            Bool,
            self.bus_stop_signal_topic,
            self.bus_stop_signal_callback,
            10
        )

        self.create_subscription(
            Bool,
            self.stop_sign_algorithm_completion_topic,
            self.stop_sign_algorithm_completion_callback,
            10
        )
        
        self.create_subscription(
            Bool,
            self.park_sign_signal_topic,
            self.park_sign_signal_callback,
            10
        )

        self.create_subscription(
            Bool,
            self.l_park_algorithm_completion_topic,
            self.l_park_algorithm_completion_callback,
            10
        )
        
        self.create_subscription(
            Bool,
            self.right_turn_signal_topic,
            self.right_turn_signal_callback,
            10
        )

        self.create_subscription(
            Bool,
            self.right_turn_algorithm_completion_topic,
            self.right_turn_algorithm_completion_callback,
            10
        )

        # Düz engel algılama için
        self.create_subscription(
            Bool,
            self.straight_obstacle_signal_topic,
            self.straight_obstacle_signal_callback,
            10
        )
        
        # Engelden kaçınma algoritması tamamlanma sinyali (YORUM SATIRI KALDIRILDI)
        self.create_subscription(
            Bool,
            self.obstacle_avoidance_completion_topic,
            self.obstacle_avoidance_completion_callback,
            10
        )


        self.get_logger().info(f"Subscribing to '{self.bus_stop_signal_topic}' for bus stop signals.")
        self.get_logger().info(f"Subscribing to '{self.stop_sign_algorithm_completion_topic}' for algorithm completion.")
        self.get_logger().info(f"Subscribing to '{self.park_sign_signal_topic}' for park sign signals.")
        self.get_logger().info(f"Subscribing to '{self.l_park_algorithm_completion_topic}' for L-Park algorithm completion.")
        self.get_logger().info(f"Subscribing to '{self.right_turn_signal_topic}' for right turn signals.")
        self.get_logger().info(f"Subscribing to '{self.right_turn_algorithm_completion_topic}' for right turn algorithm completion.")
        self.get_logger().info(f"Subscribing to '{self.straight_obstacle_signal_topic}' for straight obstacle signals.")
        self.get_logger().info(f"Subscribing to '{self.obstacle_avoidance_completion_topic}' for obstacle avoidance completion signals.")


    def publish_current_state(self):
        msg = String()
        msg.data = self.current_state
        self.state_publisher.publish(msg)
        self.get_logger().info(f"Published State: {self.current_state}")

    def bus_stop_signal_callback(self, msg):
        # Yalnızca şerit takibi durumundayken durak sinyalini işle
        if msg.data is True and self.current_state == RobotState.LANE_FOLLOWING:
            self.get_logger().info("Bus stop signal received (True). Transitioning to STOP_SIGN_ALGORITHM state.")
            self.current_state = RobotState.STOP_SIGN_ALGORITHM
            self.publish_current_state()

    def stop_sign_algorithm_completion_callback(self, msg):
        # Sadece algoritma tamamlandığında ve doğru durumdaysak şerit takibine geçiş yap
        if msg.data is True and self.current_state == RobotState.STOP_SIGN_ALGORITHM:
            self.get_logger().info("Stop Sign Algorithm completion signal received (True). Transitioning to LANE_FOLLOWING state.")
            self.current_state = RobotState.LANE_FOLLOWING
            self.publish_current_state()
        elif msg.data is False:
             self.get_logger().info("Stop Sign Algorithm completion signal received (False), no state change.")


    def park_sign_signal_callback(self, msg):
        # Yalnızca şerit takibi durumundayken park sinyalini işle
        if msg.data is True and self.current_state == RobotState.LANE_FOLLOWING:
            self.get_logger().info("Park sign signal received (True). Transitioning to L_PARK_ALGORITHM state.")
            self.current_state = RobotState.L_PARK_ALGORITHM
            self.publish_current_state()

    def l_park_algorithm_completion_callback(self, msg):
        # L-Park algoritması tamamlandığında ve doğru durumdaysak durma durumuna geç
        if msg.data is True and self.current_state == RobotState.L_PARK_ALGORITHM:
            self.get_logger().info("L-Park Algorithm completion signal received (True). Transitioning to STOPPED_AFTER_PARK state.")
            self.current_state = RobotState.STOPPED_AFTER_PARK
            self.publish_current_state()

    def right_turn_signal_callback(self, msg):
        # Sağa dön sinyali alındığında ve şerit takibi durumundaysak dönüş algoritmasına geç
        if msg.data is True and self.current_state == RobotState.LANE_FOLLOWING:
            self.get_logger().info("Right turn signal received (True). Transitioning to RIGHT_TURN_ALGORITHM state.")
            self.current_state = RobotState.RIGHT_TURN_ALGORITHM
            self.publish_current_state()

    def right_turn_algorithm_completion_callback(self, msg):
        # Sağa dönüş algoritması tamamlandığında ve doğru durumdaysak şerit takibine geri dön
        if msg.data is True and self.current_state == RobotState.RIGHT_TURN_ALGORITHM:
            self.get_logger().info("Right Turn Algorithm completion signal received (True). Transitioning to LANE_FOLLOWING state.")
            self.current_state = RobotState.LANE_FOLLOWING
            self.publish_current_state()
        elif msg.data is False:
             self.get_logger().info("Right Turn Algorithm completion signal received (False), no state change.")

    # *** Düz Engel Algılama ve Durum Geçişi Güncellemesi ***
    def straight_obstacle_signal_callback(self, msg):
        # Düz engel sinyali alındığında
        if msg.data is True: # Engel algılandı
            # Eğer robot şerit takibi durumundaysa engelden kaçınmaya geç
            if self.current_state == RobotState.LANE_FOLLOWING:
                self.get_logger().info("Straight obstacle detected signal received (True). Transitioning to OBSTACLE_AVOIDANCE state.")
                self.current_state = RobotState.OBSTACLE_AVOIDANCE
                self.is_obstacle_response_active = True # Engel tepkisi başladı
                self.publish_current_state()
            else:
                self.get_logger().info(f"Straight obstacle detected, but robot is in '{self.current_state}' state. Ignoring signal for now.")
        elif msg.data is False: # Engel artık yok sinyali geldiğinde
            # Bu sinyal geldiğinde hemen LANE_FOLLOWING'e dönmüyoruz.
            # is_obstacle_response_active bayrağı hala True ise, OBSTACLE_AVOIDANCE'da kalmaya devam ediyoruz.
            # Dönüş için obstacle_avoidance_completion_callback'i bekliyoruz.
            if self.is_obstacle_response_active:
                self.get_logger().info("Straight obstacle detected signal received (False), but obstacle response is still active. Remaining in OBSTACLE_AVOIDANCE.")
            # else: eğer is_obstacle_response_active zaten False ise (yani manevra bitmiş ve sonrasında false sinyali gelmişse), 
            # bu durum zaten LANE_FOLLOWING olmalıydı veya başka bir duruma geçişi tetiklemeye gerek yok.


    def obstacle_avoidance_completion_callback(self, msg):
        # Engelden kaçınma algoritmasının tamamlandığı sinyali alındığında
        if msg.data is True and self.current_state == RobotState.OBSTACLE_AVOIDANCE:
            # Engel tepkisi tamamlandı
            self.is_obstacle_response_active = False 
            self.get_logger().info("Obstacle Avoidance Algorithm completion signal received (True). Transitioning to LANE_FOLLOWING state.")
            self.current_state = RobotState.LANE_FOLLOWING
            self.publish_current_state()
        elif msg.data is False:
             self.get_logger().info("Obstacle Avoidance Algorithm completion signal received (False), no state change.")
    # *** Güncelleme Sonu ***


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()