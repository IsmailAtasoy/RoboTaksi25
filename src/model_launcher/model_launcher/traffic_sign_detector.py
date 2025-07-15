import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import os
import torchvision
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool # Bool mesajı için import
from robotaksi_interfaces.msg import TrafficSignDetection
import time # 'time' kütüphanesini import edin

try:
    from ultralytics import YOLO
except ImportError:
    print("Uyarı: Ultralytics kütüphanesi bulunamadı. Lütfen 'pip install ultralytics' ile kurun.")
    print("Veya modelinizi manuel olarak yüklemek için kodu güncelleyin.")
    exit()

class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector_node')

        self.declare_parameter('model_path', 'default_model_path_will_be_set_below')
        self.declare_parameter('camera_topic', '/camera_rgb/image_raw')
        self.declare_parameter('depth_camera_topic', '/camera_depth/depth/image_raw')
        self.declare_parameter('detection_image_topic', '/robotaksi/traffic_sign_detections_image')
        self.declare_parameter('detection_msg_topic', '/robotaksi/traffic_sign_detections')
        self.declare_parameter('display_image', True)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('pedestrian_crossing_trigger_distance', 10.0)
        self.declare_parameter('bus_stop_trigger_distance', 5.0)
        self.declare_parameter('park_sign_trigger_distance', 10.0)
        self.declare_parameter('no_parking_sign_trigger_distance', 5.0)
        self.declare_parameter('right_turn_sign_trigger_distance', 4.0) 
        self.declare_parameter('depth_info_topic', '/camera_depth/camera_info')

        # *** YENİ PARAMETRELER ***
        # Düz engelin algılanması için tetikleme mesafesi
        self.declare_parameter('straight_obstacle_trigger_distance', 3.0) # metre
        # Levha algılanmadığında bu mantığı kullanıp kullanmayacağımızı belirler
        self.declare_parameter('enable_straight_obstacle_detection', True)
        # Yeni engel mesajı göndermeden önce beklenecek süre (saniye)
        self.declare_parameter('straight_obstacle_cooldown_s', 10.0) 
        # *** YENİ PARAMETRELER SONU ***


        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.depth_camera_topic = self.get_parameter('depth_camera_topic').get_parameter_value().string_value
        self.detection_image_topic = self.get_parameter('detection_image_topic').get_parameter_value().string_value
        self.detection_msg_topic = self.get_parameter('detection_msg_topic').get_parameter_value().string_value
        self.display_image = self.get_parameter('display_image').get_parameter_value().bool_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.iou_threshold = self.get_parameter('iou_threshold').get_parameter_value().double_value
        self.pedestrian_crossing_trigger_distance = self.get_parameter('pedestrian_crossing_trigger_distance').get_parameter_value().double_value
        self.bus_stop_trigger_distance = self.get_parameter('bus_stop_trigger_distance').get_parameter_value().double_value
        self.park_sign_trigger_distance = self.get_parameter('park_sign_trigger_distance').get_parameter_value().double_value
        self.no_parking_sign_trigger_distance = self.get_parameter('no_parking_sign_trigger_distance').get_parameter_value().double_value
        self.right_turn_sign_trigger_distance = self.get_parameter('right_turn_sign_trigger_distance').get_parameter_value().double_value
        self.depth_info_topic = self.get_parameter('depth_info_topic').get_parameter_value().string_value

        # *** YENİ PARAMETRE DEĞERLERİNİ AL ***
        self.straight_obstacle_trigger_distance = self.get_parameter('straight_obstacle_trigger_distance').get_parameter_value().double_value
        self.enable_straight_obstacle_detection = self.get_parameter('enable_straight_obstacle_detection').get_parameter_value().bool_value
        self.straight_obstacle_cooldown_s = self.get_parameter('straight_obstacle_cooldown_s').get_parameter_value().double_value
        # *** YENİ PARAMETRE DEĞERLERİ SONU ***


        self.bus_stop_signal_publisher = self.create_publisher(Bool, '/bus_stop_detected_signal', 10)
        self.park_sign_signal_publisher = self.create_publisher(Bool, '/park_sign_detected_signal', 10)
        self.no_parking_sign_signal_publisher = self.create_publisher(Bool, '/no_parking_sign_detected_signal', 10)
        self.right_turn_sign_signal_publisher = self.create_publisher(Bool, '/right_turn_detected_signal', 10)
        # *** YENİ PUBLISHER ***
        self.straight_obstacle_detected_publisher = self.create_publisher(Bool, '/straight_obstacle_detected_signal', 10)
        # *** YENİ PUBLISHER SONU ***


        package_share_directory = get_package_share_directory('model_launcher')
        default_model_full_path = os.path.join(package_share_directory, 'models', 'model.pt')
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if self.model_path == 'default_model_path_will_be_set_below':
            self.model_path = default_model_full_path

        self.get_logger().info(f'Kullanılacak model yolu: {self.model_path}')

        self.bridge = CvBridge()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'YOLOv8 modeli başarıyla yüklendi: {self.model_path}')
            self.model.to(self.device)
        except Exception as e:
            self.get_logger().error(f"YOLOv8 modeli yüklenirken hata oluştu: {e}")
            self.get_logger().error("Model yolu doğru mu ve Ultralytics kurulu mu kontrol edin.")
            rclpy.shutdown()
            return

        self.rgb_sub = message_filters.Subscriber(self, Image, self.camera_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_camera_topic)
        self.depth_info_sub = message_filters.Subscriber(self, CameraInfo, self.depth_info_topic)

        self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub, self.depth_info_sub], 10)
        self.ts.registerCallback(self.synced_callback)

        self.image_publisher_ = self.create_publisher(
            Image,
            self.detection_image_topic,
            10
        )
        self.detection_publisher_ = self.create_publisher(
            TrafficSignDetection,
            self.detection_msg_topic,
            10
        )

        self.get_logger().info(f'Trafik Levhası Algılama Düğümü başlatıldı.')
        self.get_logger().info(f'RGB Kamera topic: {self.camera_topic}')
        self.get_logger().info(f'Derinlik Kamera topic: {self.depth_camera_topic}')
        self.get_logger().info(f'Algılama sonuçları görüntü topic: {self.detection_image_topic}')
        self.get_logger().info(f'Algılama sonuçları mesaj topic: {self.detection_msg_topic}')
        self.get_logger().info(f'Yaya Geçidi Tetikleme Mesafesi: {self.pedestrian_crossing_trigger_distance:.2f}m')
        self.get_logger().info(f'Durak Tabelası Tetikleme Mesafesi: {self.bus_stop_trigger_distance:.2f}m')
        self.get_logger().info(f'Park Levhası Tetikleme Mesafesi: {self.park_sign_trigger_distance:.2f}m')
        self.get_logger().info(f'Park Yapılmaz Levhası Tetikleme Mesafesi: {self.no_parking_sign_trigger_distance:.2f}m')
        self.get_logger().info(f'Sağa Dön Levhası Tetikleme Mesafesi: {self.right_turn_sign_trigger_distance:.2f}m')
        # *** YENİ LOG MESAJLARI ***
        self.get_logger().info(f'Düz Engel Tetikleme Mesafesi: {self.straight_obstacle_trigger_distance:.2f}m')
        self.get_logger().info(f'Düz Engel Algılama Etkin: {self.enable_straight_obstacle_detection}')
        self.get_logger().info(f'Düz Engel Tekrar Tetikleme Bekleme Süresi: {self.straight_obstacle_cooldown_s:.1f}s')
        # *** YENİ LOG MESAJLARI SONU ***

        self.class_names = ['donel kavsak', 'dur tabelasi', 'durak', 'engelli park', 'girilmez', 'iki yonlu yol', 'ileri mecburi', 'ileri ve saga mecburi yon', 'ileri ve sola mecburi yon', 'ileriden saga mecburi yon', 'ileriden sola mecburi yon', 'isikli isaret cihazi', 'kirmizi isik', 'park', 'park yapilmaz', 'sag seridin sonu', 'saga donulmez', 'saga_don', 'sagdan gidiniz', 'serit duzenleme levhasi', 'sol seridin sonu', 'saga_don', 'sola mecburi', 'tunel', 'tasit trafigine kapali', 'tunel', 'yaya gecidi', 'yesil isik']

        self.depth_scale = 1.0

        self.lower_road_hsv = np.array([0, 0, 50])
        self.upper_road_hsv = np.array([180, 50, 150])

        # Yeni durum değişkenleri
        self.is_pedestrian_crossing_near_and_triggered = False
        self.is_bus_stop_triggered = False
        self.is_park_sign_triggered = False
        self.is_no_parking_sign_triggered = False
        self.is_right_turn_sign_triggered = False
        # *** YENİ DURUM DEĞİŞKENİ ***
        self.is_straight_obstacle_triggered = False
        self.last_straight_obstacle_trigger_time = 0.0 # Son tetiklenme zamanı
        # *** YENİ DURUM DEĞİŞKENİ SONU ***


    def load_raw_torch_model(self, model_path):
        self.get_logger().warn("load_raw_torch_model fonksiyonu şu anda kullanılmıyor çünkü Ultralytics YOLO sınıfı kullanılıyor.")
        return None

    def preprocess_image(self, cv_image):
        self.get_logger().warn("preprocess_image fonksiyonu şu anda kullanılmıyor çünkü Ultralytics YOLO sınıfı kullanılıyor.")
        return None

    def postprocess_detections(self, predictions, original_shape):
        detections = []
        for r in predictions:
            if r.boxes is not None:
                boxes_data = r.boxes.data.cpu().numpy()

                for *xyxy, conf, cls in boxes_data:
                    x1, y1, x2, y2 = map(int, xyxy)
                    confidence = float(conf)
                    class_id = int(cls)

                    detections.append({
                        'box': [x1, y1, x2, y2],
                        'confidence': confidence,
                        'class_id': class_id,
                        'label': self.class_names[class_id] if class_id < len(self.class_names) else str(class_id)
                    })
        return detections

    def synced_callback(self, rgb_msg, depth_msg, depth_info_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')

            if depth_msg.encoding == "32FC1":
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
                self.depth_scale = 1.0
            elif depth_msg.encoding == "16UC1":
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
                self.depth_scale = 0.001
            else:
                self.get_logger().error(f"Desteklenmeyen derinlik görüntüsü kodlaması: {depth_msg.encoding}")
                return

            if cv_image.shape[:2] != depth_image.shape[:2]:
                self.get_logger().warn(f"RGB ({cv_image.shape[:2]}) ve Derinlik ({depth_image.shape[:2]}) görüntü boyutları farklı, derinlik görüntüsü yeniden boyutlandırılıyor.")
                depth_image_resized = cv2.resize(depth_image, (cv_image.shape[1], cv_image.shape[0]),
                                                interpolation=cv2.INTER_NEAREST)
                depth_image = depth_image_resized

        except Exception as e:
            self.get_logger().error(f'Görüntü veya CameraInfo dönüştürme hatası (senkronize): {e}')
            return

        original_shape = cv_image.shape

        predictions = self.model.predict(cv_image, conf=self.confidence_threshold, iou=self.iou_threshold, verbose=False)

        processed_detections = self.postprocess_detections(predictions, original_shape)

        output_image = cv_image.copy()

        # Bu karede bir yaya geçidi, durak, park, park yapılmaz veya sağa dön tabelası algılanıp algılanmadığını kontrol etmek için bayraklar
        pedestrian_crossing_found_in_frame = False
        bus_stop_found_in_frame = False
        park_sign_found_in_frame = False
        no_parking_sign_found_in_frame = False
        right_turn_sign_found_in_frame = False
        
        # *** YENİ: Herhangi bir trafik levhası algılandı mı? ***
        traffic_sign_detected_in_frame = False
        if len(processed_detections) > 0:
            traffic_sign_detected_in_frame = True
        # *** YENİ SONU ***


        # --- Trafik Levhası Uzaklığı Hesaplaması ---
        for det in processed_detections:
            x1, y1, x2, y2 = det['box']
            label = det['label']
            confidence = det['confidence']

            x1 = max(0, x1)
            y1 = max(0, y1)
            x2 = min(original_shape[1], x2)
            y2 = min(original_shape[0], y2)

            if x2 <= x1 or y2 <= y1:
                self.get_logger().warn(f"Geçersiz sınırlayıcı kutu koordinatları: [{x1}, {y1}, {x2}, {y2}]")
                continue

            depth_roi = depth_image[y1:y2, x1:x2]

            valid_depths = depth_roi[
                (depth_roi > 0) &
                np.isfinite(depth_roi)
            ]

            current_sign_distance_m = float('inf')
            if len(valid_depths) > 0:
                raw_distance_value = np.median(valid_depths)
                current_sign_distance_m = raw_distance_value * self.depth_scale
            
                # Şüpheli uzaklıkları filtrele
                if current_sign_distance_m > 10.0 or current_sign_distance_m < 0.1: # 10m'den uzak veya 0.1m'den yakın engelleri filtrele
                    self.get_logger().warn(f"Şüpheli trafik levhası uzaklığı: {current_sign_distance_m:.2f}m. Filtreleniyor. Levha: {label}")
                    current_sign_distance_m = float('inf')
            else:
                self.get_logger().warn(f"'{label}' levhası için geçerli derinlik bilgisi bulunamadı. Kutu: ({x1},{y1})-({x2},{y2})")

            output_image = self.draw_detections(output_image, [det], current_sign_distance_m)

            # **Algılanan levha bir "yaya_gecidi" ise ve mesafe kontrolü yapılıyorsa**
            if label == "yaya_gecidi":
                pedestrian_crossing_found_in_frame = True
                if current_sign_distance_m < self.pedestrian_crossing_trigger_distance and not self.is_pedestrian_crossing_near_and_triggered:
                    detection_msg = TrafficSignDetection()
                    detection_msg.label = label
                    detection_msg.confidence = confidence
                    detection_msg.is_pedestrian_crossing = True
                    detection_msg.distance = current_sign_distance_m
                    self.detection_publisher_.publish(detection_msg)
                    self.get_logger().info(f"Yaya Geçidi Algılandı (Yakınlık Tetiklendi)! Güven: {confidence:.2f}, Mesafe: {current_sign_distance_m:.2f}m - Mesaj Yayınlandı")
                    self.is_pedestrian_crossing_near_and_triggered = True
                elif current_sign_distance_m < self.pedestrian_crossing_trigger_distance and self.is_pedestrian_crossing_near_and_triggered:
                    self.get_logger().info(f"Yaya Geçidi hala yakın ({current_sign_distance_m:.2f}m), ancak zaten tetiklendi. Yeni mesaj yayınlanmıyor.")
                # else:
                #     self.get_logger().info(f"Yaya Geçidi algılandı ({current_sign_distance_m:.2f}m), ancak tetikleme mesafesinden ({self.pedestrian_crossing_trigger_distance:.2f}m) daha uzakta.")

            # **Algılanan levha bir "durak" ise ve mesafe kontrolü yapılıyorsa**
            if label == "durak":
                bus_stop_found_in_frame = True
                if current_sign_distance_m < self.bus_stop_trigger_distance and not self.is_bus_stop_triggered:
                    bool_msg = Bool()
                    bool_msg.data = True
                    self.bus_stop_signal_publisher.publish(bool_msg)
                    self.get_logger().info(f"Durak Tabelası Algılandı (Yakınlık Tetiklendi)! Güven: {confidence:.2f}, Mesafe: {current_sign_distance_m:.2f}m - Bool Mesajı Yayınlandı")
                    self.is_bus_stop_triggered = True
                # elif current_sign_distance_m < self.bus_stop_trigger_distance and self.is_bus_stop_triggered:
                #     self.get_logger().info(f"Durak Tabelası hala yakın ({current_sign_distance_m:.2f}m), ancak zaten tetiklendi. Yeni mesaj yayınlanmıyor.")
                # else:
                #     self.get_logger().info(f"Durak Tabelası algılandı ({current_sign_distance_m:.2f}m), ancak tetikleme mesafesinden ({self.bus_stop_trigger_distance:.2f}m) daha uzakta.")

            # **Algılanan levha bir "park" ise ve mesafe kontrolü yapılıyorsa**
            if label == "park" or label == "engelli park":
                park_sign_found_in_frame = True
                if current_sign_distance_m < self.park_sign_trigger_distance and not self.is_park_sign_triggered:
                    bool_msg = Bool()
                    bool_msg.data = True
                    self.park_sign_signal_publisher.publish(bool_msg)
                    self.get_logger().info(f"Park Levhası Algılandı (Yakınlık Tetiklendi)! Güven: {confidence:.2f}, Mesafe: {current_sign_distance_m:.2f}m - Bool Mesajı Yayınlandı")
                    self.is_park_sign_triggered = True
                # elif current_sign_distance_m < self.park_sign_trigger_distance and self.is_park_sign_triggered:
                #     self.get_logger().info(f"Park Levhası hala yakın ({current_sign_distance_m:.2f}m), ancak zaten tetiklendi. Yeni mesaj yayınlanmıyor.")
                # else:
                #     self.get_logger().info(f"Park Levhası algılandı ({current_sign_distance_m:.2f}m), ancak tetikleme mesafesinden ({self.park_sign_trigger_distance:.2f}m) daha uzakta.")

            # **Algılanan levha bir "park yapilmaz" ise ve mesafe kontrolü yapılıyorsa**
            if label == "park yapilmaz":
                no_parking_sign_found_in_frame = True
                if current_sign_distance_m < self.no_parking_sign_trigger_distance and not self.is_no_parking_sign_triggered:
                    bool_msg = Bool()
                    bool_msg.data = True
                    self.no_parking_sign_signal_publisher.publish(bool_msg)
                    self.get_logger().info(f"Park Yapılmaz Levhası Algılandı (Yakınlık Tetiklendi)! Güven: {confidence:.2f}, Mesafe: {current_sign_distance_m:.2f}m - Bool Mesajı Yayınlandı")
                    self.is_no_parking_sign_triggered = True
                # elif current_sign_distance_m < self.no_parking_sign_trigger_distance and self.is_no_parking_sign_triggered:
                #     self.get_logger().info(f"Park Yapılmaz Levhası hala yakın ({current_sign_distance_m:.2f}m), ancak zaten tetiklendi. Yeni mesaj yayınlanmıyor.")
                # else:
                #     self.get_logger().info(f"Park Yapılmaz Levhası algılandı ({current_sign_distance_m:.2f}m), ancak tetikleme mesafesinden ({self.no_parking_sign_trigger_distance:.2f}m) daha uzakta.")

            # **Algılanan levha bir "saga_don" ise ve mesafe kontrolü yapılıyorsa**
            if label == "saga_don": # Modelinizdeki tam sınıf adını kullanın
                right_turn_sign_found_in_frame = True
                # Sağa Dön levhası tetikleme mesafesi kontrolü
                if current_sign_distance_m < self.right_turn_sign_trigger_distance and not self.is_right_turn_sign_triggered:
                    bool_msg = Bool()
                    bool_msg.data = True
                    self.right_turn_sign_signal_publisher.publish(bool_msg) # Sağa Dön Bool mesajını yayınla!
                    self.get_logger().info(f"Sağa Dön Levhası Algılandı (Yakınlık Tetiklendi)! Güven: {confidence:.2f}, Mesafe: {current_sign_distance_m:.2f}m - Bool Mesajı Yayınlandı")
                    self.is_right_turn_sign_triggered = True # Mesajı yayınladığımızı işaretle
                # elif current_sign_distance_m < self.right_turn_sign_trigger_distance and self.is_right_turn_sign_triggered:
                #     self.get_logger().info(f"Sağa Dön Levhası hala yakın ({current_sign_distance_m:.2f}m), ancak zaten tetiklendi. Yeni mesaj yayınlanmıyor.")
                # else:
                #     self.get_logger().info(f"Sağa Dön Levhası algılandı ({current_sign_distance_m:.2f}m), ancak tetikleme mesafesinden ({self.right_turn_sign_trigger_distance:.2f}m) daha uzakta.")

        # --- Durum Bayraklarını Sıfırlama Mantığı ---
        # Bu kısım her karede çalışır ve trafik işaretlerinin varlığına göre bayrakları sıfırlar.
        # Bu, bir işaret görüş alanından çıktığında veya çok uzakta olduğunda algılama durumunu sıfırlar.

        if not pedestrian_crossing_found_in_frame:
            if self.is_pedestrian_crossing_near_and_triggered:
                self.get_logger().info("Yaya Geçidi artık algılanmıyor veya uzakta. Tetikleme bayrağı sıfırlanıyor.")
                self.is_pedestrian_crossing_near_and_triggered = False

        # Durak tabelası, Park Levhası ve Park Yapılmaz Levhası için otomatik sıfırlama yapmıyoruz
        # çünkü bunlar genellikle bir eylem (durma, park etme) gerektirir ve bu eylemler
        # başka bir düğüm tarafından yönetilmelidir. Ancak eğer levha görüş alanından tamamen çıkarsa
        # ve o eylem tamamlandıysa, ilgili düğümün bu bayrağı sıfırlaması gerekir.
        # Şu anki mantıkta sadece True sinyali bir kez gönderiliyor ve bu bayraklar bu düğüm tarafından
        # manuel olarak False'a ayarlanmıyor. Bu, RobotStateController gibi bir üst düzey düğümün
        # bu durumları yönettiğini varsayarız.

        if not right_turn_sign_found_in_frame:
            if self.is_right_turn_sign_triggered:
                self.get_logger().info("Sağa Dön Levhası artık algılanmıyor veya uzakta. Tetikleme bayrağı sıfırlanıyor.")
                self.is_right_turn_sign_triggered = False


        # --- Gelişmiş Genel Engel Uzaklığı Hesaplaması ---
        h, w = depth_image.shape[:2]

        general_roi_y_start = int(h * 0.55)
        general_roi_y_end = int(h * 0.65)
        general_roi_x_start = int(w * 0.35)
        general_roi_x_end = int(w * 0.45)

        if general_roi_y_start >= general_roi_y_end or general_roi_x_start >= general_roi_x_end:
            self.get_logger().warn("Genel engel ROI koordinatları geçersiz, varsayılan ROI kullanılıyor.")
            general_roi_y_start = int(h * 0.4)
            general_roi_y_end = int(h * 0.7)
            general_roi_x_start = int(w * 0.2)
            general_roi_x_end = int(w * 0.8)

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        road_mask = cv2.inRange(hsv_image, self.lower_road_hsv, self.upper_road_hsv)

        road_mask_roi = road_mask[general_roi_y_start:general_roi_y_end, general_roi_x_start:general_roi_x_end]
        general_depth_roi = depth_image[general_roi_y_start:general_roi_y_end, general_roi_x_start:general_roi_x_end]

        valid_depths_outside_road = general_depth_roi[
            (~(road_mask_roi > 0)) & # Yol maskesi olmayan pikseller
            (general_depth_roi > 0) & # Derinliği sıfırdan büyük olan pikseller (geçerli derinlik)
            np.isfinite(general_depth_roi) # Sonsuz olmayan derinlik değerleri
        ]

        overall_min_distance_m = float('inf')
        if len(valid_depths_outside_road) > 0:
            raw_min_distance_value = np.min(valid_depths_outside_road) # En yakın derinlik değeri
            overall_min_distance_m = raw_min_distance_value * self.depth_scale
        
            if overall_min_distance_m > 10.0 or overall_min_distance_m < 0.1: # Şüpheli uzaklıkları filtrele
                self.get_logger().warn(f"Şüpheli genel engel uzaklığı (yol hariç): {overall_min_distance_m:.2f}m. Filtreleniyor.")
                overall_min_distance_m = float('inf')

        display_general_distance_text = f"Genel Engel (Yol Haric): "
        if overall_min_distance_m != float('inf'):
            display_general_distance_text += f"{overall_min_distance_m:.2f}m"
        else:
            display_general_distance_text += "Yok/Bilinmiyor"

        cv2.putText(output_image, display_general_distance_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        cv2.rectangle(output_image, (general_roi_x_start, general_roi_y_start),
                    (general_roi_x_end, general_roi_y_end), (0, 0, 255), 2)

        # *** YENİ: Levha algılanmadığında ve engel mesafesi uygunsa sinyal gönder ***
        # Fotoğraftaki gibi bir engel için, levha algılanmadığı durumlarda genel engel ROI'sine bakacağız.
        # Bu engelin 'dikdörtgen' olduğu varsayımı, zaten ROI'nin kendisi tarafından sağlanıyor.
        if self.enable_straight_obstacle_detection and not traffic_sign_detected_in_frame:
            current_time = time.time() # Mevcut zamanı al

            # Engel algılandıysa VE daha önce tetiklenmediyse VEYA bekleme süresi dolduysa
            if overall_min_distance_m < self.straight_obstacle_trigger_distance and (
               not self.is_straight_obstacle_triggered or 
               (current_time - self.last_straight_obstacle_trigger_time >= self.straight_obstacle_cooldown_s)
            ):
                # Sinyali yayınla
                bool_msg = Bool()
                bool_msg.data = True
                self.straight_obstacle_detected_publisher.publish(bool_msg)
                self.get_logger().info(f"Düz Engel Algılandı (Levha Yok, Mesafe: {overall_min_distance_m:.2f}m)! Bool Mesajı Yayınlandı.")
                self.is_straight_obstacle_triggered = True # Bayrağı ayarla
                self.last_straight_obstacle_trigger_time = current_time # Son tetiklenme zamanını güncelle
            elif overall_min_distance_m >= self.straight_obstacle_trigger_distance and self.is_straight_obstacle_triggered:
                # Engel yeterince uzakta veya görüş alanından çıktıysa bayrağı sıfırla
                # NOT: Bu sıfırlama, engel uzaklaştığında her zaman gerçekleşir,
                # 10 saniye kısıtlaması sadece 'True' mesajı göndermek için geçerlidir.
                bool_msg = Bool()
                bool_msg.data = False
                self.straight_obstacle_detected_publisher.publish(bool_msg) # False sinyali gönder
                self.get_logger().info(f"Düz Engel artık uzakta veya görüş alanından çıktı. Tetikleme bayrağı sıfırlanıyor.")
                self.is_straight_obstacle_triggered = False
                # Bu durumda last_straight_obstacle_trigger_time'ı sıfırlamaya gerek yok,
                # çünkü 'True' tetiklenmesi için 10 saniye kuralı zaten onu yönetecek.
            # else: Eğer zaten tetiklendiyse ve bekleme süresi dolmadıysa veya henüz tetikleme mesafesine gelmediyse log spam'ini önlemek için bir şey yapmıyoruz.
            
        # Eğer `traffic_sign_detected_in_frame` True ise, yani bir levha algılandıysa,
        # düz engel algılama mantığını resetlememiz veya pas geçmemiz gerekebilir.
        # Şu anki mantık `not traffic_sign_detected_in_frame` ile zaten bunu sağlıyor.
        # Yani levha algılandığında düz engel mantığı devreye girmiyor.
        # Eğer levha algılandığında düz engel sinyalinin DURMASINI istiyorsanız (yani ikisi aynı anda olmasın):
        if traffic_sign_detected_in_frame and self.is_straight_obstacle_triggered:
            bool_msg = Bool()
            bool_msg.data = False
            self.straight_obstacle_detected_publisher.publish(bool_msg)
            self.get_logger().info("Trafik levhası algılandığı için düz engel tetikleme bayrağı sıfırlandı.")
            self.is_straight_obstacle_triggered = False
            # Levha öncelikli olduğu için cooldown'ı sıfırlayabiliriz
            self.last_straight_obstacle_trigger_time = 0.0

        # *** YENİ SONU ***


        if self.display_image:
            road_mask_bgr = cv2.cvtColor(road_mask, cv2.COLOR_GRAY2BGR)
            output_image = cv2.addWeighted(output_image, 1, road_mask_bgr, 0.2, 0)

        # --- Görüntü Yayınlama ve Görselleştirme ---
        if self.display_image:
            display_depth_image = depth_image.copy()
            display_depth_image[~np.isfinite(display_depth_image)] = 0
            display_depth_image[display_depth_image == 0] = 0

            max_display_distance = 10.0
            display_depth_image = (display_depth_image / max_display_distance * 255).astype(np.uint8)
            display_depth_image = cv2.applyColorMap(display_depth_image, cv2.COLORMAP_JET)

            cv2.imshow("Traffic Sign Detections with Distance", output_image)
            cv2.imshow("Raw Depth Image (Normalized)", display_depth_image)
            cv2.waitKey(1)

        try:
            self.image_publisher_.publish(self.bridge.cv2_to_imgmsg(output_image, encoding='bgr8'))
        except Exception as e:
            self.get_logger().error(f'İşlenmiş görüntü yayınlama hatası: {e}')

    def draw_detections(self, image, detections, distance_m=None):
        for det in detections:
            x1, y1, x2, y2 = det['box']
            label = det.get('label', str(det.get('class_id', '')))
            confidence = det.get('confidence', 0)
            color = (0, 255, 0)

            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

            display_text = f"{label} {confidence:.2f}"
            if distance_m is not None and distance_m != float('inf'):
                display_text += f" ({distance_m:.2f}m)"
            elif distance_m == float('inf'):
                display_text += " (Uzaklık Yok)"

            font_scale = max(0.4, min(1.0, (x2 - x1) / 200.0))
            font_thickness = max(1, int(font_scale * 2))

            cv2.putText(image, display_text, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, font_thickness)
        return image

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
