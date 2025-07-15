import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Model yolu, eğer TrafficSignDetector için gerekiyorsa
    # 'model_launcher' paketi yerine kendi modelinizin bulunduğu paketi kullanmalısınız
    # Örneğin, 'your_package_name' veya modelinizi içeren gerçek paket adı.
    model_launcher_share_dir = get_package_share_directory('model_launcher') 
    model_path = os.path.join(model_launcher_share_dir, 'models', 'model.pt') # Modelinizin yolu

    ld = LaunchDescription()

    # 1. TrafficSignDetector Node
    traffic_sign_detector_node = Node(
        package='model_launcher', # Bu paketin adı sizin traffic_sign_detector.py dosyanızı içeren paket olmalı
        executable='traffic_sign_detector', # traffic_sign_detector.py dosyasındaki main fonksiyonunun entry point adı (setup.py'de tanımlı)
        name='traffic_sign_detector',
        output='screen',
        parameters=[
            {'model_path': model_path},
            {'camera_topic': '/camera_rgb/image_raw'},
            {'depth_camera_topic': '/camera_depth/depth/image_raw'},
            {'detection_image_topic': '/robotaksi/traffic_sign_detections_image'}, # Eklenmiş topic
            {'detection_msg_topic': '/robotaksi/traffic_sign_detections'}, # Eklenmiş topic
            {'display_image': True},
            {'confidence_threshold': 0.5},
            {'iou_threshold': 0.45},
            {'pedestrian_crossing_trigger_distance': 10.0},
            {'bus_stop_trigger_distance': 5.0},
            {'park_sign_trigger_distance': 10.0},
            {'no_parking_sign_trigger_distance': 5.0}, # Eklenmiş parametre
            {'right_turn_sign_trigger_distance': 4.0}, # Sağa dönüş tabelası için tetikleme mesafesi (adı düzeltildi)
            {'depth_info_topic': '/camera_depth/camera_info'}, # Eklenmiş parametre
            # *** YENİ PARAMETRELER: Düz engel algılama için ***
            {'straight_obstacle_trigger_distance': 3.0}, # Düz engel tetikleme mesafesi (metre)
            {'enable_straight_obstacle_detection': True} # Düz engel algılamayı etkinleştir
            # *** YENİ PARAMETRELER SONU ***
        ]
    )

    # 2. RobotStateController Node
    robot_state_controller_node = Node(
        package='general_control', # Bu paketin adı sizin robot_state_controller.py dosyanızı içeren paket olmalı
        executable='robot_state_controller', # robot_state_controller.py dosyasındaki main fonksiyonunun entry point adı
        name='robot_state_controller',
        output='screen',
        parameters=[
            {'bus_stop_signal_topic': '/bus_stop_detected_signal'},
            {'stop_sign_algorithm_completion_topic': '/stop_sign_algorithm_completed'},
            {'park_sign_signal_topic': '/park_sign_detected_signal'},
            {'l_park_algorithm_completion_topic': '/l_park_algorithm_completed'},
            {'robot_current_state_topic': '/robot_current_state'},
            {'right_turn_signal_topic': '/right_turn_detected_signal'}, 
            {'right_turn_algorithm_completion_topic': '/right_turn_algorithm_completed'},
            # *** YENİ PARAMETRE: Düz engel algılama sinyal topic'i ***
            {'straight_obstacle_signal_topic': '/straight_obstacle_detected_signal'}
            # *** YENİ PARAMETRE SONU ***
        ]
    )

    # 3. StopSignAlgorithm Node (durak_controller paketi altında olduğu varsayılıyor)
    stop_sign_algorithm_node = Node(
        package='durak_controller',
        executable='stop_sign_node', # stop_sign_node.py dosyasının entry point adı
        name='stop_sign_algorithm',
        output='screen',
        parameters=[
            {'robot_current_state_topic': '/robot_current_state'},
            {'stop_sign_algorithm_completion_topic': '/stop_sign_algorithm_completed'},
            {'algorithm_duration_sec': 10.0} # Örnek süre, ayarlayın
        ]
    )

    # 4. LParkNode (durak_controller paketi altında olduğu varsayılıyor)
    l_park_node = Node(
        package='durak_controller',
        executable='l_park_node', # l_park_node.py dosyasının entry point adı
        name='l_park_algorithm',
        output='screen'
    )

    # 5. RightTurnAlgorithm Node (durak_controller paketi altında olduğu varsayılıyor)
    right_turn_algorithm_node = Node(
        package='durak_controller', 
        executable='right_turn_node', 
        name='right_turn_algorithm', 
        output='screen',
        parameters=[ 
            {'base_linear_speed': 0.5},
            {'wheel_radius': 0.1},
            {'ramp_steps': 20},
            {'ramp_interval': 0.05},
            {'turn_angle_degrees': -45.0}, 
            {'turn_forward_distance': 5.0}, 
            {'pre_turn_forward_distance': 6.5}, 
            {'post_turn_forward_distance': 1.5}, 
            {'turn_ramp_duration_s': 1.0} 
        ]
    )

    # *** YENİ NODE: ObstacleResponseNode ***
    obstacle_response_node = Node(
        package='durak_controller', # obstacle_response_node.py dosyanızı içeren paketin adı
        executable='obstacle_response_node', # obstacle_response_node.py dosyasındaki main fonksiyonunun entry point adı
        name='obstacle_response_node',
        output='screen',
        parameters=[
            # Burada ObstacleResponseNode için ek parametreler tanımlayabilirsiniz,
            # ancak şu anda basit olduğu için zorunlu değil.
        ]
    )
    # *** YENİ NODE SONU ***


    # 6. RobustLaneDetector Node
    robust_lane_detector_node = Node(
        package='lane_follower',
        executable='lane_follower',
        name='lane_follower',
        output='screen',
        parameters=[
            {'roi_height_ratio': 0.6},
            {'roi_width_ratio': 0.8},
            {'min_line_angle': 20.0},
            {'lane_width_pixels': 300}
        ]
    )

    # 7. LaneControlPID Node (veya LaneControlDirect - ikisini aynı anda çalıştırmayın)
    lane_control_pid_node = Node(
        package='lane_follower',
        executable='lane_control_pid',
        name='lane_control_pid',
        output='screen',
        parameters=[
            {'kp': 0.5},
            {'ki': 0.02},
            {'kd': 0.1},
            {'max_output': 0.5},
            {'base_speed': 0.8},
            {'min_speed': 0.5},
            {'timeout_sec': 1.0}
        ]
    )

    # Düğümleri LaunchDescription'a ekle
    ld.add_action(traffic_sign_detector_node)
    ld.add_action(robot_state_controller_node)
    ld.add_action(stop_sign_algorithm_node)
    ld.add_action(l_park_node)
    ld.add_action(right_turn_algorithm_node)
    ld.add_action(obstacle_response_node) # YENİ: ObstacleResponseNode'u ekliyoruz!
    ld.add_action(robust_lane_detector_node)
    ld.add_action(lane_control_pid_node)

    return ld