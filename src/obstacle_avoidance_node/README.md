# Engelden Kaçınma Node

Bu ROS2 package'ı, araç için basit engelden kaçınma hareketi gerçekleştiren node'ları içerir.

## Hareket Deseni

Node, aşağıdaki basit hareket desenini gerçekleştirir:

**Sola dönüp engelden kaçınana kadar devam et** - Sol ve sağ ön tekerleri negatif açıya çevirerek sola dönüş yapar ve belirlenen süre boyunca bu hareketi sürdürür.

## Kullanım

### Basit Node
```bash
ros2 run obstacle_avoidance_node obstacle_avoidance
```

### Gelişmiş Node (Parametreli)
```bash
ros2 run obstacle_avoidance_node advanced_obstacle_avoidance
```

### Parametreler (Gelişmiş Node)

- `front_velocity`: Ön tekerler hızı (m/s) - Varsayılan: 0.5
- `rear_velocity`: Arka tekerler hızı (m/s) - Varsayılan: 0.5
- `left_angle`: Sol ön teker açısı (radyan) - Varsayılan: 1.5
- `right_angle`: Sağ ön teker açısı (radyan) - Varsayılan: 1.5
- `step_distance`: Adım mesafesi (m) - Varsayılan: 2.0
- `step_duration`: Engelden kaçınma süresi (s) - Varsayılan: 15.0
- `wobble_frequency`: Sallanma frekansı (Hz) - Varsayılan: 2.0
- `wobble_amplitude`: Sallanma genliği (radyan) - Varsayılan: 0.1

### Parametre ile Çalıştırma
```bash
ros2 run obstacle_avoidance_node advanced_obstacle_avoidance --ros-args \
  -p front_velocity:=0.3 \
  -p rear_velocity:=0.4 \
  -p left_angle:=1.0 \
  -p right_angle:=1.0 \
  -p step_duration:=20.0
```

## Topics

### Subscribed Topics
- Yok

### Published Topics
- `/velocity_controller/commands` (std_msgs/msg/Float64MultiArray)
  - `data[0]`: Ön tekerler hızı (m/s)
  - `data[1]`: Arka tekerler hızı (m/s)

- `/position_controller/commands` (std_msgs/msg/Float64MultiArray)
  - `data[0]`: Sol ön teker açısı (radyan)
  - `data[1]`: Sağ ön teker açısı (radyan)

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select obstacle_avoidance_node
source install/setup.bash
```

## Manuel Komutlar

Node çalışmadan önce manuel olarak test etmek için:

```bash
# Açı komutu (sol teker, sağ teker) - sola dönüş
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [-1.5, -1.5]"

# Hız komutu (ön tekerler, arka tekerler)
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, 0.5]"
```

## Açı Değerleri

- **Negatif değerler**: Sola dönüş (engelden kaçınma)
- **Pozitif değerler**: Sağa dönüş
- **0.0**: Düz gitme
- **1.5 radyan**: Yaklaşık 86 derece

## Doğal Hareket

Node, tekerlerin doğal sallanma hareketi ile çalışır:
- **Sallanma frekansı**: 2 Hz (saniyede 2 kez)
- **Sallanma genliği**: 0.1 radyan (yaklaşık 5.7 derece)
- Bu sayede robotik değil, organik hareket sağlanır 