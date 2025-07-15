# 🚗 Robotaksi25

**Robotaksi25**, [Robotaksi Binek Otonom Yarışması](https://robotaksi.org/) kapsamında geliştirilen otonom araç sistemini içerir. Bu proje, araç algılama, yol planlama, kontrol sinyalleri üretme gibi adımları entegre ederek tümleşik bir sürücüsüz sistem sunmayı hedefler.

---

## 🎯 İçerik

- `perception/`: Kameralı nesne tanıma ve yol izleme algoritmaları  
- `mapping/`: Harita oluşturma ve SLAM modülleri  
- `planning/`: Yol planlama ve görev yönetimi  
- `control/`: Araç kontrol algoritmaları (PID, MPC vb.)  
- `simulation/`: Gazebo/ROS veya PyBullet tabanlı simülasyon ortamları  
- `hardware/`: Sensör entegrasyonları, CAN-bus arayüzü, düşük seviye iletişim  
- `docs/`: Yapılandırılmış dokümantasyon dosyaları  

---

## 📋 Özellikler

- **Araç algılama ve sınıflandırma**  
  - YOLOv8 veya benzeri modellerle nesne tespiti  
  - Gerçek zamanlı yol çizgisi algılama  
- **Yol planlama**  
  - A* veya RRT tabanlı global yol planlaması  
  - MPC tabanlı lokal kontrol  
- **Kontrol sistemi**  
  - PID kontrolör ile hız ve direksiyon kontrolü  
  - CAN-BUS üzerinden araç komut gönderimi  
- **SLAM & Haritalama**  
  - Lidar ve/veya stereo kamera bazlı harita oluşturma  
- **Simülasyon desteği**  
  - Simülasyon tabanlı test için ROS ve Gazebo örnekleri  
- **Modüler yapı**  
  - Her bir aşama için ayrı ROS-package veya Python modülleri  

---

## ⚙️ Kurulum

### Gereksinimler

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.8+ ve gerekli paketler


