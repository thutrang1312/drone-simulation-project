# DRONE SIMULATION PROJECT - TỔNG QUAN

## 1. MỤC TIÊU DỰ ÁN

Xây dựng hệ thống mô phỏng drone sử dụng ROS + Gazebo với các chức năng:
- Mô phỏng quadrotor trong môi trường 3D
- Điều khiển drone tự động và bằng keyboard
- Đọc dữ liệu từ sensors (IMU, Camera)
- Navigation và tránh vật cản

## 2. KIẾN TRÚC HỆ THỐNG

### 2.1. Thành phần chính
┌─────────────────────────────────────────┐
│          GAZEBO SIMULATOR               │
│  ┌───────────┐      ┌────────────────┐ │
│  │  Drone    │      │  Environment   │ │
│  │  Model    │      │  (World)       │ │
│  └─────┬─────┘      └────────────────┘ │
│        │                                │
│        │ Physics Simulation             │
└────────┼────────────────────────────────┘
│
│ ROS Topics
│
┌────────┼────────────────────────────────┐
│        │         ROS NODES              │
│  ┌─────▼─────┐                          │
│  │  Gazebo   │                          │
│  │  Plugin   │                          │
│  └─────┬─────┘                          │
│        │                                │
│  ┌─────▼──────┐  ┌──────────────────┐  │
│  │ Controller │  │  Sensor Reader   │  │
│  │   Node     │  │     Node         │  │
│  └────────────┘  └──────────────────┘  │
│                                         │
│  ┌────────────────────────────────────┐│
│  │      Keyboard Control Node         ││
│  └────────────────────────────────────┘│
└─────────────────────────────────────────┘

### 2.2. ROS Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Lệnh điều khiển vận tốc |
| `/imu` | sensor_msgs/Imu | Dữ liệu IMU |
| `/ground_truth/state` | nav_msgs/Odometry | Vị trí thực tế drone |
| `/quadrotor/camera/image_raw` | sensor_msgs/Image | Hình ảnh từ camera |

### 2.3. Nodes

| Node | File | Chức năng |
|------|------|-----------|
| drone_controller | drone_controller.py | Điều khiển tự động |
| sensor_reader | sensor_reader.py | Đọc dữ liệu sensors |
| keyboard_control | keyboard_control.py | Điều khiển keyboard |

## 3. CẤU TRÚC THƯ MỤC
drone_project/
├── urdf/                  # Robot models
│   ├── quadrotor.urdf
│   └── quadrotor.xacro
├── worlds/                # Gazebo environments
│   ├── empty.world
│   ├── obstacle_world.world
│   └── indoor_world.world
├── launch/                # Launch files
│   ├── drone_simulation.launch
│   └── drone_control.launch
├── scripts/               # Python nodes
│   ├── drone_controller.py
│   ├── sensor_reader.py
│   └── keyboard_control.py
├── config/                # Config files
│   └── params.yaml
├── docs/                  # Documentation
│   ├── OVERVIEW.md
│   └── WORKFLOW.md
└── meshes/                # 3D models

## 4. YÊU CẦU HỆ THỐNG

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- Python 3.8+
- OpenCV (cho xử lý ảnh)

## 5. CÀI ĐẶT

### 5.1. Clone repository
```bash
git clone https://github.com/YOUR_USERNAME/drone-simulation-project.git ~/drone_ws/src/
cd ~/drone_ws
```

### 5.2. Install dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 5.3. Build workspace
```bash
catkin_make
source devel/setup.bash
```

## 6. SỬ DỤNG

### 6.1. Khởi động simulation
```bash
roslaunch drone_project drone_simulation.launch
```

### 6.2. Điều khiển tự động
```bash
rosrun drone_project drone_controller.py
```

### 6.3. Điều khiển keyboard
```bash
rosrun drone_project keyboard_control.py
```

### 6.4. Đọc sensors
```bash
rosrun drone_project sensor_reader.py
```

## 7. PHÁT TRIỂN TIẾP THEO

- [ ] Thêm SLAM (Simultaneous Localization and Mapping)
- [ ] Path planning tự động
- [ ] Obstacle avoidance
- [ ] Multi-drone simulation
- [ ] Mission planning GUIS
