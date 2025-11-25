# DRONE SIMULATION - WORKFLOW

## 1. WORKFLOW TỔNG QUAN
START
│
├─> Khởi động Gazebo
│     │
│     ├─> Map Key to Twist Command
│     │     ├─> 'W' → linear.x = 0.5
│     │     ├─> 'S' → linear.x = -0.5
│     │     ├─> 'A' → linear.y = 0.5
│     │     ├─> 'D' → linear.y = -0.5
│     │     ├─> 'R' → linear.z = 0.3
│     │     ├─> 'F' → linear.z = -0.3
│     │     ├─> 'Q' → angular.z = 0.5
│     │     ├─> 'E' → angular.z = -0.5
│     │     └─> 'SPACE' → all = 0
│     │
│     ├─> Publish Twist to /cmd_vel
│     │
│     └─> Loop back
│
└─> Exit on 'X' key

## 3. DATA FLOW
Gazebo Physics Engine
│
├─> Simulates Drone Dynamics
│     │
│     ├─> Applies Forces from cmd_vel
│     ├─> Calculates Position/Orientation
│     └─> Updates Visual/Collision models
│
├─> Sensors Generate Data
│     │
│     ├─> IMU Plugin
│     │    └─> Publishes to /imu
│     │
│     ├─> Camera Plugin
│     │    └─> Publishes to /camera/image_raw
│     │
│     └─> Ground Truth
│          └─> Publishes to /ground_truth/state
│
└─> ROS Nodes Subscribe
│
├─> drone_controller reads state
├─> sensor_reader reads all sensors
└─> Display/Log data

## 4. TIMELINE - TUẦN 2

### Day 1-2: Setup
- Cài đặt ROS + Gazebo
- Cài đặt Hector/PX4
- Test mô hình có sẵn

### Day 3-4: Development
- Người 1: URDF modeling
- Người 2: World creation
- Người 3: ROS nodes
- Người 4,5: Documentation

### Day 5-6: Integration
- Tích hợp tất cả components
- Testing và debugging
- Tạo launch files

### Day 7: Demo
- Chạy simulation hoàn chỉnh
- Record video demo
- Presentation

## 5. TESTING CHECKLIST

- [ ] Gazebo mở được không?
- [ ] Drone model hiển thị đúng không?
- [ ] World load đúng không?
- [ ] Topics publish/subscribe hoạt động?
- [ ] IMU data có đọc được?
- [ ] Camera image hiển thị được?
- [ ] Keyboard control hoạt động?
- [ ] Autonomous control hoạt động?
- [ ] Drone bay đúng trajectory không?
- [ ] Launch file chạy được?

## 6. TROUBLESHOOTING

### Problem: Gazebo không khởi động
```bash
killall gzserver gzclient
gazebo --verbose
```

### Problem: Drone không bay
```bash
# Kiểm tra topics
rostopic list
rostopic echo /cmd_vel

# Kiểm tra plugins
rosrun rqt_graph rqt_graph
```

### Problem: Camera không hiển thị
```bash
# Kiểm tra topic
rostopic list | grep camera
rostopic hz /quadrotor/camera/image_raw

# Test với image_view
rosrun image_view image_view image:=/quadrotor/camera/image_raw
```
