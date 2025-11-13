# Ros2_Robot_Model_With_Sensors
This ROS2 package my_robot contains a complete robot model with integrated sensors and plugins. It includes CAD .stl files, a detailed .xacro robot description with differential drive controller, odometry, IMU, joint state, and Gazebo sensor plugins for RGB-D camera, 2D/3D LiDAR, and IMU, plus friction coefficients for simulation realism.

<img width="1291" height="745" alt="Screenshot from 2025-11-13 11-41-10" src="https://github.com/user-attachments/assets/5258858f-9dd4-4260-bc53-f6be22275658" />
<img width="1291" height="745" alt="Screenshot from 2025-11-13 11-40-36" src="https://github.com/user-attachments/assets/016ca12d-c97f-4550-9b09-5d4de2448a02" />

---

## 🏗️ Package Structure

```
├── my_bot/
  ├── setup.py
  ├── setup.cfg
  ├── package.xml
  ├── my__bot/
  │ └── init.py
  ├── resource/
  │ └── my_bot
  ├── launch/
  │ ├── gazebo.launch.py              # launch file gazebo
  │ └── rviz.launch.py		            # launch file rviz2
  ├── worlds/
  │ ├── bigmap.sdf
  │ ├── short.sdf
  │ ├── tugbot_depot.sdf
  │ └── map.sdf
  ├── models/
  │ ├── meshes
  │ 	├── base_plate.stl
  │ 	├── left_wheel.stl
  │ 	├── right_wheel.stl
  │ 	└── lidar.stl
  │ └── urdf
  │		 └── my_bot.xacro              # robot xacro
```

---

## Overview

- **Robot Description:** Defined in `.xacro` with modular structure and linked `.stl` CAD meshes.  
- **Controllers and Plugins:** Includes differential drive, odometry, IMU, and joint state publisher plugins for smooth robot motion and state updates.  
- **Sensor Integration:** Integrated RGB-D camera, 2D LiDAR, 3D LiDAR, and IMU sensors for realistic perception.  
- **Physical Properties:** Includes coefficients of friction, mass, and inertia values for accurate physical simulation.  
- **Simulation Maps:** Contains pre-built **small**, **big**, and **warehouse** environments for spawning and testing robot behavior.  
- **Gazebo Bridging:** Uses **`gz_sim_bridge`** to activate and connect ROS2 topics with Gazebo topics for seamless data exchange.  

---

## Features

- Full **URDF/XACRO** robot description with imported CAD meshes.  
- Differential drive controller for robot base control.  
- Real-time **odometry**, **IMU**, and **joint state** publishing.  
- Sensor simulation for **RGB-D camera**, **2D LiDAR**, **3D LiDAR**, and **IMU** in Gazebo.  
- Adjustable parameters for materials, friction, and sensor placement.  
- Supports **multi-environment testing** with included Gazebo maps.  
- **`gz_sim_bridge`** integration ensures ROS2 ↔ Gazebo communication.  

---

## 🔧 Sensors Overview

This robot integrates multiple sensors defined in the `.xacro` file, each simulated through Gazebo plugins for realistic perception and control.

---

### ⚙️ Odometry Plugin

**Plugin:** `diff_drive_controller` / `odom_publisher`  
- Responsible for generating **odometry data** based on the robot’s wheel joint states and movement.  
- Publishes linear and angular velocity along with robot position on the `/odom` topic.  
- Integrates with the **TF tree** to maintain continuous transformation between `odom` → `base_link`.  
- Works together with the **Differential Drive Plugin** to ensure accurate pose estimation.  
- Essential for **navigation**, **path tracking**, and **SLAM**.  
- Parameters typically include wheel separation, wheel radius, and update frequency for motion accuracy.

*Example Simulation:*  
<img width="905" height="742" alt="odom" src="https://github.com/user-attachments/assets/3cc0fb82-d90b-46b1-94a7-1434501f35d2" />

---

### 📷 RGB-D Camera (Depth Camera)

**Plugin:** `rgbd_camera`  
- Simulates a depth-sensing camera that provides both **RGB** and **depth** data streams.  
- Useful for **3D vision**, **object detection**, and **mapping** tasks.  
- Publishes image data to the topic `/depth_camera`.  
- Visual output can be viewed in **RViz** or **Gazebo GUI**.  
- Parameters include field of view (`1.047 rad`), resolution (`640x480`), and depth range (`0.05–3 m`).

*Example Simulation:*  
<img width="1289" height="742" alt="cam1" src="https://github.com/user-attachments/assets/51f81a73-c24f-48d5-a8bf-8e951475abfa" />
<img width="1291" height="745" alt="cam2" src="https://github.com/user-attachments/assets/f4ecbbed-c640-400f-8c67-7bd5881b39bd" />

---

### 🔦 2D LiDAR Sensor

**Plugin:** `gpu_lidar`  
- Provides **360° planar laser scans** for SLAM and obstacle detection.  
- Publishes scan data to the topic `/scan`.  
- Operates at **30 Hz** with a detection range of `0.2–10 m`.  
- Simulates a traditional 2D LiDAR like **RPLidar** or **Hokuyo**.  
- Configured for horizontal scanning with 360 samples.

*Example Simulation:*  
<img width="1289" height="742" alt="2dlidar1" src="https://github.com/user-attachments/assets/b36267d2-b859-4b05-ae38-12e74f2388b8" />
<img width="905" height="742" alt="2dlidar2" src="https://github.com/user-attachments/assets/793e9e1a-67a1-4e9d-acea-7def69756244" />
<img width="1289" height="742" alt="2dlidar3" src="https://github.com/user-attachments/assets/1396b07c-3f38-4a26-8d13-d14cfe00c5ec" />

---

### 🌀 3D LiDAR Sensor

**Plugin:** `gpu_ray`  
- Simulates a **3D LiDAR** that captures a volumetric point cloud of the environment.  
- Provides both horizontal and vertical scanning (16 vertical layers).  
- Publishes point cloud data to the topic `/lidar`.  
- Useful for **3D mapping**, **environment reconstruction**, and **object detection**.  
- Range: `0.2–30 m`, with 360° horizontal and ±15° vertical FOV.

*Example Simulation:*  
<img width="1289" height="742" alt="3dlidar1" src="https://github.com/user-attachments/assets/75714c4c-018b-4dcc-948e-650c6b5560fc" />
<img width="905" height="742" alt="3dlidar2" src="https://github.com/user-attachments/assets/4bb92a26-6fab-4b65-8f6b-35b4c4b019dd" />
<img width="1289" height="742" alt="3dlidar3" src="https://github.com/user-attachments/assets/15377b56-d404-4b7b-9a80-8f72bf53560e" />
<img width="905" height="742" alt="3dlidar4" src="https://github.com/user-attachments/assets/93aaee71-5aea-4e3e-b3d4-49f71dd081cb" />

---

### 🧭 IMU Sensor

**Plugin:** `imu`  
- Simulates an **Inertial Measurement Unit** that provides orientation, angular velocity, and linear acceleration.  
- Publishes data to the topic `/imu`.  
- Used for **localization**, **motion tracking**, and **sensor fusion** with odometry.  
- Attached to the robot’s `base_link` for accurate movement estimation.  
- Update rate: **20 Hz** with visualization enabled.

*Example Simulation:*  
<img width="905" height="742" alt="imu" src="https://github.com/user-attachments/assets/befb462f-7bad-4d6b-95c6-639c7c215e52" />

---

## ⚙️ Summary of Sensor Topics

| Sensor Type | Plugin Name | Topic | Data Type | Use Case |
|--------------|-------------|--------|------------|-----------|
| RGB-D Camera | `rgbd_camera` | `/depth_camera` | `Image / PointCloud2` | Vision, depth perception |
| 2D LiDAR | `gpu_lidar` | `/scan` | `LaserScan` | SLAM, obstacle detection |
| 3D LiDAR | `gpu_ray` | `/lidar` | `PointCloud2` | 3D mapping, perception |
| IMU | `imu` | `/imu` | `Imu` | Orientation, motion tracking |
| Odometry | `diff_drive_controller` / `odom_publisher` | `/odom` | `nav_msgs/Odometry` | Publishes robot’s position, linear & angular velocity for navigation and SLAM |

---

## 🚀 How to Run

### Change Workspace
```bash
cd ros_ws/src
```

### Clone Repository
```bash
git clone https://github.com/yashbhaskar/Ros2_Robot_Model_With_Sensors.git
```

### Change Workspace
```bash
cd ..
```

### Build The Package
```bash
colcon build --packages-select my_bot
source install/setup.bash
```

### Spawn Robot In Ignition Gazebo And Rviz2 
```bash
ros2 launch my_bot gazebo.launch.py
```

### Launch Rviz If You Want To Visualize Your Xacro/Urdf File In Rviz2 (Optional)
```bash
ros2 launch my_bot rviz.launch.py
```

---

## ✉️ Contact

📧 Yash Bhaskar – ybbhaskar19@gmail.com

📌 GitHub: https://github.com/yashbhaskar

