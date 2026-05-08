# Multi-Robot Semantic Mapping in ROS 2 & Gazebo Harmonic

## 🎯 Project Objective
This project demonstrates a multi-robot system for real-time object detection and global semantic map fusion. Two differential drive robots explore a Gazebo simulation environment, identifying objects using YOLO and merging their local detections into a unified, coordinate-based global semantic map.

## 🏗️ System Architecture
- **Robots:** 2 × Differential drive robots equipped with RGB-D cameras.
- **Simulation:** Gazebo Harmonic using the `ros_gz_bridge` for clock and topic synchronization.
- **Namespacing:** Full isolation between `/robot1` and `/robot2` namespaces.
- **Perception:** YOLO-based object detection (2D) projected into 3D metric space using depth data.
- **Mapping:** Object-based semantic mapping where detections are transformed via odometry into a common `map` frame.
- **Fusion:** A centralized offline code that clusters and merges overlapping detections from both robots.

## 🚀 Key Features
- **Localization:** Uses the odometry and camera coordinates to achieve global coordinates without dense point cloud overhead.
- **Independent Multi-Robot System:** Independent state-machine controllers for autonomous exploration.
- **Synchronized Multi-Robot Bridge:** A custom-configured ros_gz_bridge that solves the common "silent clock" issue by dynamically mapping /world/<world_name>/clock to the ROS 2 /clock topic.
## 🛠️ Installation & Setup

### Setup
The current setup has been built using: 
- ROS 2 Jazzy
- Gazebo Harmonic
- YOLO26n

### Build
Place the package folder inside the `src` folder in the ROS workspace. The merger codes are available in the `WorkspaceFiles` folder in the package. 
```bash
cd ~/ros2_ws
colcon build --packages-select multi_robot_project --symlink-install
source install/setup.bash
