# Terrain Mapping Drone Control
### Autonomous UAV Landing & 3D Environment Mapping in Gazebo Harmonic + ROS 2 Jazzy

---

## Overview

This project implements an autonomous drone mission in a simulated Gazebo Harmonic environment using PX4 SITL and ROS 2 Jazzy. The system performs two primary tasks:

1. **Autonomous Landing** — The drone detects an ArUco marker mounted on a Mars rover model and autonomously lands on it using computer vision and PX4 offboard control.
2. **3D Environment Mapping** — During a spiral descent trajectory, the drone captures RGB-D data and constructs a 3D point cloud of the environment using RTAB-Map SLAM and Open3D.

---

## Repository Structure

```
terrain_mapping_drone_control/
├── launch/
│   ├── rover_world.launch.py          # Gazebo world + PX4 SITL + bridge
│   ├── rover_mission.launch.py        # ArUco detection + landing mission
│   ├── rtabmap_jazzy_visual.launch.py # RTAB-Map with visual odometry
│   ├── rtabmap_jazzy_droneOdo.launch.py # RTAB-Map with vehicle odometry
│   └── master_launch.py               # Master script (all-in-one)
├── terrain_mapping_drone_control/
│   ├── aruco_tracker.py               # ArUco marker detection node
│   ├── rover_detect_land.py           # Autonomous landing controller
│   ├── spiral_trajectory.py           # Spiral descent trajectory node
│   ├── px4_odom_converter.py          # PX4 NED→ENU odometry converter
│   ├── geometry_tracker.py            # Depth-based geometry tracker
│   ├── bag_to_pointcloud.py           # Offline point cloud generation
│   └── pose_visualizer.py             # Drone pose visualization
├── models/
│   ├── rover/                         # Mars rover SDF model with ArUco
│   ├── cylinder/                      # Tall cylinder obstacle
│   └── cylinder_short/                # Short cylinder obstacle
└── README.md
```

---

## Simulation Environment

The Gazebo world contains:
- **x500_depth_mono drone** — PX4 quadrotor with OakD-Lite RGB-D camera (30° downward tilt) and downward mono camera
- **Mars Rover** — Static model with ArUco marker (ID 0) on top, placed at (5, 10, 0)
- **Two cylinders** — Obstacles at (5, 0, 0) and (-5, 0, 0) used for visual odometry

---

## Running the Project

### Step 1 — Source the environment

Open a terminal and source the workspace in every new terminal:

```bash
source ~/ros2_ws/rosenv.sh
```

### Step 2 — Launch the Gazebo world

Run the MicroXRCEAgent using the command:
```bash
MicroXRCEAgent udp4 -p 8888
```
And run the Ground control station QGroundControl. After this, launch the world file in gazebo.
```bash
ros2 launch terrain_mapping_drone_control rover_world.launch.py
```

Wait ~20 seconds for PX4 SITL and Gazebo to fully initialize before proceeding.

### Step 3 — Launch the drone mission

```bash
ros2 launch terrain_mapping_drone_control rover_mission.launch.py
```

The drone will arm, take off, search for the rover, circle around the rover, search for ArUco marker on the rover, and autonomously land on it.

### Step 4 — Launch the PX4 odometry converter

```bash
python3 ~/ros2_ws/src/terrain_mapping_drone_control/terrain_mapping_drone_control/px4_odom_converter.py
```

This converts PX4's native `VehicleOdometry` (NED frame, px4_msgs) to standard `nav_msgs/Odometry` (ENU frame) with simulation timestamps synchronized to the Gazebo clock.

### Step 5 — Launch RTAB-Map (choose one)

**Option A — Vehicle odometry (recommended):**
```bash
ros2 launch terrain_mapping_drone_control rtabmap_jazzy_droneOdo.launch.py
```

**Option B — Visual odometry (fallback):**
```bash
ros2 launch terrain_mapping_drone_control rtabmap_jazzy_visual.launch.py
```

### Step 6 — Record mission data (optional)

To record a rosbag for offline processing:

```bash
ros2 bag record \
  /drone/front_rgb \
  /drone/front_depth \
  /drone/front_rgb/camera_info \
  /drone/front_depth/camera_info \
  /drone/odom \
  /tf /tf_static \
  --storage mcap \
  -o ~/ros2_ws/mission_bag
```

Stop recording with `Ctrl+C` **after** the mission completes.

### Step 7 — Generate offline point cloud

After recording, process the bag with Open3D:

```bash
python3 ~/ros2_ws/src/terrain_mapping_drone_control/terrain_mapping_drone_control/bag_to_pointcloud.py
```

The output point cloud is saved to `~/ros2_ws/mission_map.ply`.

---

## Autonomous FLight and Landing

### How it works

1. The drone takes off to a search altitude using PX4 offboard control mode
2. The drone yaws and looks for the rover. 
3. If the rover is found, the drone aligns itself and moves towards the rover.
4. After reaching the desired distance the drone circles around the rover.
5. `aruco_tracker.py` subscribes to `/drone/down_mono` and detects ArUco marker ID 0 using OpenCV
6. When the marker is detected, the drone's position is adjusted to center over it
7. The drone descends and lands on the rover

### Key nodes

| Node | Topic | Description |
|------|-------|-------------|
| `aruco_tracker` | `/drone/down_mono` | Detects ArUco marker from downward camera |
| `rover_detect_land` | `/fmu/in/trajectory_setpoint` | Controls drone position via PX4 offboard |



---

## 3D Mapping

### How it works

1. The front RGB-D camera captures the environment at ~5Hz
2. `px4_odom_converter.py` converts PX4 odometry to ROS standard format with correct NED→ENU frame conversion and simulation timestamps
3. RTAB-Map fuses RGB-D frames with odometry to build an incremental 3D map
4. The map is exported as a `.ply` point cloud


### RTAB-Map Configuration

Key parameters for drone RGB-D mapping:

```python
'subscribe_rgbd': True          # Use pre-synced RGBD topic
'approx_sync': True             # Allow timestamp approximation
'approx_sync_max_interval': 0.5 # 500ms sync window
'RGBD/LinearUpdate': '0.05'     # Add node every 5cm movement
'RGBD/AngularUpdate': '0.02'    # Add node every ~1° rotation
'visual_odometry': False        # Use external odometry
```


### Known Issues & Limitations

- **Sim time synchronization** — The Gazebo clock does not flow through `/clock` during early startup. The `px4_odom_converter` uses camera info timestamps as a proxy for simulation time.
- **Featureless environment** — The flat Gazebo terrain provides insufficient texture for visual odometry. Vehicle odometry is required for reliable mapping.
- **Camera coverage** — The front-facing camera only captures a partial view of the environment during the spiral. A downward-facing camera would provide better ground coverage.
- **ROS 2 discovery delay** — FastDDS node discovery can take 20-30 seconds. Always wait for topics to appear before checking node status.

---

## Offline Point Cloud Generation

The `bag_to_pointcloud.py` script processes a recorded mission bag and generates a fused point cloud:

```python
# Camera extrinsic: OpenCV optical frame → ROS body frame
OPTICAL_TO_BODY = np.array([
    [ 0,  0,  1,  0],   # body_X = cam_Z (depth/forward)
    [-1,  0,  0,  0],   # body_Y = -cam_X (left)
    [ 0, -1,  0,  0],   # body_Z = -cam_Y (up)
    [ 0,  0,  0,  1]
])
```

**Configuration** (top of script):
```python
MIN_DEPTH  = 0.5    # metres
MAX_DEPTH  = 18.0   # metres (just under 19.1m sensor far clip)
FRAME_SKIP = 1      # process every frame
VOXEL_SIZE = 0.05   # metres, final point cloud resolution
```

---

## Export 3D Map from RTAB-Map

After a mapping session, export the accumulated point cloud:

```bash
cd ~/.ros
rtabmap-export --cloud --voxel 0.02 --output mission_map rtabmap.db
```

View in CloudCompare:
```bash
sudo apt install cloudcompare
cloudcompare ~/.ros/mission_map_cloud.ply
```

Or view online at [3dviewer.net](https://3dviewer.net) (drag and drop the `.ply` file).

---

## Demonstration Video

A full demonstration video showing the autonomous landing mission and 3D mapping pipeline is included in the repository.

---

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| `ros2 topic list` shows nothing | FastDDS discovery delay | Wait 30s after sourcing |
| `/drone/odom` not publishing | `px4_odom_converter` not receiving camera info | Start world before converter |
| rtabmap viz blank | TF `odom→base_link` not available | Restart rtabmap after odom is flowing |
| `rtabmap.db` missing after session | Process killed before database flush | Always use `Ctrl+C`, wait for "Saving memory... done" |
| Depth min ~9m | Drone too far from objects at bag start | Normal — objects 9-15m from camera during spiral |
|Messy Point Cloud | Uneven Odometry or transforms | Unknown| 

---

