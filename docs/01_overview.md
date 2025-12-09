# Hold My Gear – Jackal Firefighter Following Robot

Autonomous follow-me Clearpath Jackal robot developed for the Austin Fire Department Air Shop to carry heavy oxygen tanks.  
Built for **ME396P – Application Programming for Engineers**, UT Austin.

The robot uses an Intel RealSense depth camera and YOLOv11 running on the Jackal onboard computer to track a specific firefighter and follow them in real time.

Two operating modes are provided:

1. **Nav2-based follower (recommended)**  
   YOLO detects the firefighter, computes a waypoint, and Nav2 handles mapping, planning, and obstacle avoidance.

2. **Direct velocity follower (simple / no path planning)**  
   YOLO computes relative position and publishes `/cmd_vel` directly.

---

## 1. Repository Layout

```text
hold_my_gear_jackal/
├── requirements.txt
│
├── docs/
│   ├── 01_overview.md
│   ├── 02_robot_setup.md
│   └── 03_offboard_setup.md
│
├── robot/
│   ├── config/
│   │   └── robot.yaml
│   │
│   └── packages/
│       └── jackal_yolo_follow/
│           ├── package.xml
│           ├── setup.py
│           ├── setup.cfg
│           ├── resource/
│           └── jackal_yolo_follow/
│               ├── yolo_nav2_follower.py
│               ├── nav_to_pose_test.py
│               └── yolo_follower.py
│
└── offboard/
    ├── config/
        ├── cyclonedds.xml
        ├── robot.yaml
        ├── robot.urdf.xacro
        ├── robot.srdf
        ├── robot.srdf.xacro
        ├── platform/
        ├── sensors/
        ├── manipulators/
        └── platform-extras/
    
````

### Main Folders

* `robot/` – Everything that runs on the Jackal onboard PC.
* `offboard/` – Visualization and configuration files used on the offboard laptop.
* `docs/` – Detailed setup documentation for both machines.

---

## 2. System Overview

### Sensors

* Intel RealSense D435 depth + RGB camera
* Jackal IMU and wheel odometry

### Perception

* YOLOv11 (Ultralytics) detects a single firefighter in RGB images.
* Bounding box and depth are used to estimate a relative target pose.

### Mapping and Localization

* SLAM Toolbox generates a 2D map.
* Nav2 uses SLAM or AMCL for localization.

### Navigation

Two independent approaches are implemented.

---

### Approach 1: Nav2-Based Follower

**Files:** `yolo_nav2_follower.py`, `nav_to_pose_test.py`

Pipeline:

1. YOLO detects the firefighter in the RGB image.
2. Depth information is used to estimate the 3D position of the target relative to the robot.
3. The estimate is converted to a global pose and published as a `geometry_msgs/PoseStamped` goal.
4. `nav_to_pose_test.py` or an equivalent node forwards goals to Nav2 via `nav2_simple_commander.BasicNavigator`.
5. Nav2 plans and executes a safe path to the target pose, handling obstacle avoidance.

This is the recommended operational mode.

---

### Approach 2: Direct Velocity Follower

**File:** `yolo_follower.py`

Pipeline:

1. YOLO keeps the firefighter centered in the image.

2. Angular velocity is computed from the horizontal offset:

   ```text
   angular_z = k_ang * x_offset
   ```

3. Forward velocity is computed from distance error:

   ```text
   linear_x = k_lin * (desired_distance - current_distance)
   ```

4. Commands are published directly on `/cmd_vel`.

This mode does not use Nav2 or SLAM and does not perform obstacle avoidance. It is intended for testing in open, controlled environments at low speeds.

---

## 3. Requirements

### Hardware

* Clearpath Jackal J100 (ROS 2 Jazzy image)
* Intel RealSense D435 or D455
* Offboard laptop with:

  * Ubuntu 24.04 Desktop
  * ROS 2 Jazzy Desktop

### Software

#### Robot (onboard computer)

* ROS 2 Jazzy (Clearpath factory image)
* Nav2 and SLAM Toolbox (Clearpath packages)
* RealSense ROS 2 driver (`realsense2_camera`)
* Python 3.12 and YOLO dependencies (from `requirements.txt`)

#### Offboard laptop

* ROS 2 Jazzy Desktop
* Clearpath Desktop metapackage (`ros-jazzy-clearpath-desktop`)
* Mirrored `robot.yaml` and DDS configuration from the robot
* RViz2

---

## 4. Quick Start Guide

The commands below summarize the demo procedure. See `docs/02_robot_setup.md` and `docs/03_offboard_setup.md` for detailed instructions.

### 4.1 Robot Setup (Onboard PC)

1. **Clone the repo into the Clearpath workspace**

   ```bash
   cd ~/clearpath_ws/src
   git clone https://github.com/tylerjhom/hold_my_gear_jackal.git
   cp -r hold_my_gear_jackal/robot/packages/jackal_yolo_follow .
   ```

2. **Install dependencies and build**

   ```bash
   cd ~/clearpath_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

3. **Update Clearpath configuration (if needed)**

   ```bash
   sudo cp ~/hold_my_gear_jackal/robot/config/robot.yaml /etc/clearpath/robot.yaml
   ```

4. **Launch RealSense and depth-to-laserscan**

   ```bash
   ros2 launch realsense2_camera rs_launch.py

   ros2 run depthimage_to_laserscan depthimage_to_laserscan_node \
     --ros-args \
     -r depth:=/camera/camera/depth/image_rect_raw \
     -r depth_camera_info:=/camera/camera/depth/camera_info \
     -r scan:=/j100_0000/sensors/lidar2d_0/scan \
     -p range_min:=0.3 \
     -p range_max:=5.0 \
     -p output_frame:=base_link
   ```

5. **Launch SLAM and Nav2**

   ```bash
   ros2 launch clearpath_nav2_demos slam.launch.py
   ros2 launch clearpath_nav2_demos nav2.launch.py
   ```

6. **Run follower nodes**

   ```bash
   source ~/vision_venv/bin/activate

   # Nav2-based follower
   ros2 run jackal_yolo_follow yolo_nav2_follower

   # Optional: Nav2 test node
   ros2 run jackal_yolo_follow nav_to_pose_test

   # Alternative direct velocity follower
   ros2 run jackal_yolo_follow yolo_follower
   ```
