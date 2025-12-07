# **Hold My Gear – Jackal Firefighter Following Robot**

Autonomous **follow-me Clearpath Jackal robot** developed for the **Austin Fire Department Air Shop** to carry heavy oxygen tanks.
Built for **ME396P – Application Programming for Engineers**, UT Austin.

The robot uses an Intel RealSense depth camera and YOLOv11 running **on the Jackal onboard computer** to track a specific firefighter and follow them in real time.

Two operating modes are provided:

1. **Nav2-based follower (recommended)**
   YOLO detects the firefighter → sends a waypoint → Nav2 handles mapping, planning, and obstacle avoidance.

2. **Direct velocity follower (simple / no path planning)**
   YOLO computes relative position → publishes `/cmd_vel` directly.

---

# **1. Repository Layout**

```text
hold-my-gear-jackal/
├── README.md
├── requirements.txt
│
├── docs/
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
    └── config/
        ├── cyclonedds.xml
        ├── robot.yaml
        ├── robot.urdf.xacro
        ├── robot.srdf
        ├── robot.srdf.xacro
        ├── platform/
        ├── sensors/
        ├── manipulators/
        └── platform-extras/
```

### Main Folders

* **`robot/`** contains everything that runs on the Jackal onboard PC.
* **`offboard/`** contains visualization & configuration files used on your laptop.
* **`docs/`** explains setup for both machines.

---

# **2. System Overview**

### Sensors

* Intel RealSense D435 depth + RGB camera
* Jackal IMU + wheel odometry

### Perception

* YOLOv11 (Ultralytics) detects a single firefighter in RGB images
* Bounding box → pixel offset → relative target pose

### Mapping & Localization

* SLAM Toolbox generates a map
* Nav2 uses AMCL or SLAM state for localization

### Navigation

Two independent approaches:

---

## **Approach 1: Nav2-Based Follower**

**Files:** `yolo_nav2_follower.py`, `nav_to_pose_test.py`

Pipeline:

1. YOLO detects the target in RGB.
2. Depth measurement to 3D relative target position.
3. Convert to global pose to publish `PoseStamped`.
4. Nav2 receives pose as a goal and plans a safe path.
5. Jackal follows while avoiding obstacles.

This is the **recommended production approach**.

---

## **Approach 2: Direct Velocity Follower**

**File:** `yolo_follower.py`

Pipeline:

1. YOLO centers target in image.
2. Proportional control for angular velocity:

   ```
   angular_z = k * (x_offset)
   ```
3. Depth → forward velocity:

   ```
   linear_x = k * (desired_distance – current_distance)
   ```
4. Publish raw `/cmd_vel`.

⚠️ **No obstacle avoidance.**
Great for testing, not safe indoors or crowded areas.

---

# **3. Requirements**

## **Hardware**

* Clearpath Jackal J100 (ROS 2 Jazzy image)
* Intel RealSense D435 (or D455)
* Offboard laptop with:

  * Ubuntu **24.04**
  * ROS 2 Jazzy Desktop

---

## **Software**

### *Robot (onboard computer)*

* ROS 2 Jazzy (flashed by Clearpath)
* Nav2 (installed via Clearpath packages)
* SLAM Toolbox
* RealSense ROS2 driver (`realsense2_camera`)
* Python3.12 + YOLO dependencies (from `requirements.txt`)

### *Offboard Laptop*

* ROS 2 Jazzy Desktop
* Clearpath Desktop metapackage (`ros-jazzy-clearpath-desktop`)
* Copied `robot.yaml`, URDF, and DDS config from robot
* RViz2

---

# **4. Quick Start Guide**

This section gives a high-level summary.
Full details are in `docs/02_robot_setup.md` and `docs/03_offboard_setup.md`.

---

## **4.1 Robot Setup (Onboard PC)**

### 1. Clone this repo (via HTTPS with Personal Access Token)

```bash
cd ~/clearpath_ws/src
git clone https://github.com/tylerjhom/hold_my_gear_jackal.git
cp -r hold_my_gear_jackal/robot/packages/jackal_yolo_follow .
```

### 2. Build the workspace

```bash
cd ~/clearpath_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 3. Update Clearpath robot.yaml (if needed)

```bash
sudo cp ~/hold_my_gear_jackal/robot/config/robot.yaml /etc/clearpath/robot.yaml
```

(Reboot or regenerate setup if required.)

### 4. Launch RealSense driver

```bash
ros2 launch realsense2_camera rs_launch.py
```

### 5. Launch a follower mode

#### Nav2 follower:

```bash
ros2 run jackal_yolo_follow yolo_nav2_follower
```

(Typically launched within a larger bringup launch file; see docs.)

#### Direct `/cmd_vel` follower:

```bash
ros2 run jackal_yolo_follow yolo_follower
```

---

## **4.2 Offboard Laptop Setup**

### 1. Create or reuse offboard workspace

```bash
cd ~/jackal_follow_ws/src
git clone https://github.com/tylerjhom/hold_my_gear_jackal.git
```

### 2. Build (optional; only if running offboard nodes)

```bash
cd ~/jackal_follow_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### 3. Start RViz

```bash
rviz2 -d ~/jackal_follow_ws/src/hold_my_gear_jackal/offboard/rviz/jackal_follow.rviz
```

---

# **5. Safety Notes**

- ALWAYS have an E-stop accessible.
- Validate Nav2 maps before following indoors.
- Direct follower (`yolo_follower.py`) does **not** avoid obstacles -  use at low speeds only.
- Ensure the RealSense depth stream is reliable before engaging autonomy.

---

# **6. License & Credits**

Developed for the UT Austin course **ME396P – Application Programming for Engineers**
by **Tyler Hom** & **Kate Whitmire**.

Built on:

* Clearpath Jackal ROS 2 ecosystem
* Intel RealSense
* Nav2 + SLAM Toolbox
* Ultralytics YOLOv11

---
