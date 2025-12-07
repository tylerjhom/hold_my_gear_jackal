# Hold My Gear – Jackal Firefighter Following Robot

Follow-me Clearpath Jackal robot that carries gear for Austin Fire Department’s Air Shop crew.

The robot uses an Intel RealSense depth camera and YOLOv11 for person detection, with two main operating modes:

1. **Nav2-based follower** – YOLO publishes waypoints to Nav2, which handles mapping, path planning, and obstacle avoidance.
2. **Direct velocity follower** – YOLO computes a relative target and publishes `/cmd_vel` directly (no obstacle avoidance).

This project was developed for **ME396P – Application Programming for Engineers** at UT Austin.

---

## 1. Repository Layout

```text
hold-my-gear-jackal/
├── README.md
├── requirements.txt
├── docs/
│   ├── 02_robot_setup.md
│   └── 03_offboard_setup.md
├── robot/
│   ├── config/
│   │   └── robot.yaml
│   └── jackal_yolo_follow/
│       ├── package.xml
│       ├── setup.py, setup.cfg
│       ├── resource/
│       ├── jackal_yolo_follow/
│       │   ├── yolo_nav2_follower*.py
│       │   ├── nav_to_pose_test*.py
│       │   ├── yolo_follower.py
│       │   └── ...
│       ├── launch/
│       │   ├── yolo_nav2_follow.launch.py
│       │   └── yolo_direct_follow.launch.py
│       └── experimental/
└── offboard/
    ├── rviz/
    │   └── jackal_follow.rviz
    ├── config/
    │   └── cyclonedds.xml
    └── scripts/
        └── start_offboard_viz.sh
````

* **`robot/`** – Everything that runs on the Jackal’s onboard computer.
* **`offboard/`** – RViz configs and any tools that run on the offboard laptop.
* **`docs/`** – Detailed setup guides for the robot and offboard machines.

---

## 2. System Overview

High-level architecture:

* **Sensors:** Intel RealSense D435 depth camera mounted on the Jackal.
* **Perception:** YOLOv11-based detector (Ultralytics) running on the Jackal; detects the target firefighter in RGB images.
* **Mapping:** SLAM Toolbox builds a map from the RealSense depth data and odometry.
* **Navigation:** Nav2 plans collision-free paths to the firefighter’s current pose.
* **Visualization:** RViz2 on the offboard laptop shows the map, robot pose, and camera feeds.

Two Python-based approaches are implemented:

1. `yolo_nav2_follower.py` + `nav_to_pose_test.py`

   * YOLO node publishes `geometry_msgs/PoseStamped` to a `goal_pose` topic.
   * Nav2 node listens and forwards these as Nav2 goals using `nav2_simple_commander.BasicNavigator`.

2. `yolo_follower.py`

   * YOLO node directly computes linear and angular velocity commands based on target position and sends `/cmd_vel` to the Jackal’s base.

---

## 3. Requirements

### Hardware

* Clearpath Jackal J100 (ROS 2 Jazzy image).
* Intel RealSense D435 (or similar) depth camera.
* Offboard laptop:

  * Ubuntu **24.04** Desktop.
  * Wi-Fi or Ethernet connection to the Jackal.

### Software

* **Robot (Jackal onboard PC)**

  * ROS 2 Jazzy (Clearpath image).
  * Nav2, SLAM Toolbox (installed via Clearpath debs).
  * `ros-jazzy-realsense2-camera` or equivalent RealSense driver.
  * YOLO dependencies (see `requirements.txt`).

* **Offboard laptop**

  * ROS 2 Jazzy (binary install).
  * Clearpath Desktop metapackage (`ros-jazzy-clearpath-desktop`).
  * RViz2.

See `docs/02_robot_setup.md` and `docs/03_offboard_setup.md` for installation steps.

---

## 4. Quick Start (Very High Level)

### 4.1 Robot (onboard PC)

1. **Clone repo into a workspace**

   ```bash
   cd ~/clearpath_ws/src    # or another colcon workspace
   git clone https://github.com/<your-org>/hold-my-gear-jackal.git
   ln -s hold-my-gear-jackal/robot/jackal_yolo_follow .
   ```

2. **Install dependencies and build**

   ```bash
   cd ~/clearpath_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

3. **Ensure Clearpath robot configuration uses the provided robot.yaml**

   * Copy or symlink `robot/config/robot.yaml` to `/etc/clearpath/robot.yaml`.
   * Regenerate Clearpath setup if needed (see docs).

4. **Launch RealSense + SLAM + Nav2**
   (Either with Clearpath’s provided launch files or your own; documented in `docs/02_robot_setup.md`.)

5. **Launch a follower method**

   * **Method 1 – Nav2-based follower (recommended)**

     ```bash
     ros2 launch jackal_yolo_follow yolo_nav2_follow.launch.py
     ```

   * **Method 2 – Direct `/cmd_vel` follower (no obstacle avoidance)**

     ```bash
     ros2 launch jackal_yolo_follow yolo_direct_follow.launch.py
     ```

---

### 4.2 Offboard Laptop

1. Follow Clearpath’s “Offboard Computer Setup” for Jazzy (robot.yaml mirror, Clearpath Desktop, etc.).

2. Clone the same repo (for configs/RViz):

   ```bash
   cd ~/jackal_follow_ws/src
   git clone https://github.com/<your-org>/hold-my-gear-jackal.git
   ```

3. Build if you plan to run any offboard nodes:

   ```bash
   cd ~/jackal_follow_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

4. Start RViz with the provided configuration:

   ```bash
   rviz2 -d ~/jackal_follow_ws/src/hold-my-gear-jackal/offboard/rviz/jackal_follow.rviz
   ```

---

## 5. Safety Notes

* Always have an E-stop ready.
* The **direct follower (`yolo_follower.py`) does not perform obstacle avoidance**; use only in open environments and low speeds.
* The Nav2-based follower depends on good SLAM and localization; verify map and robot pose in RViz before enabling autonomous following.

---

## 6. License and Credits

* Developed by **Tyler Hom & Kate Whitmire**, UT Austin – ME396P Application Programming for Engineers.
* Built on Clearpath Robotics Jackal, Nav2, SLAM Toolbox, Intel RealSense, and Ultralytics YOLOv11.

````

