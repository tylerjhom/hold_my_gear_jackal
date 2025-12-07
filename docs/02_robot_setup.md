# Robot Setup (Onboard Jackal PC)

This guide describes how to configure the Clearpath Jackal onboard computer to run the Hold My Gear follower system. It assumes a Clearpath Jackal J100 running the ROS 2 Jazzy developer image.

---

## 1. Base System Assumptions

The Jackal image is assumed to provide:

- ROS 2 Jazzy
- Clearpath platform services
- Nav2 and SLAM Toolbox
- A workspace at `~/clearpath_ws`

If your image differs, adapt the paths accordingly.

---

## 2. Install System Dependencies

1. Update `apt` and `rosdep`:

   ```bash
   sudo apt update
   rosdep update

2. Install RealSense driver (if not already installed):

   ```bash
   sudo apt install ros-jazzy-realsense2-camera
   ```

3. Create and configure a Python virtual environment for YOLO:

   ```bash
   cd ~
   python3 -m venv vision_venv
   source vision_venv/bin/activate
   pip install --upgrade pip
   pip install -r ~/clearpath_ws/src/hold_my_gear_jackal/requirements.txt
   ```

You can place `vision_venv` elsewhere; update paths accordingly.

---

## 3. Clone the Project and Add the Package

1. Navigate to the Clearpath workspace source folder:

   ```bash
   cd ~/clearpath_ws/src
   ```

2. Clone the repository:

   ```bash
   git clone https://github.com/tylerjhom/hold_my_gear_jackal.git
   ```

3. Expose the `jackal_yolo_follow` package to the workspace (copy or symlink):

   ```bash
   cp -r hold_my_gear_jackal/robot/packages/jackal_yolo_follow .
   # or:
   # ln -s hold_my_gear_jackal/robot/packages/jackal_yolo_follow .
   ```

---

## 4. Use the Provided `robot.yaml`

1. Copy the project’s `robot.yaml` into Clearpath’s configuration location:

   ```bash
   sudo cp ~/hold_my_gear_jackal/robot/config/robot.yaml /etc/clearpath/robot.yaml
   ```

2. If necessary, regenerate the Clearpath `setup.bash`:

   ```bash
   source /opt/ros/jazzy/setup.bash
   sudo ros2 run clearpath_generator_common generate_bash -s /etc/clearpath
   ```

Many images will already be configured; only regenerate if you have modified the robot configuration.

---

## 5. Build the Workspace

1. Install dependencies and build:

   ```bash
   cd ~/clearpath_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

2. Source the workspace and add it to `.bashrc` if desired:

   ```bash
   echo "source ~/clearpath_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

---

## 6. Launching the Navigation Stack

This section describes the sequence used during the demo to bring up camera, mapping, and navigation.

### 6.1 Launch the RealSense Camera

On the robot:

```bash
ros2 launch realsense2_camera rs_launch.py
```

Verify that the color and depth topics are present (for example):

* `/camera/camera/color/image_raw`
* `/camera/camera/depth/image_rect_raw`

The exact topic names may differ depending on camera configuration.

---

### 6.2 Depth Image to Laser Scan Conversion

Run the `depthimage_to_laserscan` node to convert the depth image into a 2D scan compatible with SLAM Toolbox:

```bash
ros2 run depthimage_to_laserscan depthimage_to_laserscan_node \
  --ros-args \
  -r depth:=/camera/camera/depth/image_rect_raw \
  -r depth_camera_info:=/camera/camera/depth/camera_info \
  -r scan:=/j100_0000/sensors/lidar2d_0/scan \
  -p range_min:=0.3 \
  -p range_max:=5.0 \
  -p output_frame:=base_link
```

This command:

* Takes depth data from the RealSense camera.
* Publishes a `sensor_msgs/LaserScan` to the `/j100_0000/sensors/lidar2d_0/scan` topic.
* Sets the frame to `base_link` so that SLAM Toolbox and Nav2 treat the scan as if it came from a planar lidar mounted at the robot base.

---

### 6.3 Launch SLAM

Start SLAM Toolbox using Clearpath’s demo:

```bash
ros2 launch clearpath_nav2_demos slam.launch.py
```

---

### 6.4 Launch Nav2

Start Nav2 with Clearpath’s demo launch file:

```bash
ros2 launch clearpath_nav2_demos nav2.launch.py
```

Verify that the map topic is publishing:

```bash
ros2 topic hz /j100_0000/map
```

---

## 7. Running the Follower Nodes

Before running the follower nodes, activate the YOLO virtual environment:

```bash
source ~/vision_venv/bin/activate
```

### 7.1 Method 1 – Nav2-Based Follower (Recommended)

Run the Nav2 follower node:

```bash
ros2 run jackal_yolo_follow yolo_nav2_follower
```

You may also run the Nav2 test node independently:

```bash
ros2 run jackal_yolo_follow nav_to_pose_test
```

Conceptually, the two nodes perform:

* `yolo_nav2_follower`

  * Subscribes to RGB (and depth) images.
  * Runs YOLO.
  * Publishes `geometry_msgs/PoseStamped` goals on a dedicated topic.

* `nav_to_pose_test`

  * Subscribes to the goal topic.
  * Uses `nav2_simple_commander.BasicNavigator` to send goals to Nav2.
  * Handles goal updates and cancellations.

In practice, you may wrap these nodes into a single launch file for convenience.

---

### 7.2 Method 2 – Direct Velocity Follower

Run the direct follower:

```bash
ros2 run jackal_yolo_follow yolo_follower
```

This node:

* Subscribes to camera topics.
* Computes linear and angular velocities based on the target position.
* Publishes directly to `/cmd_vel`.

This mode does not use SLAM or Nav2 and does not perform obstacle avoidance. Use only in controlled environments and at low speeds.

---

## 8. Stopping the System

* Use `Ctrl+C` in each terminal to stop nodes.
* Use the physical Jackal E-stop for emergency stop.
* Shut down platform services using your standard Clearpath procedures if required.

````
