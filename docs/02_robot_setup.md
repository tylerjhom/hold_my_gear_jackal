
# Robot Setup (Onboard Jackal PC)

This guide assumes you are using a Clearpath Jackal J100 running the ROS 2 Jazzy developer image.

## 1. Base System Assumptions

The Jackal image should already include:

- ROS 2 Jazzy
- Clearpath platform services
- Nav2 and SLAM Toolbox
- A pre-created workspace at `~/clearpath_ws`

If your image differs, adapt the workspace paths accordingly.

---

## 2. Install System Dependencies

1. **Update apt and rosdep**

   ```bash
   sudo apt update
   rosdep update


2. **Install RealSense driver (if not already installed)**

   ```bash
   sudo apt install ros-jazzy-realsense2-camera
   ```

3. **Install Python dependencies for YOLO**

   From your home directory:

   ```bash
   cd ~
   python3 -m venv vision_venv
   source vision_venv/bin/activate
   pip install --upgrade pip
   pip install -r ~/clearpath_ws/src/hold-my-gear-jackal/requirements.txt
   ```

   (You can store `vision_venv` wherever you prefer; update paths accordingly.)

---

## 3. Clone the Project and Add the Package

1. **Go to your Clearpath workspace source folder**

   ```bash
   cd ~/clearpath_ws/src
   ```

2. **Clone the repo**

   ```bash
   git clone https://github.com/<your-org>/hold-my-gear-jackal.git
   ```

3. **Expose the `jackal_yolo_follow` package to the workspace**

   Either symlink or copy:

   ```bash
   ln -s hold-my-gear-jackal/robot/jackal_yolo_follow .
   ```

---

## 4. Use the Provided `robot.yaml`

1. Copy (or symlink) the project’s `robot.yaml` to Clearpath’s config location:

   ```bash
   sudo cp ~/clearpath_ws/src/hold-my-gear-jackal/robot/config/robot.yaml /etc/clearpath/robot.yaml
   ```

2. Regenerate Clearpath bash setup if required:

   ```bash
   source /opt/ros/jazzy/setup.bash
   sudo ros2 run clearpath_generator_common generate_bash -s /etc/clearpath
   ```

   (Your image may already be configured; only regenerate if you changed robot config.)

---

## 5. Build the Workspace

```bash
cd ~/clearpath_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Add the workspace to your shell (if not already in `.bashrc`):

```bash
echo "source ~/clearpath_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 6. Launching the Robot Stack

### 6.1 Bring up platform + SLAM + Nav2

Depending on your Clearpath setup, this may be one of:

```bash
# Example: Clearpath Nav2 demo
ros2 launch clearpath_nav2_demos nav2.launch.py
```

or a similar Clearpath-provided launch file that starts:

* Jackal base controllers
* SLAM Toolbox
* Nav2 stack

Confirm in RViz (from the offboard laptop) that:

* The robot base frame is present.
* SLAM is building a map.
* Nav2 is active.

### 6.2 Bring up the RealSense camera

On the robot:

```bash
ros2 launch realsense2_camera rs_launch.py
```

Confirm topics (e.g.):

* `/j100_0000/sensors/camera_0/color/image`
* `/j100_0000/sensors/camera_0/depth/image`

---

## 7. Running the Follower Nodes

### 7.1 Method 1 – Nav2-based follower (recommended)

Starts both the YOLO detection node and the Nav2 bridge node.

```bash
ros2 launch jackal_yolo_follow yolo_nav2_follow.launch.py
```

Conceptually, this launch file should:

* Start `yolo_nav2_follower`:

  * Subscribes to camera image.
  * Runs YOLO.
  * Publishes `geometry_msgs/PoseStamped` on a `goal_pose` topic.
* Start `nav_to_pose_test`:

  * Subscribes to `goal_pose`.
  * Uses `nav2_simple_commander.BasicNavigator` to send Nav2 goals.
  * Cancels/updates goals based on new target positions.

### 7.2 Method 2 – Direct velocity follower

```bash
ros2 launch jackal_yolo_follow yolo_direct_follow.launch.py
```

This mode:

* Runs `yolo_follower`:

  * Subscribes to camera (and optionally depth).
  * Computes linear and angular velocity commands to follow the person.
  * Publishes `/cmd_vel` directly.

**Warning:** This mode does not use Nav2, SLAM, or obstacle avoidance. Use only in controlled environments and at low speeds.

---

## 8. Stopping the System

* Use `Ctrl+C` in each terminal to stop nodes.
* Use the Jackal E-stop for emergency stops.
* Optionally, stop base services using your standard Clearpath commands if needed.

````
