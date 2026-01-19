
# Offboard Laptop Setup

The offboard computer is used primarily for visualization and monitoring. It runs RViz and connects to the Jackal over the network.

---

## 1. Base System Requirements

- Ubuntu 24.04 Desktop
- ROS 2 Jazzy (installed from binary packages)
- Network access to the Jackal (same Wi-Fi network or direct Ethernet)

---

## 2. Install ROS 2 Jazzy and Clearpath Desktop

Install ROS 2 Jazzy using the official instructions for Ubuntu 24.04.

Then add Clearpath’s repositories and install the Clearpath Desktop metapackage:

```bash
wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -

sudo sh -c 'echo \
  "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > \
  /etc/apt/sources.list.d/clearpath-latest.list'

sudo apt update
sudo apt install ros-jazzy-clearpath-desktop
````

Update `rosdep` sources for Clearpath:

```bash
sudo wget \
  https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list \
  -O /etc/ros/rosdep/sources.list.d/50-clearpath.list

rosdep update
```

---

## 3. Mirror the Robot Configuration

The offboard computer should mirror the robot’s Clearpath configuration so that it shares the same namespaces, domain ID, and DDS configuration.

1. Create a Clearpath configuration folder:

   ```bash
   mkdir -p ~/clearpath
   ```

2. Copy the robot’s `robot.yaml` and related files either from the Jackal (`/etc/clearpath`) or from this repository:

   ```bash
   cp ~/jackal_follow_ws/src/hold_my_gear_jackal/offboard/config/robot.yaml ~/clearpath/robot.yaml
   ```

   If you have copied additional Clearpath files (URDF, SRDF, platform, sensors, etc.), place them under `~/clearpath` following the same structure as `/etc/clearpath` on the robot.

3. Generate the offboard `setup.bash`:

   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 run clearpath_generator_common generate_bash -s ~/clearpath
   ```

4. Add this to your `~/.bashrc` so it is sourced automatically:

   ```bash
   echo "source ~/clearpath/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

---

## 4. Create a Workspace (Optional)

If you want to maintain offboard code and RViz configurations in a ROS workspace:

```bash
cd ~
mkdir -p jackal_follow_ws/src
cd jackal_follow_ws/src
```

Clone the project:

```bash
git clone https://github.com/tylerjhom/hold_my_gear_jackal.git
```

If you plan to run any offboard ROS nodes, build the workspace:

```bash
cd ~/jackal_follow_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "source ~/jackal_follow_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

If you only want RViz configurations, you may skip the build step.

---

## 5. Networking

* Ensure the laptop is on the same network as the Jackal.
* For Wi-Fi, connect to the Jackal’s SSID or the shared lab network.
* For direct Ethernet, configure a static IP in the `192.168.131.xxx` range (for example `192.168.131.99`).

With `~/clearpath/setup.bash` sourced, DDS discovery should function without additional configuration.

Verify connectivity:

```bash
ros2 topic list
```

You should see topics such as:

* `/j100_0000/cmd_vel`
* `/j100_0000/sensors/camera_0/color/image`
* `/tf` and `/tf_static`
* `/j100_0000/map` (once SLAM is running)

If no topics are visible, confirm:

* The robot is powered and running its ROS stack.
* The correct `setup.bash` files are sourced.
* The firewall is not blocking DDS; if needed:

  ```bash
  sudo ufw disable
  ```

---

## 6. Running RViz

### 6.1 Clearpath Navigation Visualization

A convenient way to visualize Nav2 and SLAM is via Clearpath’s visualization launch file:

```bash
ros2 launch clearpath_viz view_navigation.launch.py namespace:=j100_0000
```

This should display the map, robot pose, and navigation information.

---

### 6.2 Using the Project RViz Configuration

You can also use the project-specific RViz configuration:

```bash
rviz2 -d ~/jackal_follow_ws/src/hold_my_gear_jackal/offboard/rviz/jackal_follow.rviz
```

This configuration is intended to show:

* Map and robot pose
* RealSense RGB and/or depth images
* Laser scan derived from depth
* Nav2 goals and paths
* Any custom markers related to the follower behavior

If you do not yet have a saved configuration:

1. Start RViz:

   ```bash
   rviz2
   ```

2. Add displays for:

   * `Map` (topic `/j100_0000/map`)
   * `RobotModel` (from TF)
   * `LaserScan` or `PointCloud2` corresponding to the depth-derived scan
   * `Image` for the RealSense RGB stream
   * Any relevant Nav2 topics (global costmap, local costmap, etc.)

3. Save the configuration as:

   ```text
   ~/jackal_follow_ws/src/hold_my_gear_jackal/offboard/rviz/jackal_follow.rviz
   ```

---

## 7. Typical Workflow

1. On the **robot**:

   * Launch RealSense and the depth-to-laserscan node.
   * Launch SLAM and Nav2.
   * Start one of the follower modes (`yolo_nav2_follower` or `yolo_follower`).

2. On the **laptop**:

   * Open a terminal with `~/clearpath/setup.bash` and any workspace setup sourced.
   * Run either `view_navigation.launch.py` or RViz with `jackal_follow.rviz`.
   * Monitor map, robot pose, and camera feeds.
   * Optionally interact with the robot using Nav2 goal tools in RViz.

If you do not see map data or TF frames in RViz, recheck that:

* The offboard `setup.bash` is properly sourced.
* The robot’s navigation stack is running.
* Network connectivity and firewall settings are correct.

```
