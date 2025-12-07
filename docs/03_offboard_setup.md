# Offboard Laptop Setup

The offboard computer is used for visualization and teleoperation. It runs RViz and connects to the Jackal over the network.

## 1. Base System Requirements

- Ubuntu **24.04** Desktop
- ROS 2 Jazzy (installed from binary packages)
- Network access to the Jackal (same Wi-Fi or direct Ethernet)

---

## 2. Install ROS 2 Jazzy and Clearpath Desktop

Follow the official ROS 2 Jazzy installation instructions for Ubuntu 24.04.

Then add Clearpath’s repositories and install the Clearpath desktop metapackage:

```bash
wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -

sudo sh -c 'echo \
  "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > \
  /etc/apt/sources.list.d/clearpath-latest.list'

sudo apt update
sudo apt install ros-jazzy-clearpath-desktop
````

Update rosdep sources for Clearpath:

```bash
sudo wget \
  https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list \
  -O /etc/ros/rosdep/sources.list.d/50-clearpath.list

rosdep update
```

---

## 3. Mirror the Robot Configuration (robot.yaml + setup.bash)

1. Create a Clearpath config folder:

   ```bash
   mkdir -p ~/clearpath
   ```

2. Copy the robot’s `robot.yaml` (from `/etc/clearpath/robot.yaml` on the Jackal or from this repo):

   ```bash
   cp /path/to/robot.yaml ~/clearpath/robot.yaml
   # or:
   cp ~/jackal_follow_ws/src/hold-my-gear-jackal/robot/config/robot.yaml ~/clearpath/robot.yaml
   ```

3. Generate the offboard `setup.bash`:

   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 run clearpath_generator_common generate_bash -s ~/clearpath
   ```

4. Add it to your `~/.bashrc`:

   ```bash
   echo "source ~/clearpath/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

Now every new terminal will have the correct Clearpath networking, domain ID, and namespaces.

---

## 4. Create a Workspace (optional but recommended)

If you want to keep offboard code and RViz configs in a proper workspace:

```bash
cd ~
mkdir -p jackal_follow_ws/src
cd jackal_follow_ws/src
```

Clone the project:

```bash
git clone https://github.com/<your-org>/hold-my-gear-jackal.git
```

(If you only want the RViz configuration, you don’t have to build anything.)

If you do plan to run any offboard ROS nodes, build:

```bash
cd ~/jackal_follow_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "source ~/jackal_follow_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 5. Networking

* Ensure the laptop is on the same network as the Jackal.
* If using Wi-Fi, connect to the Jackal’s SSID or the shared lab network.
* If using Ethernet directly, set a static IP in the `192.168.131.xxx` range (e.g. `192.168.131.99`).

With the Clearpath-generated `setup.bash` sourced, DDS discovery should “just work.”

You can verify connectivity with:

```bash
ros2 topic list
```

You should see topics such as:

* `/j100_0000/cmd_vel`
* `/j100_0000/sensors/camera_0/color/image`
* `/tf`
* `/tf_static`
* `/map` (once SLAM is running)

---

## 6. Running RViz

Use the provided RViz configuration:

```bash
rviz2 -d ~/jackal_follow_ws/src/hold-my-gear-jackal/offboard/rviz/jackal_follow.rviz
```

The RViz config should:

* Display the map and robot pose.
* Show RealSense RGB and/or depth images.
* Visualize Nav2 goals and paths.
* Optionally show the target firefighter’s detected position.

If you don’t yet have a config saved, you can:

1. Start RViz with default settings: `rviz2`.
2. Add displays for:

   * `Map` (topic `/map`)
   * `RobotModel` (TF frames)
   * `LaserScan` or `PointCloud2` derived from the depth camera.
   * `Image` (camera RGB).
3. Save the config as:

   ```bash
   File → Save Config As…
   ~/jackal_follow_ws/src/hold-my-gear-jackal/offboard/rviz/jackal_follow.rviz
   ```

---

## 7. Typical Workflow

1. On the **robot**:

   * Start platform + SLAM + Nav2.
   * Start RealSense.
   * Start one of the follower modes.

2. On the **laptop**:

   * Open a new terminal (with `~/clearpath/setup.bash` sourced).
   * Launch RViz with `jackal_follow.rviz`.
   * Monitor map, robot pose, and camera feeds.
   * Optionally send Nav2 goals or interact with the robot via RViz tools.

If you don’t see topics or TF frames in RViz, re-check:

* That the laptop terminal has `source ~/clearpath/setup.bash`.
* That the robot and laptop are on the same network.
* That `ros2 topic list` returns the Jackal topics.

```
