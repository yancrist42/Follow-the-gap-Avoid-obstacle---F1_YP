# F1Tenth Autonomous Racing: Reactive "Follow the Gap" Controller with Telemetry and Trail Braking

This repository contains an advanced implementation of the classic **Follow-the-Gap (FtG)** algorithm optimized for endurance testing (*Endurance*) in the **F1Tenth** simulator, using **ROS 2 Humble**.

The system combines LiDAR-based reactive navigation with a *Trail Braking* speed profile and an integrated odometry telemetry module for automatic lap-time recording. Additionally, it includes custom maps (Circuit de Catalunya) with static obstacles to test collision avoidance and reaction time.


## 1. Control Architecture and Theoretical Approach

To achieve a balance between top speed on straights and absolute stability in tight corners (completing multiple consecutive laps without collisions at an average of ~42 seconds per lap), the controller implements the following modules:

1. **Preprocessing and Field of View (FoV Filtering):**
   * The LiDAR point cloud is cropped to a strict frontal range of **-70° to +70°** to prevent "bubble hijacking" caused by nearby side walls.
   * An upper distance limit (`np.clip` at **3.9 meters**) is applied to keep the algorithm from anticipating distant exits and prematurely cutting inner apexes (*corner-cutting*).

2. **Obstacle Avoidance via Safety Bubble:**
   * The closest point to the vehicle is identified, and a safety bubble with a dynamic radius of **`0.60 meters`** is "inflated," temporarily blinding those points to force the car to seek the widest free-space *gap*.

3. **Stepped Trail Braking Speed Control:**
   * Instead of slamming the brakes (which causes oversteer or spins), the system uses the averaged reading of the frontal laser rays to apply gradual deceleration (steps of 14.0 m/s $\rightarrow$ 9.5 m/s $\rightarrow$ 7.5 m/s $\rightarrow$ 6.0 m/s) when approaching a corner or obstacle.
   * Cornering speed is directly coupled to the steering angle (`steering_angle`), ensuring optimal traction in both wide and tight arcs.

4. **Odometry Telemetry System (`/ego_racecar/odom`):**
   * An independent submodule captures the vehicle's two-dimensional position upon startup, registers that point as the finish line, and accurately calculates the elapsed time per lap, printing clean results to the terminal console.

## 2. Code Structure

The functional core of the controller resides in the `gap_node.py` file. This script defines the `ReactiveFollowGap` class, a ROS 2 node that interacts with the physics engine through:

* **Subscribers:**
  * `/scan` (`sensor_msgs/msg/LaserScan`): Receives environment readings and feeds the `lidar_callback` function.
  * `/ego_racecar/odom` (`nav_msgs/msg/Odometry`): Tracks spatial position through the `odom_callback` function for lap timing.
* **Publisher:**
  * `/drive` (`ackermann_msgs/msg/AckermannDriveStamped`): Sends the calculated acceleration and steering angle commands to the simulator actuators.
* **Key Functions:**
  * `preprocess_lidar`: Filters out blind angles and crops the effective range.
  * `find_max_gap` & `find_best_point`: Analyze the free-space array to determine the trajectory.

## 3. Installation and Usage Guide (From Scratch)

This guide details the verified steps to install the base F1Tenth simulator from scratch in a clean environment, integrate this reactive controller, and run the simulation on the custom Catalunya map.

### 3.1. Prerequisites
* **Operating System:** Ubuntu 22.04 LTS
* **Middleware:** ROS 2 Humble installed (`source /opt/ros/humble/setup.bash`).
* **Base Tools:** Run the following block to install `pip`, `colcon`, and `rosdep2` without configuration conflicts:

```bash
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions python3-rosdep2 git
rosdep update
```

### 3.2. Install the Physics Engine (`f1tenth_gym`) and Create the Workspace
Clone and install the official physics engine in editable mode, install compatible dependencies (`numpy<2.0`), and set up the ROS 2 workspace:

```bash
# 1. Clone and install the official physics engine
cd ~
git clone https://github.com/f1tenth/f1tenth_gym.git
cd ~/f1tenth_gym
pip3 install "numpy<2.0" transforms3d
pip3 install -e .

# 2. Create the ROS 2 workspace and clone the base simulator
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros.git
```

### 3.3. Clone this Reactive Controller
Inside the same `src` folder of your workspace, clone this repository to add the *Follow the Gap* algorithm and custom maps:

```bash
cd ~/f1tenth_ws/src
git clone https://github.com/yancrist42/Follow-the-gap-Avoid-obstacle---F1_YP.git
```
[Screencast from 11-15-2026 06:31:49 PM.webm](https://github.com/user-attachments/assets/6161680b-c2ff-490c-8acb-20bdaef17989)


### 3.4. Configure the Custom Map
Move the Circuit de Catalunya files to the simulator package and automatically update the path, image format, and sensor offset in `sim.yaml`:

```bash
# 1. Copy the map files (.png and .yaml)
cp ~/f1tenth_ws/src/Follow-the-gap-Avoid-obstacle---F1_YP/maps/Catalunya* ~/f1tenth_ws/src/f1tenth_gym_ros/maps/

# 2. Automatically configure map_path and the .png extension in sim.yaml
sed -i "s|map_path:.*|map_path: '/home/$USER/f1tenth_ws/src/f1tenth_gym_ros/maps/Catalunya_map'|g" ~/f1tenth_ws/src/f1tenth_gym_ros/config/sim.yaml
sed -i "s|map_img_ext:.*|map_img_ext: '.png'|g" ~/f1tenth_ws/src/f1tenth_gym_ros/config/sim.yaml

# 3. Edit the scan_distance_to_base_link parameter to match the vehicle's calibrated sensor geometry
sed -i "s|scan_distance_to_base_link:.*|scan_distance_to_base_link: 0.0|g" ~/f1tenth_ws/src/f1tenth_gym_ros/config/sim.yaml
```
*(Note: If the vehicle spawns outside the track when starting the simulation, adjust the initial coordinates `sx`, `sy`, and `stheta` inside `~/f1tenth_ws/src/f1tenth_gym_ros/config/sim.yaml`).*

### 3.5. Install ROS 2 Dependencies and Build
Return to the root of your workspace, install dependencies skipping the obsolete `python-transforms3d-pip` key (already installed via `pip` in step 3.2), and build:

```bash
source /opt/ros/humble/setup.bash
cd ~/f1tenth_ws
rosdep install -i --from-path src --rosdistro humble --skip-keys python-transforms3d-pip -y
colcon build --symlink-install
source install/setup.bash
```
[Screencast from 11-15-2026 06:35:03 PM.webm](https://github.com/user-attachments/assets/a5f99bb4-b104-4500-a36a-f7f37e8d1f12)

### 3.6. Running the Simulation

1. **Terminal 1 (Simulator and RViz):** Launch the graphical environment and physics bridge:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/f1tenth_ws/install/setup.bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```

2. **Terminal 2 (Reactive Controller):** Open a new terminal, source the environment, and run the node:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/f1tenth_ws/install/setup.bash
   python3 ~/f1tenth_ws/src/Follow-the-gap-Avoid-obstacle---F1_YP/src/controllers/controllers/gap_node.py
   ```
https://youtu.be/-MU0UabO3GE
   
You will see the vehicle begin processing laser scanner data and dodging track obstacles while the terminal prints clean lap times for each completed lap.

### 3.6.1 Simulation on 10 lap

https://youtu.be/sevWEooKhg8

### 3.7. Common Troubleshooting
* **Error launching `gym_bridge_launch.py` related to `coverage`:** On some Ubuntu 22.04 installations, the `coverage` library conflicts with the simulation environment. If this occurs, uninstall it and rebuild:
  ```bash
  pip3 uninstall -y coverage
  sudo apt purge -y python3-coverage
  cd ~/f1tenth_ws && rm -rf build/ install/ log/
  colcon build --symlink-install
  ```

## 4. Key Calibration Parameters (Tuning)

If you want to test the controller on other tracks or modify driving aggressiveness, you can adjust the following critical variables directly inside the `gap_node.py` script:

| Parameter | Optimal Value | Technical Description |
| :--- | :--- | :--- |
| **`bubble_radius`** | `0.60 m` | Safety radius to isolate critical obstacles and prevent frontal collisions or lateral wall scrapes. |
| **`np.clip` (LiDAR)** | `3.9 m` | Maximum vision distance to prevent *corner-cutting* and force the car to stay centered. |
| **Straight Speed** | `14.0 m/s` | Maximum speed on long straights with clear traction. The system steps it down automatically using *Trail Braking*. |
| **Field of View** | `±70°` | Useful opening angle of the LiDAR. Ignores side and rear walls to prevent erratic steering decisions. |


## 5. Recorded Data

Link of excel sheet for ros bag:
https://drive.google.com/file/d/1_EJ_xP-l7UVr-FJKIuDuV1RTgYB_k-qN/view?usp=drive_link
