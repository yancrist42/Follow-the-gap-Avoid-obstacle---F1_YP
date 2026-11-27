# README — `ReactiveFollowGap` Node (`obstacle_node.py`)

Complementary documentation for the repository (alongside the base `f1tenth_gym_ros` `README.md`) covering the `ReactiveFollowGap` node (`obstacle_node.py`), located in the `controller` directory next to `gap_node`.

---

## Description of the Approach Used

The `ReactiveFollowGap` node implements a reactive controller based on **Follow the Gap**, speed modulation (*Trail Braking*), and lap telemetry without requiring a prior map of the track:

- **LIDAR Preprocessing (`preprocess_lidar`):** Converts `/scan` (`LaserScan`) readings into a NumPy array, clips the maximum range (e.g., `3.9` m in the stable baseline configuration), and zeroes out (`0.0`) rays outside the frontal `-70°` to `+70°` sector.
- **Safety Bubble (`lidar_callback`):** Identifies the closest valid point (`closest_dist`) within the frontal sector, computes the angular span equivalent to `self.bubble_radius` (`0.62` m by default), and zeroes out (`0.0`) nearby indices to prevent getting too close to walls or obstacles.
- **Gap Search and Target Point (`find_max_gap` and `find_best_point`):** Finds the longest continuous sequence of readings greater than `0.01` and selects a target point within that gap (such as the center index among the maximum range values).
- **Steering and Speed Control (`lidar_callback`):** Converts the selected index into a `steering_angle` clamped to `[-0.4, 0.4]` and calculates `speed` scaled to `max_speed` (`10.0` by default) based on steering angle, forward distance (`front_dist`), and obstacle proximity (`closest_dist`).
- **Lap Telemetry (`odom_callback`):** Reads `(x, y)` from `/ego_racecar/odom`, sets the finish line at the initial position, arms the lap trigger once the vehicle exceeds `10.0` m from the start, and logs each lap and its duration upon returning within `4.5` m.

---

## Code Structure

The `controller` folder houses both `gap_node` and the obstacle avoidance/overtaking node (`obstacle_node.py`). Inside `obstacle_node.py`, the code is organized into:

- **`ReactiveFollowGap.__init__`:** Configures the `'reactive_node'` node, subscriptions to `/scan` and `/ego_racecar/odom`, the publisher to `/drive` (`AckermannDriveStamped`), `self.bubble_radius`, the `max_speed` and `car_label` parameters, and telemetry variables (`start_x`, `start_y`, `lap_start_time`, `lap_count`, `is_away`).
- **`odom_callback`:** Manages the finish line, the `is_away` state (> `10.0` m), and lap counting/timing (< `4.5` m).
- **`preprocess_lidar`:** Applies range clipping and the `-70°` to `+70°` angular window.
- **`find_max_gap`:** Locates the start and end indices of the widest continuous gap.
- **`find_best_point`:** Selects the target index inside the chosen gap.
- **`lidar_callback`:** Runs on every scan to detect the closest obstacle, apply the safety bubble, find the best gap, compute `steering_angle` and `speed`, and publish to `/drive`.
- **`main`:** Initializes `rclpy`, runs `rclpy.spin`, catches `KeyboardInterrupt` (`Ctrl+C`) to cleanly close telemetry, and destroys the node and `rclpy` in `finally`.

---

## Running Instructions

Assuming ROS 2, `f1tenth_gym`, `f1tenth_gym_ros`, and the simulation environment are already installed and built according to the main `README.md`, follow these steps to run this node in obstacle scenarios or with two agents on track.

### 1. Modifications in `sim.yaml` and Maps
Edit `f1tenth_gym_ros/config/sim.yaml` according to the scenario you want to test:

- **Change Map (`map_path`):** Point `map_path` to the desired map, either the standard track or the obstacle map **`Catalunya_Obstacle`** (using the corresponding path in your environment or container).
- **Configure One or Two Agents (`num_agent`):** Set `num_agent` to `1` (single agent, e.g., on `Catalunya_Obstacle`) or `2` (two agents for overtaking tests).
- **Verify Poses and Topics:** Adjust initial poses (`ego` / `opponent`) if needed. With two agents, the simulator publishes `/scan`, `/ego_racecar/odom`, `/opp_scan`, and `/opp_racecar/odom`, and subscribes to `/drive` and `/opp_drive` (you must publish to both drive topics for both vehicles to move).

[Screencast from 11-22-2026 12:02:12 PM.webm](https://github.com/user-attachments/assets/a924139d-6b75-4790-9941-ca0188ce2825)

### 2. Node Modifications and Building
- Keep or update `obstacle_node.py` inside the `controller` folder (`~/f1tenth_ws/src/controllers/controllers/obstacle_node.py`), tuning parameters such as `max_speed`, `self.bubble_radius`, or the distance clipping in `preprocess_lidar` to refine driving or overtaking behavior.
- If you modify `sim.yaml` or ROS 2 packages, rebuild and re-source the workspace:

```bash
cd ~/f1tenth_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 3. Launching the Simulation and Nodes

#### Step 3.1 — Start the Simulation

In a terminal, launch the simulation environment:

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_ws/install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
> **RViz Visual Note (2-Agent Mode):** Even when setting `num_agent: 2` in `sim.yaml`, RViz only renders the 3D model of the primary vehicle (`ego_racecar`) by default, so the second vehicle (`opp_racecar`) will initially appear only as the point contour of its chassis in the `LaserScan`. To also display the opponent's 3D model in RViz:
> 1. In the left panel (**Displays**), click **Add** and select **RobotModel**.
> 2. Expand the newly added **RobotModel** and change its **Description Topic** property to `/opp_robot_description` (or `/opp_racecar/robot_description`).
> 3. *(Optional)* Save the configuration via **File -> Save Config**.

#### Step 3.2 — Run the Main Agent (`EGO`)

In a second terminal, launch the node for the `EGO` vehicle:

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_ws/install/setup.bash

python3 ~/f1tenth_ws/src/Follow-the-gap-Avoid-obstacle---F1_YP/src/controllers/controllers/obstacle_node.py --ros-args \
  -r __node:=ego_gap_node \
  -p max_speed:=16.0 \
  -p car_label:=EGO
```

#### Step 3.3 — Run the Second Agent (`OPP`) *(only with `num_agent: 2`)*

In a third terminal, launch the node for the `OPP` opponent by remapping its LIDAR, drive, and odometry topics:

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_ws/install/setup.bash

python3 ~/f1tenth_ws/src/Follow-the-gap-Avoid-obstacle---F1_YP/src/controllers/controllers/obstacle_node.py --ros-args \
  -r __node:=opp_gap_node \
  -r /scan:=/opp_scan \
  -r /drive:=/opp_drive \
  -r /ego_racecar/odom:=/opp_racecar/odom \
  -p max_speed:=4.0 \
  -p car_label:=OPP
```
https://youtu.be/aJ9S07VAKsQ

To stop any of the nodes cleanly and display the closing telemetry summary, press `Ctrl + C` in its respective terminal.

#### Simulation on 10 lap

https://youtu.be/7FbAcX7UaMs

### 4. Recorded Data

Link for excel sheet of ros bag:
https://drive.google.com/file/d/1XI3qqJkJISjag6Wb8UnnYU8yFjsih1Ks/view?usp=drive_link
