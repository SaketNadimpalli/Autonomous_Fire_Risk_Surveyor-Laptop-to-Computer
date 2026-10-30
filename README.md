# Autonomous Fire Risk Surveyor

A multi-robot system in ROS2 that surveys a simulated forest for fire risk. A drone explores and maps the area on its own while a ground robot (Clearpath Husky) navigates below. Built for UTS 41068 Robotics Studio 1 by a team of four.

<!-- Add a GIF or screenshot of RViz here: drone exploring, frontiers in blue, A* path -->
<!-- ![Drone exploring the forest in RViz](docs/demo.gif) -->

## What it does

1. **Maps the forest:** the drone builds an occupancy map with SLAM Toolbox as it flies.
2. **Picks where to explore next:** a frontier explorer finds the edges between mapped and unmapped space, groups them into clusters, and scores each one by how much unknown area it would reveal and how far away it is.
3. **Plans a path:** a custom A* planner plans across the occupancy grid to the chosen frontier, with obstacle inflation and a configurable way of treating unknown space.
4. **Follows the path:** a pure-pursuit-style path follower turns the path into velocity commands for the drone.
5. **Ground robot:** the Husky runs Nav2 in the same simulation and shares its map with the drone's planner.

The planned final step, the drone sending fire hotspot locations for the Husky to inspect, was not finished (see below).

The drone and the Husky run together in one Ignition Gazebo world, each with its own namespace, TF tree and EKF localisation.

## My part

I led the team and designed the drone's autonomous navigation (`src/afr_planning`):

- **`astar_planner.py`:** A* on the SLAM occupancy grid with 8-connected moves, obstacle inflation, three policies for unknown cells (blocked, free or penalised), and goal snapping to the nearest free cell. Publishes the path and RViz markers.
- **`frontier_explorer.py`:** frontier detection and clustering, scoring by expected information gain and distance, goal time-outs and re-selection, with RViz markers for every cluster and the chosen target.
- **`path_follower.py`:** look-ahead path following with speed limits and a goal tolerance.
- **Integration:** got the drone and the Husky running in the same world, and set up the drone's EKF (`robot_localization`), `twist_mux` and the `drone/map` to `drone/odom` to `drone_base_link` TF chain.

Teammates worked on perception (fire hotspot detection from the drone camera, in `src/temp_dylan`) and the ground robot.

## Packages

| Package | Contents |
|---|---|
| `afr_planning` | A* planner, frontier explorer and path follower for the drone |
| `41068_ignition_bringup_with_drone` | Simulation worlds, Husky and drone models, SLAM, Nav2 and EKF configs, launch files (based on the subject's starter package) |
| `forest_core` | Early LiDAR obstacle avoidance and exploration nodes |
| `uts_forest_bringup` | Bringup launch files |
| `temp_dylan` | Perception: hotspot detector, object mapper, path smoothing |

## Running it

Tested on **Ubuntu 22.04, ROS2 Humble and Ignition Gazebo Fortress**.

```bash
# The repo is the workspace: it contains the src/ folder
git clone https://github.com/SaketNadimpalli/Autonomous_Fire_Risk_Surveyor-Laptop-to-Computer.git ~/afr_ws
cd ~/afr_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

Then, each in its own terminal:

```bash
# 1. Simulation with SLAM, Nav2 and RViz
ros2 launch 41068_ignition_bringup 41068_ignition.launch.py rviz:=true nav2:=true world:=large_demo

# 2. A* planner for the drone
ros2 run afr_planning astar_planner --ros-args -p unknown_policy:=penalized -p unknown_penalty:=2.0 \
  -p inflate_radius:=0.5 -p inflate_ignore_unknown:=true -p occ_threshold:=60 -p plan_rate_hz:=2.0

# 3. Path follower
ros2 run afr_planning path_follower

# 4. Autonomous exploration
ros2 run afr_planning frontier_explorer
```

To send the drone a single goal by hand instead of running the explorer:

```bash
ros2 topic pub /drone/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'drone/map'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" -1
```

More setup notes and troubleshooting are in `src/41068_ignition_bringup_with_drone/41068_ignition_bringup/README.md`.

## What I'd do differently

- Integrate early and often. We built the drone navigation, perception and ground robot separately and left joining them until the last weeks, so the drone-to-Husky handoff was never finished. Next time I'd agree the interfaces between parts (topics, message types, frames) in week one and get a rough end-to-end version running before polishing any single part.
- Plan in 3D. The planner works on a 2D grid at a fixed altitude, so the drone can't fly over or under canopy.
- Replace the hand-tuned frontier scoring with an information-gain measure based on the camera's field of view, since the goal is to see the ground rather than just map it.
- Finish the handoff: have the perception node publish hotspot positions in the shared map frame, and a small task allocator send the nearest one to the Husky as a Nav2 goal.
