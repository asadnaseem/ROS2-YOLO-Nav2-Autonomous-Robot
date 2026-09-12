# ROS2-YOLO-Nav2-Autonomous-Robot
# YOLO + Nav2 Professional Sensor-Fusion Simulation

A complete ROS 2 Humble simulation containing a custom **Semantic Rover**, a
dense logistics-yard Gazebo world, Nav2 localization and navigation, RViz, lidar, RGB-D
camera, and YOLOv8.

YOLO is not used as a replacement for Nav2. YOLO detects semantic objects in
the RGB image, the depth camera estimates their positions, and the node
publishes those positions as `/yolo/obstacles`. Both Nav2 costmaps subscribe to
that point cloud. Nav2 then replans or steers around the detected obstacle. The
lidar and RGB-D remain responsible for walls and objects YOLO cannot recognize.
This separation between semantic perception and geometric safety is deliberate.

## Tested target platform

- Ubuntu 22.04
- ROS 2 Humble Desktop
- Gazebo Classic 11

Ubuntu 24.04 normally uses ROS 2 Jazzy and modern Gazebo; this Humble/Gazebo
Classic project should be run on Ubuntu 22.04 or in an Ubuntu 22.04 VM.

## One-time dependencies

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-gazebo-ros-pkgs ros-humble-cv-bridge \
  python3-colcon-common-extensions python3-pip
```

## One command to build and run

```bash
cd ~/yolo_nav2_project && chmod +x run.sh && ./run.sh
```

The script builds the package and launches everything. After Nav2 becomes
active, a fault-tolerant patrol manager continuously visits six warehouse
waypoints. It retries failures, skips unreachable goals, and starts the next
lap instead of stopping after one goal. You can still send a manual RViz goal.

The Gazebo GUI opens by default. The world uses lightweight geometry and stop
stop-sign models so YOLO has reliable semantic targets without loading large
vehicle or human meshes. Tables, chairs, indoor trees, racks, crates and barrels exercise geometric
avoidance. For maximum VM speed, use `./run.sh gazebo_gui:=false`.

All furniture and plants use lightweight Gazebo primitives sized for the room,
leaving clear patrol corridors around the central obstacles.

## What you should see

1. Gazebo opens a warehouse with racks, a crate, and a person.
2. The new blue Semantic Rover starts at `(-4, -3)`.
3. RViz shows the static map, lidar, Nav2 path and local costmap.
4. YOLO detections appear in the camera view.
5. Orange point clusters show YOLO obstacles in RViz.
6. Nav2 drives to the goal while avoiding mapped, lidar, and YOLO obstacles.

## Main topics

- `/camera/image_raw` — RGB camera
- `/camera/depth/image_raw` — metric depth image
- `/yolo/annotated_image` — detection overlay
- `/yolo/obstacles` — semantic `PointCloud2` used by Nav2
- `/depth/obstacles` — geometric RGB-D obstacles, including walls and unknown objects
- `/scan` — lidar safety obstacles
- `/local_costmap/costmap` — combined local costmap
- `/cmd_vel` — Nav2 velocity output
- `/mission/status` — patrol state and remaining distance

## Architecture

```text
RGB image -> YOLO bounding box ----+
                                   +-> 3D semantic points -> Nav2 costmaps
Depth image -----------------------+                         |
Lidar ------------------------------------------------------+-> cmd_vel
Static map -------------------------------------------------+
```

This is a simulation project. For a physical robot, replace the Gazebo camera,
depth, lidar, odometry and motor interfaces with the real hardware topics while
keeping the YOLO-to-costmap and Nav2 nodes.
