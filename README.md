# Autonomous UAV GPS-Denied Forest Navigation

An end-to-end autonomous navigation stack for Unmanned Aerial Vehicles (UAVs) operating in GPS-denied forest environments. Built with **ROS 2 Humble**, **PX4 Autopilot (SITL)**, **Gazebo Classic**, and **MAVROS**.

---

## 🌟 Key Features

* **GPS-Denied Localization**: PCL-based 3D LiDAR Iterative Closest Point (ICP) odometry fused into PX4's EKF2 via MAVROS (`/mavros/vision_pose/pose`).
* **Reactive Obstacle Avoidance**: Dynamic point cloud filtering and 3D obstacle dodging algorithm for navigating dense forest canopies.
* **Autonomous Waypoint Manager**: State-machine node handling automated arming, OFFBOARD mode switching, waypoint tracking, and landing.
* **Resource Optimized**: Single-script orchestrator (`launch_all.sh`) configured to execute full SITL + ROS 2 stack cleanly under 6.0 GB RAM and ~30% CPU utilization.
* **Procedural Forest Environment**: 1 km traversal track with 450 procedurally generated trees and scaled ground plane boundaries.

---

## 🏗️ Architecture Overview

```
+-----------------------------------------------------------------------------------+
|                                  GAZEBO SIMULATION                                |
|  +------------------------+                        +---------------------------+  |
|  |   3D Velodyne LiDAR    |                        |        Iris UAV           |  |
|  +-----------+------------+                        +-------------+-------------+  |
+--------------|---------------------------------------------------|----------------+
               | /velodyne_points                                  | Sensors
               v                                                   v
+------------------------------+                     +------------------------------+
|     uav_localization_node    |                     |      PX4 SITL + MAVROS       |
|   (3D PCL ICP Odometry)      |                     |    (EKF2 Position Fusion)    |
+--------------+---------------+                     +--------------+---------------+
               | /mavros/vision_pose/pose                           ^
               +----------------------------------------------------+
                                                                    | /mavros/setpoint_position/local
+------------------------------+     /planner/cmd_pose     +--------+---------------------+
|        uav_planner           |-------------------------->|         uav_mission          |
|  (3D Obstacle Avoidance)     |                           |   (Offboard State Machine)   |
+--------------+---------------+                           +------------------------------+
               ^ /uav/local_obstacles                               ^ /mission/waypoint
               |                                                    |
+--------------+---------------+                                    |
|   local_obstacle_filter      |------------------------------------+
+------------------------------+
```

---

## 📦 Package Breakdown

| Package | Language | Function |
| :--- | :--- | :--- |
| **`uav_bringup`** | Python / ROS 2 | Main launch orchestrator for MAVROS and all autonomy nodes. |
| **`uav_description`** | XML / SDF | Custom Iris quadcopter model integrated with 3D LiDAR & IMU sensors. |
| **`uav_gazebo`** | World / SDF | Procedural 1 km forest world environment (`simple_forest.world`). |
| **`uav_localization`** | C++ (PCL) | Frame-to-frame ICP scan matching for pose estimation. |
| **`uav_obstacle_detection`** | C++ (PCL) | Filters raw pointclouds into local collision threat zones. |
| **`uav_planner`** | Python | Generates reactive waypoints to dodge trees in real time. |
| **`uav_mission`** | Python | System state manager for takeoff, waypoint progression, and landing. |

---

## 🚀 Quick Start

### 1. Prerequisites
Ensure the following software packages are installed on your Linux system:
* ROS 2 Humble
* PX4 Autopilot (`PX4-Autopilot` directory setup)
* Gazebo Classic 11
* MAVROS (`ros-humble-mavros`, `ros-humble-mavros-extras`)
* PCL ROS (`ros-humble-pcl-ros`, `ros-humble-pcl-conversions`)

### 2. Workspace Setup & Build
```bash
# Clone the repository
git clone https://github.com/shinde0001/UAV-.git
cd UAV-

# Build the ROS 2 packages
source /opt/ros/humble/setup.bash
colcon build
```

### 3. Launching the Autonomous Mission
Run the master orchestration script to launch Gazebo, spawn the UAV, boot PX4 SITL, and start all ROS 2 nodes:
```bash
chmod +x launch_all.sh
./launch_all.sh
```

---

## ⚙️ Configuration & EKF2 Tuning

For GPS-denied operations, PX4's EKF2 is configured to fuse external vision odometry instead of GPS:
* `EKF2_EV_CTRL = 3` (Fuses Horizontal & Vertical External Vision Position)
* `EKF2_HGT_REF = 0` (Barometric altitude reference)
* `EKF2_GPS_CTRL = 0` (Disables GPS requirement)

---

## 📄 License
This project is licensed under the MIT License.
