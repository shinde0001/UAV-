#!/bin/bash

export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=/home/parth/gr/src/uav_description/models:/home/parth/gr/src/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
export GAZEBO_PLUGIN_PATH=/home/parth/gr/src/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic
export LD_LIBRARY_PATH=/home/parth/gr/install/px4/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$GAZEBO_PLUGIN_PATH:$LD_LIBRARY_PATH

echo "Cleaning up old processes..."
pkill -9 -f gzserver
pkill -9 -f px4
pkill -9 -f mavros_node
pkill -9 -f uav_localization_node
pkill -9 -f local_obstacle_filter
pkill -9 -f planner_node
pkill -9 -f mission_node
rm -rf /home/parth/gr/src/PX4-Autopilot/build/px4_sitl_default/rootfs/eeprom
sleep 2

echo "Starting Gazebo Server and GUI..."
gzserver /home/parth/gr/src/uav_gazebo/worlds/simple_forest.world -s libgazebo_ros_init.so -s libgazebo_ros_factory.so &
GZ_PID=$!
gzclient &
GZCLIENT_PID=$!
sleep 5

echo "Spawning Iris UAV with LiDAR and IMU..."
gz model --spawn-file=/home/parth/gr/src/uav_description/models/iris_lidar_imu/model.sdf --model-name=iris -x 0 -y 0 -z 1.0
sleep 5

echo "Starting PX4 SITL (Daemon mode)..."
cd /home/parth/gr/src/PX4-Autopilot
export PX4_SYS_AUTOSTART=10016 
export PX4_SIM_MODEL=iris 
./build/px4_sitl_default/bin/px4 -d ./build/px4_sitl_default/etc &
PX4_PID=$!

echo "Waiting for PX4 to boot and connect to Gazebo (10s)..."
sleep 10

echo "Starting ROS 2 Autonomy Stack..."
source /opt/ros/humble/setup.bash
source /home/parth/gr/install/setup.bash
ros2 launch uav_bringup ros_nodes.launch.py

echo "Shutting down..."
kill -9 $PX4_PID
kill -9 $GZ_PID
kill -9 $GZCLIENT_PID
