#!/usr/bin/env python3
import random

world_header = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_forest">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
    </plugin>
"""

world_footer = """
  </world>
</sdf>
"""

def generate_tree(x, y, id):
    height = random.uniform(15.0, 25.0)
    radius = random.uniform(0.3, 0.6)
    return f"""
    <model name="tree_{id}">
      <pose>{x} {y} {height/2} 0 0 0</pose>
      <static>true</static>
      <link name="trunk">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{radius}</radius>
              <length>{height}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{radius}</radius>
              <length>{height}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.5 0.3 0.1 1</ambient>
            <diffuse>0.5 0.3 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

with open("/home/parth/gr/src/uav_gazebo/worlds/simple_forest.world", "w") as f:
    f.write(world_header)
    tree_id = 0
    # Create a 1km corridor along X axis (from x=20 to x=1020), width y=-30 to y=30
    for i in range(80):
        x = random.uniform(20.0, 1020.0)
        y = random.uniform(-30.0, 30.0)
        # leave a bit of clear space strictly at y=0, though it's a forest
        if -5 < y < 5:
            y = 10 if y > 0 else -10
        f.write(generate_tree(x, y, tree_id))
        tree_id += 1

    # Add a moving box (dynamic obstacle)
    f.write("""
    <model name="moving_box">
      <pose>50 5 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>2 2 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>2 2 2</size></box></geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
      <plugin name="moving_box_plugin" filename="libgazebo_ros_planar_move.so">
        <ros>
          <namespace>/obstacles</namespace>
        </ros>
        <update_rate>50.0</update_rate>
        <publish_odom>false</publish_odom>
        <publish_odom_tf>false</publish_odom_tf>
      </plugin>
    </model>
""")
    f.write(world_footer)
