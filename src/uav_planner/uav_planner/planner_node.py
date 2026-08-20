#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ReactivePlanner(Node):
    def __init__(self):
        super().__init__('reactive_planner')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_odom = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.odom_cb, qos_profile)
        self.sub_obs = self.create_subscription(PointCloud2, '/uav/local_obstacles', self.obs_cb, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/mission/waypoint', self.goal_cb, 10)
        
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)
        self.pub_cmd = self.create_publisher(PoseStamped, '/planner/cmd_pose', 10)
        
        self.timer = self.create_timer(0.2, self.plan_loop) # 5 Hz
        
        self.current_pos = None
        self.current_yaw = 0.0
        self.goal_pos = None
        self.obstacles = np.array([])
        
        self.get_logger().info("Reactive Planner Started.")
        
    def odom_cb(self, msg):
        self.current_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        # Extract yaw from quaternion
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)
        
    def goal_cb(self, msg):
        self.goal_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
    def obs_cb(self, msg):
        pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if len(pts) > 0:
            self.obstacles = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
        else:
            self.obstacles = np.array([])
        
    def plan_loop(self):
        if self.current_pos is None or self.goal_pos is None:
            return
            
        direction = self.goal_pos - self.current_pos
        dist_to_goal = np.linalg.norm(direction[:2]) # 2D horizontal distance to goal
        
        if dist_to_goal < 1.0:
            cmd = PoseStamped()
            cmd.header.frame_id = "odom"
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.pose.position.x = float(self.goal_pos[0])
            cmd.pose.position.y = float(self.goal_pos[1])
            cmd.pose.position.z = float(self.goal_pos[2])
            self.pub_cmd.publish(cmd)
            return
            
        # Direction unit vector towards goal in 2D (horizontal)
        dir_norm = np.linalg.norm(direction[:2])
        dir_2d = direction[:2] / (dir_norm if dir_norm > 1e-4 else 1.0)
        
        dodge_body = np.zeros(2) # 2D dodge in body frame (forward, left)
        
        if len(self.obstacles) > 0:
            # Obstacles are in Body Frame: X = forward, Y = left, Z = up
            rel_obs = self.obstacles
            
            # Find obstacles directly in front of UAV (0.5m < X < 8.0m, |Y| < 2.5m, |Z| < 2.0m)
            threat_mask = (rel_obs[:, 0] > 0.5) & (rel_obs[:, 0] < 8.0) & (np.abs(rel_obs[:, 1]) < 2.5) & (np.abs(rel_obs[:, 2]) < 2.0)
            threats = rel_obs[threat_mask]
            
            if len(threats) > 0:
                self.get_logger().warn("Obstacle directly ahead! Dodging...")
                # Count obstacles on left (+Y) vs right (-Y)
                left_count = np.sum(threats[:, 1] > 0)
                right_count = np.sum(threats[:, 1] <= 0)
                
                # Dodge to the side with fewer obstacles (3.5 meters sideways offset)
                if left_count <= right_count:
                    dodge_body = np.array([0.0, 3.5]) # Dodge Left (+Y body)
                else:
                    dodge_body = np.array([0.0, -3.5]) # Dodge Right (-Y body)
        
        # Transform Body Frame dodge offset to World Frame (using UAV current yaw)
        cos_y = np.cos(self.current_yaw)
        sin_y = np.sin(self.current_yaw)
        dodge_world_x = dodge_body[0] * cos_y - dodge_body[1] * sin_y
        dodge_world_y = dodge_body[0] * sin_y + dodge_body[1] * cos_y
        dodge_world = np.array([dodge_world_x, dodge_world_y])
        
        # Compute forward step (4.0m towards goal + dodge offset)
        step_len = min(4.0, dist_to_goal)
        target_world_xy = self.current_pos[:2] + dir_2d * step_len + dodge_world
        
        cmd = PoseStamped()
        cmd.header.frame_id = "odom"
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.pose.position.x = float(target_world_xy[0])
        cmd.pose.position.y = float(target_world_xy[1])
        cmd.pose.position.z = float(self.goal_pos[2]) # Maintain goal altitude (10m)
        self.pub_cmd.publish(cmd)
        
        # Publish path
        path = Path()
        path.header = cmd.header
        p1 = PoseStamped()
        p1.pose.position.x = float(self.current_pos[0])
        p1.pose.position.y = float(self.current_pos[1])
        p1.pose.position.z = float(self.current_pos[2])
        path.poses.append(p1)
        path.poses.append(cmd)
        self.pub_path.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = ReactivePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
