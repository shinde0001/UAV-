#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry, Path
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
        self.goal_pos = None
        self.obstacles = []
        
        self.get_logger().info("Reactive Planner Started.")
        
    def odom_cb(self, msg):
        self.current_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
    def goal_cb(self, msg):
        self.goal_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
    def obs_cb(self, msg):
        pts = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.obstacles = np.array(list(pts))
        
    def plan_loop(self):
        if self.current_pos is None or self.goal_pos is None:
            return
            
        direction = self.goal_pos - self.current_pos
        dist_to_goal = np.linalg.norm(direction)
        if dist_to_goal < 1.0:
            cmd = PoseStamped()
            cmd.header.frame_id = "odom"
            cmd.pose.position.x = self.goal_pos[0]
            cmd.pose.position.y = self.goal_pos[1]
            cmd.pose.position.z = self.goal_pos[2]
            self.pub_cmd.publish(cmd)
            return
            
        direction = direction / dist_to_goal
        
        dodge_vector = np.zeros(3)
        if len(self.obstacles) > 0:
            rel_obs = self.obstacles # PointCloud is already in sensor frame (relative to drone)
            dists = np.linalg.norm(rel_obs, axis=1)
            
            # Find obstacles within 15m
            close_mask = dists < 15.0
            close_obs = rel_obs[close_mask]
            
            if len(close_obs) > 0:
                close_dists = dists[close_mask]
                dirs = close_obs / close_dists[:, np.newaxis]
                dots = np.dot(dirs, direction)
                
                # If obstacle is directly in front (dot > 0.8) and close (< 10m)
                threat_mask = (dots > 0.8) & (close_dists < 10.0)
                threats = close_obs[threat_mask]
                
                if len(threats) > 0:
                    self.get_logger().warn(f"Obstacle ahead! Dodging...")
                    # Compare left vs right side to dodge into open space
                    left_perp = np.array([-direction[1], direction[0], 0.0])
                    if np.linalg.norm(left_perp) < 0.1:
                        left_perp = np.array([0.0, 1.0, 0.0])
                    else:
                        left_perp = left_perp / np.linalg.norm(left_perp)
                    
                    right_perp = -left_perp
                    
                    left_threats = np.sum(np.dot(threats, left_perp) > 0)
                    right_threats = np.sum(np.dot(threats, right_perp) > 0)
                    
                    chosen_perp = left_perp if left_threats <= right_threats else right_perp
                    dodge_vector = chosen_perp * 4.0
        
        # Target point is 5m ahead plus dodge
        target_vec = direction * 5.0 + dodge_vector
        target_pos = self.current_pos + target_vec
        
        if np.linalg.norm(dodge_vector) < 0.1 and dist_to_goal < 5.0:
            target_pos = self.goal_pos
            
        cmd = PoseStamped()
        cmd.header.frame_id = "odom"
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.pose.position.x = float(target_pos[0])
        cmd.pose.position.y = float(target_pos[1])
        cmd.pose.position.z = float(target_pos[2])
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
