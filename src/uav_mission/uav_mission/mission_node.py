#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
import time
import math
import psutil
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        
        self.state = State()
        self.current_pose = Odometry()
        self.cmd_pose = PoseStamped()
        
        self.sub_state = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.sub_odom = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.odom_cb, qos_profile)
        self.sub_cmd = self.create_subscription(PoseStamped, '/planner/cmd_pose', self.cmd_cb, 10)
        
        self.pub_setpoint = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.pub_waypoint = self.create_publisher(PoseStamped, '/mission/waypoint', 10)
        
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.waypoints = [
            (250.0, 0.0, 10.0),
            (500.0, 0.0, 10.0),
            (750.0, 0.0, 10.0),
            (1000.0, 0.0, 10.0)
        ]
        self.current_wp_idx = 0
        
        self.cmd_pose.pose.position.x = 0.0
        self.cmd_pose.pose.position.y = 0.0
        self.cmd_pose.pose.position.z = 10.0
        
        self.timer_20hz = self.create_timer(0.05, self.loop_20hz)
        self.timer_1hz = self.create_timer(1.0, self.eval_loop)
        
        self.mission_started = False
        self.distance_traveled = 0.0
        self.last_pos = None
        
        self.get_logger().info("Mission Node Started. Waiting for MAVROS...")

    def state_cb(self, msg):
        self.state = msg

    def odom_cb(self, msg):
        self.current_pose = msg
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        if self.last_pos is not None:
            self.distance_traveled += np.linalg.norm(pos - self.last_pos)
        self.last_pos = pos

    def cmd_cb(self, msg):
        self.cmd_pose = msg

    def loop_20hz(self):
        self.pub_setpoint.publish(self.cmd_pose)
        
        if not self.state.connected:
            return
            
        if not self.mission_started:
            if self.state.mode != "OFFBOARD":
                req = SetMode.Request()
                req.custom_mode = "OFFBOARD"
                self.mode_client.call_async(req)
            elif not self.state.armed:
                req = CommandBool.Request()
                req.value = True
                self.arm_client.call_async(req)
            else:
                self.get_logger().info("Offboard and Armed. Starting Mission!")
                self.mission_started = True
                
        if self.mission_started:
            if self.current_wp_idx < len(self.waypoints):
                wp = self.waypoints[self.current_wp_idx]
                wp_msg = PoseStamped()
                wp_msg.pose.position.x = wp[0]
                wp_msg.pose.position.y = wp[1]
                wp_msg.pose.position.z = wp[2]
                self.pub_waypoint.publish(wp_msg)
                
                if self.last_pos is not None:
                    dist = math.sqrt((self.last_pos[0]-wp[0])**2 + (self.last_pos[1]-wp[1])**2 + (self.last_pos[2]-wp[2])**2)
                    if dist < 5.0:
                        self.get_logger().info(f"Reached Waypoint {self.current_wp_idx+1}")
                        self.current_wp_idx += 1
            else:
                self.get_logger().info("MISSION COMPLETE. Landing...")
                req = SetMode.Request()
                req.custom_mode = "AUTO.LAND"
                self.mode_client.call_async(req)
                raise SystemExit

    def eval_loop(self):
        if self.mission_started:
            cpu = psutil.cpu_percent()
            ram = psutil.virtual_memory().used / (1024**3)
            self.get_logger().info(f"[EVAL] Dist: {self.distance_traveled:.1f}m | CPU: {cpu}% | RAM: {ram:.1f}GB")

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
