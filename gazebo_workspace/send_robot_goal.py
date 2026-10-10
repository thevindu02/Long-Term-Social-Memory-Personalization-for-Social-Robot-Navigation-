#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
import sys

class GoalSender(Node):
    def __init__(self, target_x, target_y):
        super().__init__('goal_sender')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.target_x = target_x
        self.target_y = target_y
        
        # Wait a bit for connections to establish
        time.sleep(1.0)
        self.send_goal()
        
    def send_goal(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.position.x = float(self.target_x)
        msg.pose.position.y = float(self.target_y)
        msg.pose.position.z = 0.0
        
        # Facing forward (no rotation)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published goal: x={self.target_x}, y={self.target_y}')

def main():
    rclpy.init()
    
    # Default target if none provided
    target_x = 4.0
    target_y = 0.0
    
    if len(sys.argv) >= 3:
        target_x = float(sys.argv[1])
        target_y = float(sys.argv[2])
        
    goal_sender = GoalSender(target_x, target_y)
    
    # Spin briefly to ensure message goes out
    rclpy.spin_once(goal_sender, timeout_sec=2.0)
    
    goal_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
