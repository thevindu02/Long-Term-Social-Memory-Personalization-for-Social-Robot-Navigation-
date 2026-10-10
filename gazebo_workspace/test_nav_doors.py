#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.action import ActionClient
import sys
import subprocess
import time

from rclpy.parameter import Parameter

class DoorTester(Node):
    def __init__(self):
        super().__init__('door_tester', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.tracking = False

    def odom_callback(self, msg):
        if not self.tracking:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if 1.85 < abs(y) < 2.15:
            self.min_x = min(self.min_x, x)
            self.max_x = max(self.max_x, x)

    def set_robot_pose(self, x, y, yaw):
        print(f"Setting robot pose to x={x}, y={y}, yaw={yaw} via ign service...")
        # Simplistic yaw to quaternion (only yaw=0, +-1.57, +-3.14 used here typically)
        import math
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        req = f'name: "simple_robot", position: {{x: {x}, y: {y}, z: 0.15}}, orientation: {{x: 0.0, y: 0.0, z: {sy}, w: {cy}}}'
        subprocess.run(['ign', 'service', '-s', '/world/proxy_map/set_pose', '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout', '1000', '--req', req], check=True)
        
        # Publish initial pose to AMCL
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy
        # Some small covariance
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.05
        
        # publish a few times to ensure amcl gets it
        for _ in range(3):
            self.initial_pose_pub.publish(msg)
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.1)

        print("Waiting 3s for AMCL and Costmaps to settle...")
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_test(self, door_name, start_x, start_y, start_yaw, goal_x, goal_y):
        print(f"--- Starting test for {door_name} ---")
        self.set_robot_pose(start_x, start_y, start_yaw)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        while not self.nav_client.server_is_ready():
            print("Waiting for action server...")
            rclpy.spin_once(self, timeout_sec=1.0)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(goal_x)
        goal_msg.pose.pose.position.y = float(goal_y)
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.tracking = True
        
        print(f"Sending goal: {goal_x}, {goal_y}")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            print("Goal rejected!")
            return
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        
        self.tracking = False
        print(f"Goal finished with status: {result.status}")
        print(f"Min/Max X while 1.85 < |y| < 2.15: min_x = {self.min_x}, max_x = {self.max_x}")
        print("--------------------------------------")

def main():
    rclpy.init()
    tester = DoorTester()
    
    print("Waiting for clock...")
    while tester.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(tester, timeout_sec=0.1)
    
    # 3 runs per door
    # top_1 (x=-9, y=2.0)
    for i in range(3):
        print(f"Run {i+1} for top_1")
        tester.run_test("top_1", -9.0, 0.0, 1.57, -9.0, 4.0)
        
    # bot_1 (x=-9, y=-2.0)
    for i in range(3):
        print(f"Run {i+1} for bot_1")
        tester.run_test("bot_1", -9.0, 0.0, -1.57, -9.0, -4.0)
        
    # bot_3 (x=3, y=-2.0)
    for i in range(3):
        print(f"Run {i+1} for bot_3")
        tester.run_test("bot_3", 3.0, 0.0, -1.57, 3.0, -4.0)
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
