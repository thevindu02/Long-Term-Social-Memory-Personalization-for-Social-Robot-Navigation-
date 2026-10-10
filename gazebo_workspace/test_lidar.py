#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import subprocess
import time
import math

from rclpy.parameter import Parameter

class LidarTester(Node):
    def __init__(self):
        super().__init__('lidar_tester', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.current_scan = None

    def scan_callback(self, msg):
        self.current_scan = msg

    def set_robot_pose(self, x, y, yaw):
        print(f"Setting robot pose to x={x}, y={y}, yaw={yaw} via ign service...")
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        req = f'name: "simple_robot", position: {{x: {x}, y: {y}, z: 0.15}}, orientation: {{x: 0.0, y: 0.0, z: {sy}, w: {cy}}}'
        subprocess.run(['ign', 'service', '-s', '/world/proxy_map/set_pose', '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout', '1000', '--req', req], check=True)
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)

    def get_forward_range(self):
        if not self.current_scan:
            return None
        # Assuming 360 samples, min_angle = -PI, max_angle = PI
        # Forward is at index 180
        center_idx = len(self.current_scan.ranges) // 2
        return self.current_scan.ranges[center_idx]

    def get_door_edge_jump(self):
        if not self.current_scan:
            return None
        # Look for a jump in the scan array (e.g. hitting wall vs passing through door)
        ranges = self.current_scan.ranges
        jumps = []
        for i in range(1, len(ranges)):
            if abs(ranges[i] - ranges[i-1]) > 0.5 and ranges[i] < 19.0 and ranges[i-1] < 19.0:
                jumps.append((i, ranges[i-1], ranges[i]))
        return jumps

def main():
    rclpy.init()
    tester = LidarTester()
    
    print("Waiting for clock...")
    while tester.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(tester, timeout_sec=0.1)
    
    # 1. Test at (-9, 0) facing +y (yaw=1.57)
    tester.set_robot_pose(-9.0, 0.0, 1.57)
    for _ in range(20):
        rclpy.spin_once(tester, timeout_sec=0.1)
    
    forward_range = tester.get_forward_range()
    print(f"--- LiDAR Test at (-9.0, 0.0) facing +y ---")
    print(f"Measured straight-ahead range: {forward_range:.3f} m (Expected: ~7.7 m)")
    
    jumps = tester.get_door_edge_jump()
    if jumps:
        print("Detected range jumps (door edges):")
        for j in jumps[:5]:
            print(f"  Index {j[0]}: {j[1]:.2f}m -> {j[2]:.2f}m")
    
    # 2. Test at (0, 0) facing +y (yaw=1.57)
    tester.set_robot_pose(0.0, 0.0, 1.57)
    for _ in range(20):
        rclpy.spin_once(tester, timeout_sec=0.1)
    
    forward_range = tester.get_forward_range()
    print(f"--- LiDAR Test at (0.0, 0.0) facing +y ---")
    print(f"Measured straight-ahead range: {forward_range:.3f} m (Expected: ~1.7 m)")
    
    jumps = tester.get_door_edge_jump()
    if jumps:
        print("Detected range jumps (door edges):")
        for j in jumps[:5]:
            print(f"  Index {j[0]}: {j[1]:.2f}m -> {j[2]:.2f}m")
            
    rclpy.shutdown()

if __name__ == '__main__':
    main()
