#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from hunav_msgs.msg import Agents
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import time
import math
import sys

class CostmapDirectCheck(Node):
    def __init__(self):
        super().__init__('costmap_direct_check', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.agents_sub = self.create_subscription(Agents, '/human_states', self.agents_cb, 10)
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap_cb, qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_scan = None
        self.current_agents = None
        self.current_costmap = None

    def scan_cb(self, msg):
        self.current_scan = msg

    def agents_cb(self, msg):
        self.current_agents = msg

    def costmap_cb(self, msg):
        self.current_costmap = msg

def main():
    rclpy.init()
    node = CostmapDirectCheck()

    print("Waiting for sim time and topics...", flush=True)
    while node.get_clock().now().nanoseconds == 0 or node.current_costmap is None or node.current_agents is None or node.current_scan is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("Ready! Robot is at natural spawn (0, 0). Tracking humans for 10 sim seconds...", flush=True)

    t_start = node.get_clock().now()
    closest_dist = float('inf')
    closest_agent = None

    # Drive forward slowly (0.2 m/s) towards corridor agents to meet them
    cmd = Twist()
    cmd.linear.x = 0.2

    sample_count = 0
    while (node.get_clock().now() - t_start).nanoseconds < 10e9:
        node.cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.1)
        sample_count += 1

        # Check all agents distance to robot
        if node.current_agents and node.current_costmap:
            cm = node.current_costmap
            info = cm.info
            # Costmap statistics
            data = cm.data
            occupied = [x for x in data if x > 0]
            max_c = max(data) if data else -1

            for a in node.current_agents.agents:
                d = math.hypot(a.position.position.x, a.position.position.y)
                if d < closest_dist:
                    closest_dist = d
                    closest_agent = a.name

                # When an agent is within 2.0m of the costmap center
                if d < 2.0 and sample_count % 10 == 0:
                    gx = int((a.position.position.x - info.origin.position.x) / info.resolution)
                    gy = int((a.position.position.y - info.origin.position.y) / info.resolution)
                    val = -1
                    if 0 <= gx < info.width and 0 <= gy < info.height:
                        val = data[gy * info.width + gx]
                    print(f"[Sim Time {(node.get_clock().now() - t_start).nanoseconds/1e9:.1f}s] Agent {a.name} at ({a.position.position.x:.2f}, {a.position.position.y:.2f}), dist={d:.2f}m | Costmap total occupied cells={len(occupied)}, max_cost={max_c}, cell_at_agent={val}", flush=True)

    # Stop robot
    cmd.linear.x = 0.0
    node.cmd_pub.publish(cmd)

    print("\n=======================================================", flush=True)
    print("=== COSTMAP DIRECT OBSERVATION SUMMARY ===", flush=True)
    print("=======================================================", flush=True)
    print(f"Closest agent observed: {closest_agent} at {closest_dist:.2f} m")
    cm = node.current_costmap
    data = cm.data
    occupied = [x for x in data if x > 0]
    print(f"Final Local Costmap: total cells={len(data)}, occupied/inflated cells={len(occupied)}, max_cost={max(data)}")
    print("=======================================================\n", flush=True)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
