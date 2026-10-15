#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from hunav_msgs.msg import Agents
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import subprocess
import time
import math
import sys

class VisibilityChecker(Node):
    def __init__(self):
        super().__init__('visibility_checker', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.agents_sub = self.create_subscription(Agents, '/human_states', self.agents_cb, 10)
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap_cb, qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.current_scan = None
        self.current_agents = None
        self.current_costmap = None

    def scan_cb(self, msg):
        self.current_scan = msg

    def agents_cb(self, msg):
        self.current_agents = msg

    def costmap_cb(self, msg):
        self.current_costmap = msg

    def wait_for_data(self):
        print("Waiting for /scan, /human_states, and /local_costmap/costmap...", flush=True)
        t0 = time.time()
        while self.current_scan is None or self.current_agents is None or self.current_costmap is None:
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - t0 > 60:
                print(f"TIMEOUT waiting for topics: scan={self.current_scan is not None}, agents={self.current_agents is not None}, costmap={self.current_costmap is not None}")
                sys.exit(1)
        print("All topics received!", flush=True)

    def set_robot_pose(self, x, y, yaw):
        print(f"Teleporting robot to x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} via ign service...", flush=True)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        req = f'name: "simple_robot", position: {{x: {x}, y: {y}, z: 0.15}}, orientation: {{x: 0.0, y: 0.0, z: {sy}, w: {cy}}}'
        subprocess.run(['ign', 'service', '-s', '/world/proxy_map/set_pose', '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout', '2000', '--req', req], check=True, stdout=subprocess.DEVNULL)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01

        for _ in range(5):
            self.initial_pose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def clear_local_costmap(self):
        cli = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        if cli.wait_for_service(timeout_sec=5.0):
            req = ClearEntireCostmap.Request()
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut)

    def get_costmap_value(self, wx, wy):
        if not self.current_costmap:
            return -1
        info = self.current_costmap.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)
        if 0 <= gx < info.width and 0 <= gy < info.height:
            return self.current_costmap.data[gy * info.width + gx]
        return -1

def test_visibility_at_distance(node, target_agent_name, target_dist):
    # Find target agent
    target_agent = None
    for a in node.current_agents.agents:
        if a.name == target_agent_name:
            target_agent = a
            break
    if not target_agent and len(node.current_agents.agents) > 0:
        target_agent = node.current_agents.agents[0]

    ax = target_agent.position.position.x
    ay = target_agent.position.position.y

    # Position robot target_dist meters in front of agent, facing towards +x (facing agent)
    rx = ax - target_dist
    ry = ay
    ryaw = 0.0
    node.set_robot_pose(rx, ry, ryaw)
    node.clear_local_costmap()

    # Wait 0.4s sim time (enough for 4 scan cycles and 2 costmap cycles, but agent moves <0.4m)
    t0 = node.get_clock().now()
    while (node.get_clock().now() - t0).nanoseconds < 0.4e9:
        rclpy.spin_once(node, timeout_sec=0.05)

    # Re-read agent state
    for a in node.current_agents.agents:
        if a.id == target_agent.id:
            target_agent = a
            break

    ax_now = target_agent.position.position.x
    ay_now = target_agent.position.position.y
    actual_dist = math.hypot(ax_now - rx, ay_now - ry)
    actual_bearing = math.atan2(ay_now - ry, ax_now - rx) - ryaw

    print(f"\n=======================================================", flush=True)
    print(f"=== TEST AT NOMINAL DISTANCE {target_dist:.1f} m ===", flush=True)
    print(f"=======================================================", flush=True)
    print(f"Agent: {target_agent.name} (id={target_agent.id}) at ({ax_now:.3f}, {ay_now:.3f})")
    print(f"Robot: ({rx:.3f}, {ry:.3f}, yaw={ryaw:.3f})")
    print(f"Actual Distance: {actual_dist:.3f} m, Bearing: {math.degrees(actual_bearing):.2f} deg ({actual_bearing:.4f} rad)")

    # 1. /scan check
    scan = node.current_scan
    idx = int((actual_bearing - scan.angle_min) / scan.angle_increment)
    print(f"\n--- 1. LiDAR /scan Check ---")
    print(f"LiDAR rays: {len(scan.ranges)}, min_angle={scan.angle_min:.3f}, max_angle={scan.angle_max:.3f}, inc={scan.angle_increment:.5f}")
    print(f"Center ray for agent bearing: index {idx}")
    print("Ranges around bearing (-5 to +5 rays):")
    hit_human = False
    for off in range(-5, 6):
        i = idx + off
        if 0 <= i < len(scan.ranges):
            r = scan.ranges[i]
            deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            marker = ""
            if abs(r - actual_dist) < 0.5:
                marker = " <-- [HIT HUMAN BODY/LEGS]"
                hit_human = True
            elif r > actual_dist + 1.0:
                marker = " <-- [MISSED / PASSED THROUGH TO WALL]"
            print(f"  Ray {i:3d} ({deg:+6.2f}°): {r:.3f} m{marker}")

    # 2. Local Costmap check
    costmap = node.current_costmap
    print(f"\n--- 2. Local Costmap Check ---")
    info = costmap.info
    print(f"Costmap: size={info.width}x{info.height}, res={info.resolution:.3f}m, origin=({info.origin.position.x:.2f}, {info.origin.position.y:.2f})")
    print(f"Robot-relative window extent: X=[{info.origin.position.x:.2f}, {info.origin.position.x + info.width*info.resolution:.2f}], Y=[{info.origin.position.y:.2f}, {info.origin.position.y + info.height*info.resolution:.2f}]")
    
    c_val = node.get_costmap_value(ax_now, ay_now)
    print(f"Costmap value at agent center ({ax_now:.2f}, {ay_now:.2f}): {c_val}")

    # Bounding neighborhood 0.8m x 0.8m
    max_cost = -1
    occupied_count = 0
    grid_rows = []
    dy_steps = [0.3, 0.2, 0.1, 0.0, -0.1, -0.2, -0.3]
    dx_steps = [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3]
    for dy in dy_steps:
        row = []
        for dx in dx_steps:
            v = node.get_costmap_value(ax_now + dx, ay_now + dy)
            if v > max_cost:
                max_cost = v
            if v > 0:
                occupied_count += 1
            row.append(f"{v:3d}" if v >= 0 else " --")
        grid_rows.append(" ".join(row))

    print(f"Occupied cells in agent neighborhood: {occupied_count} / {len(dy_steps)*len(dx_steps)}, max_cost={max_cost}")
    print("Costmap 7x7 patch around agent (rows=Y descending, cols=X ascending):")
    for r in grid_rows:
        print(f"  {r}")

    return hit_human, occupied_count, actual_dist

def main():
    rclpy.init()
    node = VisibilityChecker()

    print("Waiting for clock...", flush=True)
    while node.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.wait_for_data()

    # Test 1: User requested 2.0 m
    hit1, occ1, d1 = test_visibility_at_distance(node, 'agent6', 2.0)

    # Test 2: At 1.2 m (safely inside local costmap 1.5m half-width)
    hit2, occ2, d2 = test_visibility_at_distance(node, 'agent6', 1.2)

    print("\n=======================================================", flush=True)
    print("=== SUMMARY OF FINDINGS ===", flush=True)
    print("=======================================================", flush=True)
    print(f"Nominal 2.0m (actual {d1:.2f}m): LiDAR Hit={hit1}, Costmap Occupied Cells={occ1}")
    print(f"Nominal 1.2m (actual {d2:.2f}m): LiDAR Hit={hit2}, Costmap Occupied Cells={occ2}")
    print("=======================================================\n", flush=True)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
