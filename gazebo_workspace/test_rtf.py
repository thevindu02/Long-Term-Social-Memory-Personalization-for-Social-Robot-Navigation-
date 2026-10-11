#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.action import ActionClient
from lifecycle_msgs.srv import GetState
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from hunav_msgs.msg import Agents
import subprocess
import time
import math
import sys
import os
import json

class RTFTester(Node):
    def __init__(self):
        super().__init__('rtf_tester', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 1)
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_cb, qos)
        self.agents_sub = self.create_subscription(Agents, '/human_states', self.agents_cb, 10)
        
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.amcl_pose = None
        self.costmap = None
        self.current_odom = None
        self.current_agents = None

    def odom_cb(self, msg):
        self.current_odom = msg

    def amcl_cb(self, msg):
        self.amcl_pose = msg

    def costmap_cb(self, msg):
        self.costmap = msg

    def agents_cb(self, msg):
        self.current_agents = msg

    def check_readiness(self):
        print("Checking Nav2 readiness (timeout 120s)...", flush=True)
        start_time = time.time()
        
        print("Publishing dummy initial pose to unblock AMCL...", flush=True)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.5
        msg.pose.covariance[7] = 0.5
        msg.pose.covariance[35] = 0.5
        
        while self.amcl_pose is None:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - start_time > 120:
                print("TIMEOUT: AMCL did not respond to initial pose", flush=True)
                sys.exit(1)
        print("AMCL unblocked.", flush=True)
        
        print("Waiting for /navigate_to_pose action server...", flush=True)
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > 120:
                print("TIMEOUT: Action server not available", flush=True)
                sys.exit(1)
        print(f"Action server ready. (Took {time.time()-start_time:.1f}s)", flush=True)
        
        print("Waiting for bt_navigator to be active...", flush=True)
        cli = self.create_client(GetState, '/bt_navigator/get_state')
        while not cli.wait_for_service(timeout_sec=1.0):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > 120:
                print("TIMEOUT: bt_navigator service not available", flush=True)
                sys.exit(1)
                
        req = GetState.Request()
        while True:
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            state = future.result().current_state.label
            if state == 'active':
                break
            rclpy.spin_once(self, timeout_sec=1.0)
            if time.time() - start_time > 120:
                print("TIMEOUT: bt_navigator did not become active", flush=True)
                sys.exit(1)
        print(f"bt_navigator active. (Took {time.time()-start_time:.1f}s)", flush=True)

        print("Waiting for /global_costmap/costmap...", flush=True)
        while self.costmap is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > 120:
                print("TIMEOUT: Costmap not received", flush=True)
                sys.exit(1)
        print(f"Costmap received. (Took {time.time()-start_time:.1f}s)", flush=True)

        print("Waiting for /human_states...", flush=True)
        while self.current_agents is None or len(self.current_agents.agents) == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > 120:
                print("TIMEOUT: Agents not received on /human_states", flush=True)
                sys.exit(1)
        print(f"Received {len(self.current_agents.agents)} agents on /human_states. (Took {time.time()-start_time:.1f}s)", flush=True)
        print("All readiness checks passed!\n", flush=True)

def main():
    rclpy.init()
    tester = RTFTester()
    
    print("Waiting for clock...", flush=True)
    while tester.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(tester, timeout_sec=0.1)
        
    tester.check_readiness()
    
    # Record initial agent positions at t=0
    # Let's get agents by id/name
    agents_t0 = {}
    for a in tester.current_agents.agents:
        agents_t0[a.name] = (a.position.position.x, a.position.position.y, a.position.position.z)
        
    print("=== Agent positions at t=0s ===", flush=True)
    # Pick 3 agents: agent1, agent2, agent3
    for name in ['agent1', 'agent2', 'agent3']:
        if name in agents_t0:
            p = agents_t0[name]
            print(f"{name}: x={p[0]:.3f}, y={p[1]:.3f}, z={p[2]:.3f}", flush=True)
        else:
            print(f"{name} not found in agents list!", flush=True)

    # Launch stats recording in background for 60s
    print("\nStarting 60s stats recording on /world/proxy_map/stats...", flush=True)
    stats_file = "/tmp/gz_stats.txt"
    if os.path.exists(stats_file):
        os.remove(stats_file)
    stats_proc = subprocess.Popen(["bash", "-c", f"timeout 60 ign topic -e -t /world/proxy_map/stats > {stats_file}"])

    # Send one long navigation goal across the corridor (e.g. to x=9.0, y=0.0)
    print("Sending long navigation goal across main corridor to x=9.0, y=0.0...", flush=True)
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = tester.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = 9.0
    goal_msg.pose.pose.position.y = 0.0
    goal_msg.pose.pose.orientation.w = 1.0 # facing +x
    
    send_fut = tester.nav_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(tester, send_fut)
    goal_handle = send_fut.result()
    if goal_handle.accepted:
        print("Goal accepted by Nav2, robot is navigating.", flush=True)
    else:
        print("WARNING: Goal rejected by Nav2!", flush=True)

    # Monitor for 60 seconds
    t_start = time.time()
    while time.time() - t_start < 60.0:
        rclpy.spin_once(tester, timeout_sec=0.2)
        # Check memory threshold (9500 MB)
        try:
            out = subprocess.check_output("free -m | awk '/Mem:/ {print $3}'", shell=True).decode().strip()
            used_mb = int(out)
            if used_mb > 9500:
                print(f"CRITICAL ERROR: Memory used ({used_mb} MB) exceeded 9500 MB threshold! Aborting!", flush=True)
                sys.exit(2)
        except Exception:
            pass

    print("\n60s window completed. Waiting for stats recorder to finalize...", flush=True)
    stats_proc.wait()

    # Capture agent positions at t=60s
    for _ in range(10):
        rclpy.spin_once(tester, timeout_sec=0.1)

    agents_t60 = {}
    for a in tester.current_agents.agents:
        agents_t60[a.name] = (a.position.position.x, a.position.position.y, a.position.position.z)

    print("\n=== Agent positions at t=60s ===", flush=True)
    movements = {}
    for name in ['agent1', 'agent2', 'agent3']:
        if name in agents_t60 and name in agents_t0:
            p0 = agents_t0[name]
            p60 = agents_t60[name]
            dist = math.hypot(p60[0] - p0[0], p60[1] - p0[1])
            movements[name] = {
                "t0": p0,
                "t60": p60,
                "displacement": dist
            }
            print(f"{name} at t=0s: ({p0[0]:.3f}, {p0[1]:.3f}) -> at t=60s: ({p60[0]:.3f}, {p60[1]:.3f}) | Displacement: {dist:.3f} m", flush=True)

    # Robot movement check
    if tester.current_odom:
        rx = tester.current_odom.pose.pose.position.x
        ry = tester.current_odom.pose.pose.position.y
        print(f"\nFinal robot /odom pose: x={rx:.3f}, y={ry:.3f}", flush=True)

    # Parse RTF values
    print("\nAnalyzing RTF from stats topic...", flush=True)
    rtf_values = []
    if os.path.exists(stats_file):
        with open(stats_file, 'r') as f:
            for line in f:
                if 'real_time_factor:' in line:
                    parts = line.split(':')
                    if len(parts) >= 2:
                        try:
                            val = float(parts[1].strip())
                            rtf_values.append(val)
                        except ValueError:
                            pass

    if rtf_values:
        mean_rtf = sum(rtf_values) / len(rtf_values)
        min_rtf = min(rtf_values)
        print(f"Total RTF samples recorded: {len(rtf_values)}", flush=True)
        print(f"Mean RTF: {mean_rtf:.4f}", flush=True)
        print(f"Minimum RTF: {min_rtf:.4f}", flush=True)
    else:
        print("ERROR: No RTF values found in stats file!", flush=True)
        mean_rtf = 0.0
        min_rtf = 0.0

    res = {
        "mean_rtf": mean_rtf,
        "min_rtf": min_rtf,
        "samples": len(rtf_values),
        "agents": movements
    }
    with open('/tmp/rtf_results.json', 'w') as f:
        json.dump(res, f, indent=2)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
