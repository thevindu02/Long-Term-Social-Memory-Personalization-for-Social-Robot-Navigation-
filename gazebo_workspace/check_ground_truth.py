#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from hunav_msgs.msg import Agent
import time
import math
import json

class GroundTruthChecker(Node):
    def __init__(self):
        super().__init__('ground_truth_checker', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_states_sub = self.create_subscription(Agent, '/robot_states', self.robot_states_cb, 50)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 50)

        self.robot_states_log = [] # (sim_time, x, y, yaw)
        self.odom_log = []         # (sim_time, x, y, yaw)

    def robot_states_cb(self, msg):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        x = msg.position.position.x
        y = msg.position.position.y
        yaw = msg.yaw
        self.robot_states_log.append((now_sec, x, y, yaw))

    def odom_cb(self, msg):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        self.odom_log.append((now_sec, x, y, yaw))

def main():
    rclpy.init()
    node = GroundTruthChecker()

    print("Waiting for clock and initial messages on /robot_states and /odom...", flush=True)
    t0_wall = time.time()
    while (node.get_clock().now().nanoseconds == 0 or 
           len(node.robot_states_log) == 0 or 
           len(node.odom_log) == 0):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0_wall > 60:
            print("TIMEOUT waiting for initial topics!")
            rclpy.shutdown()
            return

    print("Initial messages received! Clearing buffers for measurement window...", flush=True)
    node.robot_states_log.clear()
    node.odom_log.clear()

    # Robot starts at (0, 0, 0)
    x0, y0, yaw0 = 0.0, 0.0, 0.0

    t_start_sim = node.get_clock().now().nanoseconds / 1e9
    target_drive_duration = 4.0 # 4s drive forward
    target_total_duration = 6.0 # 6s total observation

    cmd = Twist()
    cmd.linear.x = 0.2 # 0.2 m/s

    # Drive phase
    print(f"Driving robot forward at 0.2 m/s for {target_drive_duration}s sim time...", flush=True)
    while True:
        now_sim = node.get_clock().now().nanoseconds / 1e9
        elapsed = now_sim - t_start_sim
        if elapsed >= target_drive_duration:
            break
        node.cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.02)

    # Stop phase
    cmd.linear.x = 0.0
    node.cmd_pub.publish(cmd)
    print(f"Stopping robot. Observing settling until {target_total_duration}s sim time...", flush=True)
    while True:
        now_sim = node.get_clock().now().nanoseconds / 1e9
        elapsed = now_sim - t_start_sim
        if elapsed >= target_total_duration:
            break
        node.cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.02)

    t_end_sim = node.get_clock().now().nanoseconds / 1e9
    total_sim_time = t_end_sim - t_start_sim

    rs_samples = len(node.robot_states_log)
    odom_samples = len(node.odom_log)

    rs_rate = rs_samples / total_sim_time if total_sim_time > 0 else 0
    odom_rate = odom_samples / total_sim_time if total_sim_time > 0 else 0

    print("\n=======================================================", flush=True)
    print("=== GROUND TRUTH VERIFICATION REPORT ===", flush=True)
    print("=======================================================", flush=True)
    print(f"Window Sim Duration: {total_sim_time:.2f} s")
    print(f"/robot_states total samples: {rs_samples} ({rs_rate:.2f} Hz in sim time)")
    print(f"/odom total samples: {odom_samples} ({odom_rate:.2f} Hz in sim time)")

    # Compute agreement between /robot_states and /odom + start_pose
    diffs = []
    # Match each /robot_states sample to closest /odom sample
    for (t_rs, rx, ry, ryaw) in node.robot_states_log:
        best_e = float('inf')
        for (t_od, ox, oy, oyaw) in node.odom_log:
            if abs(t_od - t_rs) < 0.05: # within 50ms
                # Transform odom to world using known start pose (x0, y0, yaw0)
                ow_x = x0 + ox * math.cos(yaw0) - oy * math.sin(yaw0)
                ow_y = y0 + ox * math.sin(yaw0) + oy * math.cos(yaw0)
                e = math.hypot(rx - ow_x, ry - ow_y)
                if e < best_e:
                    best_e = e
        if best_e != float('inf'):
            diffs.append(best_e)

    max_diff = max(diffs) if diffs else float('nan')
    mean_diff = sum(diffs) / len(diffs) if diffs else float('nan')
    print(f"\nPose Agreement (/robot_states vs /odom + start_pose):")
    print(f"  Matched samples: {len(diffs)} / {rs_samples}")
    print(f"  Mean difference: {mean_diff*1000:.2f} mm")
    print(f"  Max difference:  {max_diff*1000:.2f} mm ({max_diff:.4f} m)")

    # Downsampled 10 Hz check
    dt_target = 0.1
    expected_10hz_samples = int(round(total_sim_time / dt_target))
    print(f"\n10 Hz Sampling Grid Check:")
    print(f"  Expected samples at 10 Hz: {expected_10hz_samples}")
    print(f"  Actual raw samples available: {rs_samples} (sufficient: {rs_samples >= expected_10hz_samples})")

    passed = (max_diff <= 0.05)
    print(f"\nAgreement within 0.05m (50 mm): {'PASS' if passed else 'FAIL'}")
    print("=======================================================\n", flush=True)

    report = {
        "sim_duration": total_sim_time,
        "robot_states_samples": rs_samples,
        "robot_states_rate_hz": rs_rate,
        "odom_samples": odom_samples,
        "odom_rate_hz": odom_rate,
        "mean_diff_m": mean_diff,
        "max_diff_m": max_diff,
        "agrees_within_0_05m": passed,
        "expected_10hz_samples": expected_10hz_samples
    }
    with open('/tmp/ground_truth_report.json', 'w') as f:
        json.dump(report, f, indent=2)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
