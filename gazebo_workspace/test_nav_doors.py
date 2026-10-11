#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from lifecycle_msgs.srv import GetState
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from collections import deque
import subprocess
import time
import math
import sys
import json

class DoorTester(Node):
    def __init__(self):
        super().__init__('door_tester', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 1)
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_cb, qos)
        
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.amcl_pose = None
        self.costmap = None
        self.current_odom = None
        
        self.odom_history = deque(maxlen=200)      # ~10s at 20Hz
        self.cmd_vel_history = deque(maxlen=200)  # ~10s at 20Hz
        
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.tracking = False

    def odom_cb(self, msg):
        self.current_odom = msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        now = self.get_clock().now().nanoseconds / 1e9
        self.odom_history.append((now, x, y, vx, wz))
        if self.tracking:
            if 1.85 < abs(y) < 2.15:
                self.min_x = min(self.min_x, x)
                self.max_x = max(self.max_x, x)

    def cmd_vel_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        self.cmd_vel_history.append((now, msg.linear.x, msg.angular.z))

    def amcl_cb(self, msg):
        self.amcl_pose = msg

    def costmap_cb(self, msg):
        self.costmap = msg

    def check_readiness(self):
        print("Checking readiness (timeout 120s)...", flush=True)
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

        print("Waiting for /amcl_pose...", flush=True)
        while self.amcl_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > 120:
                print("TIMEOUT: AMCL pose not received", flush=True)
                sys.exit(1)
        print(f"AMCL pose received. (Took {time.time()-start_time:.1f}s)", flush=True)
        print("All readiness checks passed!\n", flush=True)

    def count_costmap_cells(self):
        if not self.costmap:
            return (-1, -1, -1, -1)
        data = self.costmap.data
        total = len(data)
        non_free = sum(1 for c in data if c > 0)
        free = sum(1 for c in data if c == 0)
        unknown = sum(1 for c in data if c < 0)
        return (non_free, free, unknown, total)

    def call_clear_costmap(self, service_name):
        cli = self.create_client(ClearEntireCostmap, service_name)
        if not cli.wait_for_service(timeout_sec=5.0):
            print(f"Failed to clear {service_name}: service not available", flush=True)
            return False
        req = ClearEntireCostmap.Request()
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        return res is not None

    def clear_costmaps_and_report(self):
        nf_before, f_before, u_before, t_before = self.count_costmap_cells()
        g_ok = self.call_clear_costmap('/global_costmap/clear_entirely_global_costmap')
        l_ok = self.call_clear_costmap('/local_costmap/clear_entirely_local_costmap')
        
        t_wait = time.time()
        while time.time() - t_wait < 1.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        nf_after, f_after, u_after, t_after = self.count_costmap_cells()
        print(f"Costmap clear results: Global clear success={g_ok}, Local clear success={l_ok}", flush=True)
        print(f"Global costmap cell counts: non-free before={nf_before}, after={nf_after} | free before={f_before}, after={f_after}", flush=True)
        return g_ok and l_ok, nf_before, nf_after

    def set_robot_pose(self, x, y, yaw):
        print(f"\nSetting robot pose to x={x}, y={y}, yaw={yaw:.4f} via ign service...", flush=True)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        req = f'name: "simple_robot", position: {{x: {x}, y: {y}, z: 0.15}}, orientation: {{x: 0.0, y: 0.0, z: {sy}, w: {cy}}}'
        subprocess.run(['ign', 'service', '-s', '/world/proxy_map/set_pose', '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout', '1000', '--req', req], check=True)
        
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01
        
        settled = False
        for attempt in range(5):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            t_start = time.time()
            while time.time() - t_start < 2.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.amcl_pose:
                    ax = self.amcl_pose.pose.pose.position.x
                    ay = self.amcl_pose.pose.pose.position.y
                    dist = math.hypot(ax - x, ay - y)
                    if dist < 0.2:
                        settled = True
                        print(f"AMCL settled at x={ax:.3f}, y={ay:.3f} (dist={dist:.3f}m, attempt {attempt+1})", flush=True)
                        break
            if settled:
                break
                
        ax = self.amcl_pose.pose.pose.position.x if self.amcl_pose else float('nan')
        ay = self.amcl_pose.pose.pose.position.y if self.amcl_pose else float('nan')
        final_dist = math.hypot(ax - x, ay - y)
        print(f"AMCL settlement verification: dist={final_dist:.3f}m (settled={settled}, threshold=0.3m)", flush=True)
        
        self.clear_costmaps_and_report()
        return final_dist <= 0.3

    def run_test(self, run_id, door_name, start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw):
        print(f"\n==========================================", flush=True)
        print(f"=== {run_id}: {door_name} ===", flush=True)
        print(f"==========================================", flush=True)
        
        valid_pose = self.set_robot_pose(start_x, start_y, start_yaw)
        
        amcl_x = self.amcl_pose.pose.pose.position.x if self.amcl_pose else float('nan')
        amcl_y = self.amcl_pose.pose.pose.position.y if self.amcl_pose else float('nan')
        amcl_z = self.amcl_pose.pose.pose.orientation.z if self.amcl_pose else float('nan')
        amcl_w = self.amcl_pose.pose.pose.orientation.w if self.amcl_pose else float('nan')
        amcl_yaw = 2.0 * math.atan2(amcl_z, amcl_w)
        print(f"AMCL Pose before goal: x={amcl_x:.3f}, y={amcl_y:.3f}, yaw={amcl_yaw:.4f}", flush=True)
        
        if not valid_pose:
            print(f"ERROR: AMCL settled > 0.3m from true pose! Marking run INVALID.", flush=True)
            return {
                "run": run_id,
                "door": door_name,
                "start_pose": (start_x, start_y, start_yaw),
                "amcl_pose": (amcl_x, amcl_y, amcl_yaw),
                "goal_pose": (goal_x, goal_y, goal_yaw),
                "status": "INVALID",
                "final_odom": (float('nan'), float('nan'), float('nan')),
                "dist_to_goal": float('nan'),
                "min_x": float('nan'),
                "max_x": float('nan'),
                "time_taken": 0.0
            }

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(goal_x)
        goal_msg.pose.pose.position.y = float(goal_y)
        gz = math.sin(goal_yaw * 0.5)
        gw = math.cos(goal_yaw * 0.5)
        goal_msg.pose.pose.orientation.z = gz
        goal_msg.pose.pose.orientation.w = gw
        
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.tracking = True
        
        print(f"Sending goal: x={goal_x:.2f}, y={goal_y:.2f}, yaw={goal_yaw:.4f} (direction of travel)", flush=True)
        t0 = time.time()
        send_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        
        if not goal_handle.accepted:
            print("Goal REJECTED by Nav2!", flush=True)
            self.tracking = False
            return {
                "run": run_id,
                "door": door_name,
                "start_pose": (start_x, start_y, start_yaw),
                "amcl_pose": (amcl_x, amcl_y, amcl_yaw),
                "goal_pose": (goal_x, goal_y, goal_yaw),
                "status": "REJECTED",
                "final_odom": (float('nan'), float('nan'), float('nan')),
                "dist_to_goal": float('nan'),
                "min_x": float('nan'),
                "max_x": float('nan'),
                "time_taken": time.time() - t0
            }

        result_future = goal_handle.get_result_async()
        timed_out = False
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > 90.0:
                print("HARD TIMEOUT: 90s exceeded! Canceling goal...", flush=True)
                timed_out = True
                cancel_fut = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_fut)
                t_c = time.time()
                while not result_future.done() and (time.time() - t_c < 5.0):
                    rclpy.spin_once(self, timeout_sec=0.1)
                break
                
        t_taken = time.time() - t0
        self.tracking = False
        
        if timed_out:
            final_status = "TIMEOUT"
        else:
            res = result_future.result()
            st = res.status if res else -1
            if st == 4:
                final_status = "SUCCEEDED"
            elif st == 6:
                final_status = "ABORTED"
            elif st == 5:
                final_status = "CANCELED"
            else:
                final_status = f"STATUS_{st}"
                
        print(f"Goal finished with status: {final_status} in {t_taken:.2f}s", flush=True)

        if self.current_odom:
            fx = self.current_odom.pose.pose.position.x
            fy = self.current_odom.pose.pose.position.y
            oz = self.current_odom.pose.pose.orientation.z
            ow = self.current_odom.pose.pose.orientation.w
            fyaw = 2.0 * math.atan2(oz, ow)
            dist = math.hypot(fx - goal_x, fy - goal_y)
        else:
            fx, fy, fyaw, dist = float('nan'), float('nan'), float('nan'), float('nan')
            
        rec_min_x = self.min_x if self.min_x != float('inf') else float('nan')
        rec_max_x = self.max_x if self.max_x != float('-inf') else float('nan')
        
        print(f"Final /odom pose: x={fx:.3f}, y={fy:.3f}, yaw={fyaw:.3f}", flush=True)
        print(f"Distance to goal: {dist:.3f} m", flush=True)
        if not math.isnan(rec_min_x):
            print(f"Min/Max x in 1.85<|y|<2.15: min_x={rec_min_x:.3f}, max_x={rec_max_x:.3f}", flush=True)
        else:
            print("Min/Max x in 1.85<|y|<2.15: NO CROSSING RECORDED", flush=True)

        if final_status != "SUCCEEDED":
            safe_run_id = run_id.replace(" ", "_")
            dump_path = f"/tmp/cmd_vel_odom_{safe_run_id}.txt"
            with open(dump_path, 'w') as f:
                f.write(f"=== ODOM HISTORY (last 10s) for {run_id} ===\n")
                for item in list(self.odom_history):
                    f.write(f"t={item[0]:.2f} x={item[1]:.3f} y={item[2]:.3f} vx={item[3]:.3f} wz={item[4]:.3f}\n")
                f.write(f"\n=== CMD_VEL HISTORY (last 10s) for {run_id} ===\n")
                for item in list(self.cmd_vel_history):
                    f.write(f"t={item[0]:.2f} vx={item[1]:.3f} wz={item[2]:.3f}\n")
            print(f"Captured 10s /cmd_vel and /odom to {dump_path}", flush=True)

        return {
            "run": run_id,
            "door": door_name,
            "start_pose": (start_x, start_y, start_yaw),
            "amcl_pose": (amcl_x, amcl_y, amcl_yaw),
            "goal_pose": (goal_x, goal_y, goal_yaw),
            "status": final_status,
            "final_odom": (fx, fy, fyaw),
            "dist_to_goal": dist,
            "min_x": rec_min_x,
            "max_x": rec_max_x,
            "time_taken": t_taken
        }

def main():
    rclpy.init()
    tester = DoorTester()
    
    print("Waiting for clock...", flush=True)
    while tester.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(tester, timeout_sec=0.1)
        
    tester.check_readiness()
    
    results = []
    
    # 1. top_1 (x=-9, y=2.0) -> goal in room at (-9, 4.0), travel facing +y (yaw=+1.5708)
    for i in range(1, 4):
        res = tester.run_test(f"top_1 Run {i}", "top_1", -9.0, 0.0, 1.5708, -9.0, 4.0, 1.5708)
        results.append(res)
        
    # 2. bot_1 (x=-9, y=-2.0) -> goal in room at (-9, -4.0), travel facing -y (yaw=-1.5708)
    for i in range(1, 4):
        res = tester.run_test(f"bot_1 Run {i}", "bot_1", -9.0, 0.0, -1.5708, -9.0, -4.0, -1.5708)
        results.append(res)
        
    # 3. bot_3 (x=3, y=-2.0) -> goal in room at (3.0, -4.0), travel facing -y (yaw=-1.5708)
    for i in range(1, 4):
        res = tester.run_test(f"bot_3 Run {i}", "bot_3", 3.0, 0.0, -1.5708, 3.0, -4.0, -1.5708)
        results.append(res)
        
    print("\n\n=======================================================")
    print("FINAL SUMMARY OF ALL 9 RUNS")
    print("=======================================================")
    print("| Run | Door | Start Pose (x,y,yaw) | AMCL Pose Before (x,y,yaw) | Goal Pose (x,y,yaw) | Final Status | Final /odom (x,y,yaw) | Dist to Goal (m) | Min/Max x (1.85<|y|<2.15) | Time (s) |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for r in results:
        sp = f"({r['start_pose'][0]:.1f}, {r['start_pose'][1]:.1f}, {r['start_pose'][2]:.2f})"
        ap = f"({r['amcl_pose'][0]:.2f}, {r['amcl_pose'][1]:.2f}, {r['amcl_pose'][2]:.2f})"
        gp = f"({r['goal_pose'][0]:.1f}, {r['goal_pose'][1]:.1f}, {r['goal_pose'][2]:.2f})"
        fo = f"({r['final_odom'][0]:.2f}, {r['final_odom'][1]:.2f}, {r['final_odom'][2]:.2f})" if not math.isnan(r['final_odom'][0]) else "N/A"
        dg = f"{r['dist_to_goal']:.3f}" if not math.isnan(r['dist_to_goal']) else "N/A"
        mm = f"[{r['min_x']:.3f}, {r['max_x']:.3f}]" if not math.isnan(r['min_x']) else "N/A"
        tt = f"{r['time_taken']:.2f}"
        print(f"| {r['run']} | {r['door']} | {sp} | {ap} | {gp} | {r['status']} | {fo} | {dg} | {mm} | {tt} |")

    with open('/tmp/door_test_results.json', 'w') as f:
        json.dump(results, f, indent=2)
    print("\nResults saved to /tmp/door_test_results.json", flush=True)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
