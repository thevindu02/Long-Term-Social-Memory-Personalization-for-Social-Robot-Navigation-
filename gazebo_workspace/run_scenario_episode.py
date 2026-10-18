#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from action_msgs.msg import GoalStatus
from hunav_msgs.msg import Agents, Agent
from rosgraph_msgs.msg import Clock
import subprocess
import time
import math
import argparse
import json
import os
import sys

from comfort_signals.evaluator import (
    TimeSample,
    score_episode_all_profiles,
    get_git_commit_hash,
)
from comfort_signals.signals import (
    compute_distance,
    check_contact,
    compute_polyline_length,
    project_point_to_polyline
)

class ScenarioRunner(Node):
    def __init__(self, scenario, mode, run_id, seed, jitter, timeout_sec, output_dir):
        super().__init__('scenario_runner', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.scenario = scenario
        self.mode = mode
        self.run_id = run_id
        self.seed = seed
        self.jitter = jitter
        self.timeout_sec = timeout_sec
        self.output_dir = output_dir

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.agents_sub = self.create_subscription(Agents, '/human_states', self.agents_cb, 10)
        self.robot_state_sub = self.create_subscription(Agent, '/robot_states', self.robot_state_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)

        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_cb, qos)

        self.current_agents = None
        self.current_robot_state = None
        self.current_odom = None
        self.current_amcl = None
        self.current_costmap = None

        # Episode recording buffers
        self.samples = [] # List[TimeSample]
        self.raw_human_paths = {} # agent_name -> List[(x, y)]
        self.rtf_samples = []

        self.goal_handle = None
        self.nav_done = False
        self.nav_status = "PENDING"

    def agents_cb(self, msg):
        self.current_agents = msg

    def robot_state_cb(self, msg):
        self.current_robot_state = msg

    def odom_cb(self, msg):
        self.current_odom = msg

    def amcl_cb(self, msg):
        self.current_amcl = msg

    def costmap_cb(self, msg):
        self.current_costmap = msg

    def wait_for_initial_topics(self, timeout=60.0):
        print("Waiting for clock, /robot_states, /human_states, and /odom...", flush=True)
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            ready = (self.current_agents is not None and
                     self.current_robot_state is not None and
                     self.current_odom is not None and
                     self.get_clock().now().nanoseconds > 0)
            if ready:
                print("Core topics received!", flush=True)
                return True
        print("TIMEOUT waiting for initial topics!")
        return False

    def wait_for_nav_ready(self, timeout=60.0):
        print("Waiting for NavigateToPose action server and costmap...", flush=True)
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.nav_client.wait_for_server(timeout_sec=0.2) and self.current_costmap is not None:
                print("Nav2 action server and costmap ready!", flush=True)
                return True
        print("TIMEOUT waiting for Nav2 action server!")
        return False

    def teleport_robot(self, x, y, yaw):
        print(f"Teleporting robot to x={x:.3f}, y={y:.3f}, yaw={yaw:.4f}...", flush=True)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        req = f'name: "simple_robot", position: {{x: {x}, y: {y}, z: 0.15}}, orientation: {{x: 0.0, y: 0.0, z: {sy}, w: {cy}}}'
        subprocess.run(['ign', 'service', '-s', '/world/proxy_map/set_pose', '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout', '1000', '--req', req], check=True, stdout=subprocess.DEVNULL)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01

        # Settle AMCL if mode is robot_present
        if self.mode == 'robot_present':
            settled = False
            for attempt in range(5):
                msg.header.stamp = self.get_clock().now().to_msg()
                self.initial_pose_pub.publish(msg)
                t_start = time.time()
                while time.time() - t_start < 2.0:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if self.current_amcl:
                        ax = self.current_amcl.pose.pose.position.x
                        ay = self.current_amcl.pose.pose.position.y
                        dist = math.hypot(ax - x, ay - y)
                        if dist < 0.3:
                            settled = True
                            print(f"AMCL settled at x={ax:.3f}, y={ay:.3f} (dist={dist:.3f}m, attempt {attempt+1})", flush=True)
                            break
                if settled:
                    break
        else:
            for _ in range(5):
                msg.header.stamp = self.get_clock().now().to_msg()
                self.initial_pose_pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.05)

        # Clear costmaps
        for srv_name in ['/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap']:
            cli = self.create_client(ClearEntireCostmap, srv_name)
            if cli.wait_for_service(timeout_sec=2.0):
                req_c = ClearEntireCostmap.Request()
                fut = cli.call_async(req_c)
                rclpy.spin_until_future_complete(self, fut)

        t_w = time.time()
        while time.time() - t_w < 1.0:
            rclpy.spin_once(self, timeout_sec=0.1)

    def send_nav_goal(self, gx, gy, gyaw):
        print(f"Sending NavigateToPose goal: ({gx:.2f}, {gy:.2f}, yaw={gyaw:.2f})...", flush=True)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(gx)
        goal_msg.pose.pose.position.y = float(gy)
        goal_msg.pose.pose.orientation.z = math.sin(gyaw * 0.5)
        goal_msg.pose.pose.orientation.w = math.cos(gyaw * 0.5)

        for attempt in range(5):
            fut = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, fut)
            self.goal_handle = fut.result()
            if self.goal_handle and self.goal_handle.accepted:
                print("Goal ACCEPTED by Nav2.", flush=True)
                res_fut = self.goal_handle.get_result_async()
                res_fut.add_done_callback(self.goal_result_cb)
                return True
            print(f"Goal REJECTED by Nav2 (attempt {attempt+1}/5), retrying in 1.5s...", flush=True)
            t_w = time.time()
            while time.time() - t_w < 1.5:
                rclpy.spin_once(self, timeout_sec=0.1)
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        print("Goal permanently REJECTED by Nav2!")
        self.nav_done = True
        self.nav_status = "REJECTED"
        return False

    def goal_result_cb(self, future):
        res = future.result()
        self.nav_done = True
        if res.status == GoalStatus.STATUS_SUCCEEDED:
            self.nav_status = "SUCCEEDED"
        elif res.status == GoalStatus.STATUS_ABORTED:
            self.nav_status = "ABORTED"
        elif res.status == GoalStatus.STATUS_CANCELED:
            self.nav_status = "CANCELED"
        else:
            self.nav_status = f"STATUS_{res.status}"
        print(f"Navigation goal completed with status: {self.nav_status}", flush=True)

    def record_step(self, now_sim):
        if not self.current_robot_state or not self.current_agents:
            return

        rx = self.current_robot_state.position.position.x
        ry = self.current_robot_state.position.position.y
        rvx = self.current_robot_state.velocity.linear.x
        rvy = self.current_robot_state.velocity.linear.y

        h_states = {}
        for a in self.current_agents.agents:
            ax = a.position.position.x
            ay = a.position.position.y
            avx = a.velocity.linear.x
            avy = a.velocity.linear.y
            ah = a.yaw
            h_states[a.name] = {
                'pos': (ax, ay),
                'vel': (avx, avy),
                'heading': ah
            }
            if a.name not in self.raw_human_paths:
                self.raw_human_paths[a.name] = []
            self.raw_human_paths[a.name].append((ax, ay))

        sample = TimeSample(
            t=now_sim,
            robot_pos=(rx, ry),
            robot_vel=(rvx, rvy),
            human_states=h_states
        )
        self.samples.append(sample)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True, choices=['corridor', 'doorway'])
    parser.add_argument('--mode', required=True, choices=['baseline', 'robot_present'])
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--jitter', type=float, default=0.0)
    parser.add_argument('--timeout', type=float, default=125.0)
    parser.add_argument('--output-dir', default='/home/rosdev/gazebo_workspace/evaluation_results')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    rclpy.init()

    runner = ScenarioRunner(
        scenario=args.scenario,
        mode=args.mode,
        run_id=args.run_id,
        seed=args.seed,
        jitter=args.jitter,
        timeout_sec=args.timeout,
        output_dir=args.output_dir
    )

    if not runner.wait_for_initial_topics():
        rclpy.shutdown()
        sys.exit(1)

    # Scenario parameters
    if args.scenario == 'corridor':
        rx0, ry0, ryaw0 = -9.0, 0.0, 0.0
        rgx, rgy, rgyaw = 9.0, 0.0, 0.0
        human_end_x = -8.0
    elif args.scenario == 'doorway':
        rx0, ry0, ryaw0 = 3.0, 0.0, -1.5708
        rgx, rgy, rgyaw = 3.0, -4.0, -1.5708
        human_end_y = 0.5

    # Positioning
    if args.mode == 'baseline':
        # Park robot far away in Room top_3: (0.0, 5.0) behind walls
        runner.teleport_robot(0.0, 5.0, 0.0)
        print("Robot parked at (0.0, 5.0) for robot-absent baseline run.", flush=True)
    else:
        # Robot present: teleport to start pose, then wait for Nav2 to be fully ready
        runner.teleport_robot(rx0, ry0, ryaw0)
        if not runner.wait_for_nav_ready():
            rclpy.shutdown()
            sys.exit(1)

    # Wait for clock to stabilize
    t_start_sim = runner.get_clock().now().nanoseconds / 1e9
    t_start_wall = time.time()
    last_sample_sim = t_start_sim
    last_sim_time = t_start_sim
    last_wall_time = t_start_wall

    goal_dispatched = False
    print(f"\n=== Starting Episode: {args.run_id} ({args.scenario}, mode={args.mode}, jitter={args.jitter}s) ===", flush=True)

    min_robot_human_dist = float('inf')
    contact_events = 0
    first_encounter_time = None
    first_encounter_pos = None

    while rclpy.ok():
        now_sim = runner.get_clock().now().nanoseconds / 1e9
        now_wall = time.time()
        sim_elapsed = now_sim - t_start_sim
        wall_elapsed = now_wall - t_start_wall

        # Compute instant RTF
        dt_wall = now_wall - last_wall_time
        dt_sim = now_sim - last_sim_time
        if dt_wall > 0.5:
            inst_rtf = dt_sim / dt_wall
            runner.rtf_samples.append(inst_rtf)
            last_wall_time = now_wall
            last_sim_time = now_sim

        # Jitter dispatch
        if args.mode == 'robot_present' and not goal_dispatched:
            if sim_elapsed >= args.jitter:
                print(f"[Sim {sim_elapsed:.2f}s] Dispatching Nav2 goal after jitter delay {args.jitter:.2f}s...", flush=True)
                runner.send_nav_goal(rgx, rgy, rgyaw)
                goal_dispatched = True

        # Sample at 10 Hz sim time
        if (now_sim - last_sample_sim) >= 0.1:
            runner.record_step(now_sim)
            last_sample_sim = now_sim

            # Track encounter
            if runner.current_robot_state and runner.current_agents:
                rx = runner.current_robot_state.position.position.x
                ry = runner.current_robot_state.position.position.y
                for a in runner.current_agents.agents:
                    ax = a.position.position.x
                    ay = a.position.position.y
                    d = math.hypot(rx - ax, ry - ay)
                    if d < min_robot_human_dist:
                        min_robot_human_dist = d
                    if check_contact(d):
                        contact_events += 1
                    if d < 3.0 and first_encounter_time is None and args.mode == 'robot_present':
                        first_encounter_time = sim_elapsed
                        first_encounter_pos = {"robot": (rx, ry), "human": (ax, ay), "distance": d}
                        print(f"[Sim {sim_elapsed:.1f}s] Encounter entered (d={d:.2f}m)! Robot at ({rx:.2f},{ry:.2f}), Human at ({ax:.2f},{ay:.2f})", flush=True)

        # Check termination condition
        terminated = False
        term_reason = ""

        if sim_elapsed >= args.timeout:
            terminated = True
            term_reason = f"TIMEOUT ({args.timeout}s)"
        elif args.mode == 'robot_present' and runner.nav_done:
            terminated = True
            term_reason = f"NAV_DONE ({runner.nav_status})"
        elif args.mode == 'baseline':
            # Baseline completes when agent1 reaches near goal
            if 'agent1' in runner.raw_human_paths and len(runner.raw_human_paths['agent1']) > 0:
                last_hx, last_hy = runner.raw_human_paths['agent1'][-1]
                if args.scenario == 'corridor' and last_hx <= human_end_x:
                    terminated = True
                    term_reason = f"HUMAN_REACHED_GOAL (x={last_hx:.2f})"
                elif args.scenario == 'doorway' and last_hy >= human_end_y:
                    terminated = True
                    term_reason = f"HUMAN_REACHED_GOAL (y={last_hy:.2f})"

        if terminated:
            print(f"\nEpisode finished! Reason: {term_reason} at sim {sim_elapsed:.2f}s (wall {wall_elapsed:.2f}s)", flush=True)
            break

        rclpy.spin_once(runner, timeout_sec=0.02)

    # Summary and evaluation
    mean_rtf = sum(runner.rtf_samples) / len(runner.rtf_samples) if runner.rtf_samples else 1.0
    min_rtf = min(runner.rtf_samples) if runner.rtf_samples else 1.0

    print("\n=======================================================", flush=True)
    print(f"=== EPISODE SUMMARY: {args.run_id} ===", flush=True)
    print("=======================================================", flush=True)
    print(f"Scenario: {args.scenario} | Mode: {args.mode} | Seed: {args.seed} | Jitter: {args.jitter}s")
    print(f"Sim Duration: {sim_elapsed:.2f}s | Wall Duration: {wall_elapsed:.2f}s")
    print(f"RTF: Mean={mean_rtf:.3f}, Min={min_rtf:.3f}")
    print(f"Samples collected: {len(runner.samples)} (expected at 10Hz: {int(round(sim_elapsed*10))})")
    print(f"Min Robot-Human Distance: {min_robot_human_dist:.3f} m")
    print(f"Contact Events (d < 0.60m): {contact_events}")
    if first_encounter_pos:
        print(f"First Encounter at t={first_encounter_time:.2f}s: dist={first_encounter_pos['distance']:.2f}m")
        print(f"  Robot: {first_encounter_pos['robot']}, Human: {first_encounter_pos['human']}")

    # Save trajectory points
    traj_path = os.path.join(args.output_dir, f"trajectory_{args.scenario}_{args.mode}_{args.run_id}.json")
    traj_data = {
        "scenario": args.scenario,
        "mode": args.mode,
        "run_id": args.run_id,
        "seed": args.seed,
        "jitter": args.jitter,
        "sim_duration": sim_elapsed,
        "wall_duration": wall_elapsed,
        "mean_rtf": mean_rtf,
        "min_rtf": min_rtf,
        "min_dist": min_robot_human_dist,
        "contact_events": contact_events,
        "first_encounter": first_encounter_pos,
        "human_paths": runner.raw_human_paths,
        "samples": [
            {
                "t": s.t,
                "robot_pos": s.robot_pos,
                "robot_vel": s.robot_vel,
                "human_states": s.human_states
            }
            for s in runner.samples
        ]
    }
    with open(traj_path, 'w') as f:
        json.dump(traj_data, f, indent=2)
    print(f"Trajectory saved to: {traj_path}")

    # If baseline mode, save nominal reference path
    if args.mode == 'baseline':
        nom_path_file = os.path.join(args.output_dir, f"nominal_path_{args.scenario}_{args.run_id}.json")
        with open(nom_path_file, 'w') as f:
            json.dump(runner.raw_human_paths, f, indent=2)
        print(f"Nominal baseline path saved to: {nom_path_file}")

    # If robot_present, score against nominal path
    if args.mode == 'robot_present':
        # Load nominal reference (e.g. from baseline run 1)
        nom_file = os.path.join(args.output_dir, f"nominal_path_{args.scenario}_baseline_1.json")
        nom_paths = {}
        if os.path.exists(nom_file):
            with open(nom_file) as f:
                nom_paths = json.load(f)
            print(f"Loaded nominal reference from {nom_file}")
        else:
            print("WARNING: No baseline nominal file found! Evaluating without nominal reference.")

        summaries = score_episode_all_profiles(
            samples=runner.samples,
            nominal_human_paths=nom_paths,
            scenario_name=args.scenario,
            seed=args.seed,
            dt_nominal=0.1
        )

        # Final robot state and distance to goal
        rx_final = runner.current_robot_state.position.position.x if runner.current_robot_state else float('nan')
        ry_final = runner.current_robot_state.position.position.y if runner.current_robot_state else float('nan')
        dist_to_goal = math.hypot(rgx - rx_final, rgy - ry_final) if not math.isnan(rx_final) else float('nan')

        # Abort log lines extraction
        abort_logs = []
        if runner.nav_status in ["ABORTED", "CANCELED"]:
            log_file = "/tmp/sim_launch.log"
            if os.path.exists(log_file):
                try:
                    with open(log_file) as lf:
                        lines = lf.readlines()
                    for line in lines[-120:]:
                        if any(k in line for k in ['bt_navigator', 'controller_server', 'planner_server', 'ABORT', 'abort', 'cancel', 'shutting down', 'Segmentation']):
                            abort_logs.append(line.strip())
                except Exception as e:
                    abort_logs.append(f"Error reading log: {e}")

        # Human logging start time relative to simulation start
        h_start_rel = runner.samples[0].t - t_start_sim if runner.samples else 0.0

        eval_summary_file = os.path.join(args.output_dir, f"evaluation_{args.scenario}_{args.run_id}.json")
        summary_dict = {
            "run_id": args.run_id,
            "scenario": args.scenario,
            "mode": args.mode,
            "seed": args.seed,
            "jitter": args.jitter,
            "git_commit": get_git_commit_hash(),
            "sim_duration": sim_elapsed,
            "wall_duration": wall_elapsed,
            "mean_rtf": mean_rtf,
            "min_rtf": min_rtf,
            "nav2_final_status": runner.nav_status,
            "robot_final_pose": [rx_final, ry_final],
            "dist_to_goal": dist_to_goal,
            "min_robot_human_dist": min_robot_human_dist,
            "contact_events": contact_events,
            "first_encounter": first_encounter_pos,
            "human_logging_start_relative": h_start_rel,
            "abort_logs": abort_logs,
            "doorway_interaction": "standoff" if args.scenario == "doorway" and contact_events == 0 else "pass",
            "profiles": {
                p: {
                    "profile_name": s.profile_name,
                    "total_contact_events": s.total_contact_events,
                    "total_proxemics_violation_time": s.total_proxemics_violation_time,
                    "total_approach_violation_time": s.total_approach_violation_time,
                    "total_angular_violation_time": s.total_angular_violation_time,
                    "any_irregularity_violated": s.any_irregularity_violated,
                    "human_results": {
                        h: {
                            "min_distance": hr.min_distance,
                            "contact_events": hr.contact_events,
                            "proxemics_violation_time": hr.proxemics_violation_time,
                            "max_approach_speed": hr.max_approach_speed,
                            "approach_violation_time": hr.approach_violation_time,
                            "detour_ratio": hr.detour_ratio,
                            "angular_violation_time": hr.angular_violation_time,
                            "max_angular_deviation_deg": hr.max_angular_deviation_deg,
                            "max_lateral_deviation_m": hr.max_lateral_deviation_m,
                            "proxemics_violated": hr.proxemics_violated,
                            "approach_violated": hr.approach_violated,
                            "irregularity_violated": hr.irregularity_violated,
                            "contact_violated": hr.contact_violated,
                            "overall_violated": hr.overall_violated
                        }
                        for h, hr in s.human_results.items()
                    }
                }
                for p, s in summaries.items()
            }
        }
        with open(eval_summary_file, 'w') as f:
            json.dump(summary_dict, f, indent=2)
        print(f"\nFull Evaluation Summary saved to: {eval_summary_file}")
        print("\n=== RAW SUMMARY JSON ===")
        print(json.dumps(summary_dict, indent=2))

    rclpy.shutdown()

if __name__ == '__main__':
    main()
