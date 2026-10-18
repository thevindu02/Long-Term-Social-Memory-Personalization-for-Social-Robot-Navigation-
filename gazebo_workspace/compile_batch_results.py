#!/usr/bin/env python3
import json
import os
import glob
import csv
from collections import defaultdict
import numpy as np

OUTPUT_DIR = "/home/thevindu/SocialGym2/gazebo_workspace/evaluation_results"
CSV_PATH = os.path.join(OUTPUT_DIR, "batch_results_all.csv")
PLOT_PATH = os.path.join(OUTPUT_DIR, "batch_violation_times.png")

RUN_IDS = [
    ("corridor", "corridor_j00"),
    ("corridor", "corridor_j05"),
    ("corridor", "corridor_j10"),
    ("corridor", "corridor_j15"),
    ("corridor", "corridor_j20"),
    ("doorway", "doorway_j00"),
    ("doorway", "doorway_j05"),
    ("doorway", "doorway_j10"),
    ("doorway", "doorway_j15"),
    ("doorway", "doorway_j20"),
]

PROFILES = ["sensitive", "balanced", "tolerant"]

def main():
    rows = []
    episode_data = {}

    for scenario, run_id in RUN_IDS:
        eval_file = os.path.join(OUTPUT_DIR, f"evaluation_{scenario}_{run_id}.json")
        if not os.path.exists(eval_file):
            print(f"Warning: {eval_file} not found!")
            continue

        with open(eval_file) as f:
            d = json.load(f)

        episode_data[run_id] = d
        nav_status = d.get("nav2_final_status", "UNKNOWN")
        sim_dur = d.get("sim_duration", 0.0)
        wall_dur = d.get("wall_duration", 0.0)
        mean_rtf = d.get("mean_rtf", 0.0)
        min_rtf = d.get("min_rtf", 0.0)
        min_dist = d.get("min_robot_human_dist", 0.0)
        contacts = d.get("contact_events", 0)
        jitter = d.get("jitter", 0.0)
        dist_to_goal = d.get("dist_to_goal", float('nan'))
        h_logging_start = d.get("human_logging_start_relative", 0.0)

        fe = d.get("first_encounter")
        fe_t = fe.get("t", 0.0) if fe else 0.0
        fe_d = fe.get("distance", 0.0) if fe else 0.0

        for p_key in PROFILES:
            prof = d["profiles"].get(p_key, {})
            p_name = prof.get("profile_name", p_key.capitalize())
            total_prox_time = prof.get("total_proxemics_violation_time", 0.0)
            total_app_time = prof.get("total_approach_violation_time", 0.0)
            total_ang_time = prof.get("total_angular_violation_time", 0.0)
            any_irreg = prof.get("any_irregularity_violated", False)

            # agent1 results
            a1 = prof.get("human_results", {}).get("agent1", {})
            max_app_spd = a1.get("max_approach_speed", 0.0)
            detour = a1.get("detour_ratio", 0.0)
            max_ang_dev = a1.get("max_angular_deviation_deg", 0.0)
            max_lat_dev = a1.get("max_lateral_deviation_m", 0.0)
            prox_viol = a1.get("proxemics_violated", total_prox_time > 0)
            app_viol = a1.get("approach_violated", total_app_time > 0)
            contact_viol = a1.get("contact_violated", contacts > 0)
            irreg_viol = a1.get("irregularity_violated", any_irreg)
            overall_viol = a1.get("overall_violated", prox_viol or app_viol or irreg_viol or contact_viol)

            rows.append({
                "scenario": scenario,
                "run_id": run_id,
                "jitter_sec": jitter,
                "profile": p_name,
                "nav2_final_status": nav_status,
                "dist_to_goal_m": round(dist_to_goal, 3),
                "sim_duration_sec": round(sim_dur, 2),
                "wall_duration_sec": round(wall_dur, 2),
                "mean_rtf": round(mean_rtf, 3),
                "min_rtf": round(min_rtf, 3),
                "min_distance_m": round(min_dist, 3),
                "contact_events": contacts,
                "first_encounter_dist_m": round(fe_d, 3),
                "human_logging_start_sec": round(h_logging_start, 2),
                "proxemics_violation_time_sec": round(total_prox_time, 3),
                "proxemics_violated": prox_viol,
                "approach_violation_time_sec": round(total_app_time, 3),
                "max_approach_speed_mps": round(max_app_spd, 3),
                "approach_violated": app_viol,
                "detour_ratio": round(detour, 5),
                "angular_violation_time_sec": round(total_ang_time, 3),
                "max_angular_deviation_deg": round(max_ang_dev, 2),
                "max_lateral_deviation_m": round(max_lat_dev, 3),
                "irregularity_violated": irreg_viol,
                "contact_violated": contact_viol,
                "overall_violated": overall_viol
            })

    # 1. Write CSV
    if rows:
        fieldnames = list(rows[0].keys())
        with open(CSV_PATH, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        print(f"CSV generated: {CSV_PATH} ({len(rows)} rows)")

    # 2. Print Summary Table
    print("\n=========================================================================================")
    print("=== BATCH EVALUATION SUMMARY TABLE: 10 EPISODES x 3 PROFILES ===")
    print("=========================================================================================")
    # Group by scenario and profile
    grouped = defaultdict(list)
    for r in rows:
        key = (r['scenario'], r['profile'])
        grouped[key].append(r)

    print(f"{'Scenario':10s} | {'Profile':9s} | {'Prox Time (s) [min/mean/max]':28s} | {'Appr Time (s) [min/mean/max]':28s} | {'Irreg T_ang(s) [min/mean/max]':29s} | {'Detour [%]':12s} | {'Min Dist (m)':14s} | {'Contacts':8s}")
    print("-" * 155)

    summary_stats = {}
    for (scen, prof), r_list in sorted(grouped.items()):
        prox_t = [x['proxemics_violation_time_sec'] for x in r_list]
        app_t = [x['approach_violation_time_sec'] for x in r_list]
        ang_t = [x['angular_violation_time_sec'] for x in r_list]
        det = [x['detour_ratio'] * 100 for x in r_list]
        d_min = [x['min_distance_m'] for x in r_list]
        contacts = sum(x['contact_events'] for x in r_list)

        prox_str = f"{min(prox_t):.2f} / {np.mean(prox_t):.2f} / {max(prox_t):.2f}"
        app_str = f"{min(app_t):.2f} / {np.mean(app_t):.2f} / {max(app_t):.2f}"
        ang_str = f"{min(ang_t):.2f} / {np.mean(ang_t):.2f} / {max(ang_t):.2f}"
        det_str = f"{min(det):.2f} / {np.mean(det):.2f}%"
        d_str = f"{min(d_min):.2f} / {np.mean(d_min):.2f}"

        print(f"{scen:10s} | {prof:9s} | {prox_str:28s} | {app_str:28s} | {ang_str:29s} | {det_str:12s} | {d_str:14s} | {contacts:8d}")
        summary_stats[(scen, prof)] = {
            "prox": (min(prox_t), np.mean(prox_t), max(prox_t)),
            "app": (min(app_t), np.mean(app_t), max(app_t)),
            "ang": (min(ang_t), np.mean(ang_t), max(ang_t)),
            "det": (min(det), np.mean(det), max(det)),
            "dist": (min(d_min), np.mean(d_min), max(d_min)),
            "contacts": contacts
        }

    # 3. Print Episode Breakdown
    print("\n=========================================================================================")
    print("=== PER-EPISODE NAV2 & METRIC BREAKDOWN ===")
    print("=========================================================================================")
    print(f"{'Run ID':15s} | {'Scenario':9s} | {'Jitter':6s} | {'Nav2 Status':12s} | {'Sim (s)':7s} | {'Wall (s)':8s} | {'RTF':5s} | {'Min Dist':8s} | {'Contacts':8s} | {'Dist Goal':9s}")
    print("-" * 105)
    for scenario, run_id in RUN_IDS:
        d = episode_data.get(run_id, {})
        if not d: continue
        print(f"{run_id:15s} | {scenario:9s} | {d.get('jitter',0.0):5.1f}s | {d.get('nav2_final_status','N/A'):12s} | {d.get('sim_duration',0.0):7.1f} | {d.get('wall_duration',0.0):8.1f} | {d.get('mean_rtf',0.0):5.3f} | {d.get('min_robot_human_dist',0.0):7.3f}m | {d.get('contact_events',0):8d} | {d.get('dist_to_goal',0.0):8.2f}m")

    # 4. List of unexpected results / abort reasons
    print("\n=========================================================================================")
    print("=== UNEXPECTED RESULTS & ABORT REASONS WITH LOGS ===")
    print("=========================================================================================")
    unexpected_found = False
    for scenario, run_id in RUN_IDS:
        d = episode_data.get(run_id, {})
        status = d.get("nav2_final_status", "")
        abort_logs = d.get("abort_logs", [])
        if status in ["ABORTED", "TIMEOUT", "INVALID", "REJECTED"]:
            unexpected_found = True
            print(f"\nEpisode: {run_id} ({scenario}, jitter={d.get('jitter')}s) -> STATUS: {status}")
            print(f"  Final Robot Pose: {d.get('robot_final_pose')}, Dist to Goal: {d.get('dist_to_goal'):.2f}m")
            print(f"  Sim Duration: {d.get('sim_duration'):.2f}s, Wall Duration: {d.get('wall_duration'):.2f}s")
            print(f"  Min Dist: {d.get('min_robot_human_dist'):.3f}m, Contacts: {d.get('contact_events')}")
            print(f"  Captured Abort / Lifecycle Logs ({len(abort_logs)} lines):")
            for line in abort_logs[-10:]:
                print(f"    {line}")
    if not unexpected_found:
        print("None! All episodes completed with expected outcomes.")

    # 5. Generate Bar Chart using matplotlib
    try:
        import matplotlib.pyplot as plt
        scenarios = ["corridor", "doorway"]
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))

        for idx, scen in enumerate(scenarios):
            ax = axes[idx]
            x = np.arange(len(PROFILES))
            width = 0.25

            prox_means = [summary_stats[(scen, p.capitalize())]["prox"][1] for p in PROFILES]
            app_means = [summary_stats[(scen, p.capitalize())]["app"][1] for p in PROFILES]
            ang_means = [summary_stats[(scen, p.capitalize())]["ang"][1] for p in PROFILES]

            rects1 = ax.bar(x - width, prox_means, width, label='Proxemics Viol (s)', color='#2b5c8f')
            rects2 = ax.bar(x, app_means, width, label='Approach Speed Viol (s)', color='#d95f02')
            rects3 = ax.bar(x + width, ang_means, width, label='Irreg Time Viol T_θ (s)', color='#7570b3')

            ax.set_ylabel('Mean Violation Duration (sim seconds)')
            ax.set_title(f'Comfort Violation Durations: {scen.capitalize()} Scenario')
            ax.set_xticks(x)
            ax.set_xticklabels([p.capitalize() for p in PROFILES])
            ax.legend()
            ax.grid(axis='y', linestyle='--', alpha=0.7)

        plt.tight_layout()
        plt.savefig(PLOT_PATH, dpi=150)
        print(f"\nBar chart successfully saved to: {PLOT_PATH}")
    except Exception as e:
        print(f"Plotting note: {e}")

if __name__ == '__main__':
    main()
