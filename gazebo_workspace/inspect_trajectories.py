#!/usr/bin/env python3
import json
import glob
import math
import os

OUTPUT_DIR = "/home/thevindu/SocialGym2/gazebo_workspace/evaluation_results"

def analyze():
    for scen in ['corridor', 'doorway']:
        print(f"\n=======================================================", flush=True)
        print(f"=== SCENARIO: {scen.upper()} ===", flush=True)
        print(f"=======================================================", flush=True)
        pattern = os.path.join(OUTPUT_DIR, f"trajectory_{scen}_robot_present_{scen}_j*.json")
        files = sorted(glob.glob(pattern))
        for f in files:
            with open(f) as fp:
                d = json.load(fp)
            run_id = d['run_id']
            jitter = d.get('jitter', 0.0)
            samples = d.get('samples', [])
            if not samples:
                continue

            t0 = samples[0]['t']
            
            # Contact analysis
            in_contact = False
            contact_samples = 0
            contact_episodes = 0
            contact_intervals = []
            cur_start = None
            
            # Human motion
            first_move_t = None
            init_hpos = None
            pos_5, pos_10, pos_15 = None, None, None

            # Robot motion
            first_robot_move_t = None
            init_rpos = samples[0]['robot_pos']
            
            for s in samples:
                t_rel = s['t'] - t0
                rx, ry = s['robot_pos']
                h_dict = s['human_states'].get('agent1', {})
                hx, hy = h_dict.get('pos', (None, None))

                # Robot first movement (>0.05m from init)
                rdist = math.hypot(rx - init_rpos[0], ry - init_rpos[1])
                if first_robot_move_t is None and rdist > 0.05:
                    first_robot_move_t = t_rel

                # Human motion
                if hx is not None:
                    if init_hpos is None:
                        init_hpos = (hx, hy)
                    hdist = math.hypot(hx - init_hpos[0], hy - init_hpos[1])
                    if first_move_t is None and hdist > 0.05:
                        first_move_t = t_rel
                    if pos_5 is None and t_rel >= 5.0:
                        pos_5 = (round(hx, 3), round(hy, 3))
                    if pos_10 is None and t_rel >= 10.0:
                        pos_10 = (round(hx, 3), round(hy, 3))
                    if pos_15 is None and t_rel >= 15.0:
                        pos_15 = (round(hx, 3), round(hy, 3))

                # Distance & contact
                if hx is not None:
                    dist = math.hypot(rx - hx, ry - hy)
                    if dist < 0.60:
                        contact_samples += 1
                        if not in_contact:
                            in_contact = True
                            contact_episodes += 1
                            cur_start = t_rel
                    else:
                        if in_contact:
                            in_contact = False
                            contact_intervals.append((round(cur_start, 2), round(t_rel, 2), round(t_rel - cur_start, 2)))

            if in_contact:
                contact_intervals.append((round(cur_start, 2), round(t_rel, 2), round(t_rel - cur_start, 2)))

            tot_contact_dur = sum(i[2] for i in contact_intervals)

            # Get final pose and dist from evaluation json
            eval_f = os.path.join(OUTPUT_DIR, f"evaluation_{scen}_{run_id}.json")
            final_pose, dist_goal, nav_status = None, None, None
            if os.path.exists(eval_f):
                with open(eval_f) as ef:
                    ed = json.load(ef)
                final_pose = ed.get('robot_final_pose')
                dist_goal = ed.get('dist_to_goal')
                nav_status = ed.get('nav2_final_status')

            print(f"Episode: {run_id} (jitter={jitter}s):", flush=True)
            print(f"  Nav2 Status: {nav_status} | Sim Duration: {d['sim_duration']:.2f}s", flush=True)
            dist_str = f"{dist_goal:.3f}m" if dist_goal is not None else "N/A"
            print(f"  Final Robot Pose: {final_pose} | Dist to Goal: {dist_str}", flush=True)
            print(f"  Robot First Moved: {first_robot_move_t:.2f}s" if first_robot_move_t else "  Robot Never Moved", flush=True)
            print(f"  Human First Moved: {first_move_t:.2f}s" if first_move_t else "  Human Never Moved", flush=True)
            print(f"  Human Pos at t=5s:  {pos_5}", flush=True)
            print(f"  Human Pos at t=10s: {pos_10}", flush=True)
            print(f"  Human Pos at t=15s: {pos_15}", flush=True)
            print(f"  Contacts: samples={contact_samples}, episodes={contact_episodes}, duration={tot_contact_dur:.2f}s, intervals={contact_intervals}", flush=True)

if __name__ == '__main__':
    analyze()
