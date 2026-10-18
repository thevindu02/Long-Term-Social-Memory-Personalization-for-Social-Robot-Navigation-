#!/usr/bin/env python3
import json
import math
import os
from typing import List, Dict, Any, Tuple

from comfort_signals.signals import (
    project_point_to_polyline,
    wrap_to_pi,
    compute_polyline_length,
    compute_detour_ratio
)

def compute_smoothed_headings(samples: List[Dict[str, Any]], window_size: int = 5, min_speed: float = 0.1) -> List[Tuple[float, float, float, bool]]:
    """
    Computes motion heading from velocity / displacement.
    Returns list of (speed, raw_motion_heading, smoothed_motion_heading, is_valid)
    where is_valid is True only when speed > min_speed.
    """
    n = len(samples)
    headings = []
    speeds = []
    
    for i in range(n):
        s = samples[i]
        h = s['human_states']['agent1']
        vx, vy = h['vel']
        spd = math.hypot(vx, vy)
        speeds.append(spd)
        
        # Prefer displacement if consecutive samples exist, else velocity vector
        if i > 0:
            h_prev = samples[i-1]['human_states']['agent1']
            dx = h['pos'][0] - h_prev['pos'][0]
            dy = h['pos'][1] - h_prev['pos'][1]
            dt = s['t'] - samples[i-1]['t']
            if dt > 1e-4 and math.hypot(dx, dy) / dt > 0.05:
                heading = math.atan2(dy, dx)
            elif spd > 1e-4:
                heading = math.atan2(vy, vx)
            else:
                heading = 0.0
        elif spd > 1e-4:
            heading = math.atan2(vy, vx)
        else:
            heading = 0.0
        headings.append(heading)
        
    results = []
    half_w = window_size // 2
    
    for i in range(n):
        spd = speeds[i]
        is_valid = (spd > min_speed)
        
        # Smooth heading over window using circular mean (sin/cos components)
        w_start = max(0, i - half_w)
        w_end = min(n, i + half_w + 1)
        
        sin_sum = 0.0
        cos_sum = 0.0
        valid_count = 0
        
        for j in range(w_start, w_end):
            if speeds[j] > min_speed:
                sin_sum += math.sin(headings[j])
                cos_sum += math.cos(headings[j])
                valid_count += 1
                
        if valid_count > 0:
            smoothed_heading = math.atan2(sin_sum, cos_sum)
        else:
            smoothed_heading = headings[i]
            
        results.append((spd, headings[i], smoothed_heading, is_valid))
        
    return results

def analyze_scenario(traj_file: str, nom_file: str, scenario_name: str, goal_limit: float, goal_axis: str):
    with open(traj_file) as f:
        traj = json.load(f)
    with open(nom_file) as f:
        nom = json.load(f)['agent1']
        
    samples = traj['samples']
    smoothed_info = compute_smoothed_headings(samples, window_size=5, min_speed=0.1)
    
    t0 = samples[0]['t']
    records = []
    
    for i, s in enumerate(samples):
        t = s['t']
        rel_t = t - t0
        r_pos = s['robot_pos']
        h = s['human_states']['agent1']
        hx, hy = h['pos']
        msg_yaw = h['heading']
        
        spd, raw_motion_heading, smoothed_heading, is_valid = smoothed_info[i]
        
        # Check if human reached goal
        if goal_axis == 'x':
            at_goal = (hx <= goal_limit)
        else:
            at_goal = (hy >= goal_limit)
            
        _, _, nom_tang = project_point_to_polyline((hx, hy), nom)
        
        raw_dev = abs(math.degrees(wrap_to_pi(msg_yaw - nom_tang)))
        
        if is_valid and not at_goal:
            corrected_dev = abs(math.degrees(wrap_to_pi(smoothed_heading - nom_tang)))
        else:
            corrected_dev = 0.0
            
        d_robot = math.hypot(r_pos[0] - hx, r_pos[1] - hy)
        
        records.append({
            'rel_t': rel_t,
            'hx': hx,
            'hy': hy,
            'spd': spd,
            'msg_yaw': math.degrees(msg_yaw),
            'raw_motion_head': math.degrees(raw_motion_heading),
            'smoothed_head': math.degrees(smoothed_heading),
            'nom_tang': math.degrees(nom_tang),
            'raw_dev': raw_dev,
            'corrected_dev': corrected_dev,
            'd_robot': d_robot,
            'is_valid': is_valid,
            'at_goal': at_goal
        })
        
    print(f"\n=======================================================")
    print(f"=== {scenario_name} EPISODE HEADING ANALYSIS (every 0.5s) ===")
    print(f"=======================================================")
    print(f"rel_t(s) | h_pos (x, y)  | spd(m/s)| msg_yaw | sm_head | nom_tang| raw_dev | corr_dev| d_rob(m)| flags")
    print("-----------------------------------------------------------------------------------------------------")
    
    last_print = -1.0
    for r in records:
        if r['rel_t'] - last_print >= 0.49:
            last_print = r['rel_t']
            flags = ""
            if r['at_goal']:
                flags += "[AT_GOAL] "
            if not r['is_valid']:
                flags += "[LOW_SPEED]"
            print(f"{r['rel_t']:7.2f} | ({r['hx']:5.2f}, {r['hy']:5.2f}) | {r['spd']:7.3f} | {r['msg_yaw']:7.1f} | {r['smoothed_head']:7.1f} | {r['nom_tang']:7.1f} | {r['raw_dev']:7.1f} | {r['corrected_dev']:7.1f} | {r['d_robot']:7.3f} | {flags}")
            
    # Max analysis
    max_raw = max(records, key=lambda x: x['raw_dev'])
    active_records = [r for r in records if r['is_valid'] and not r['at_goal']]
    max_corr = max(active_records, key=lambda x: x['corrected_dev']) if active_records else records[0]
    
    print(f"\n--- SUMMARY FOR {scenario_name} ---")
    print(f"Raw Max Angular Dev: {max_raw['raw_dev']:.2f} deg at rel_t={max_raw['rel_t']:.2f}s (spd={max_raw['spd']:.4f} m/s, d_rob={max_raw['d_robot']:.3f}m, at_goal={max_raw['at_goal']})")
    print(f"Corrected Max Angular Dev: {max_corr['corrected_dev']:.2f} deg at rel_t={max_corr['rel_t']:.2f}s (spd={max_corr['spd']:.4f} m/s, d_rob={max_corr['d_robot']:.3f}m, at_goal={max_corr['at_goal']})")
    
    return records

if __name__ == '__main__':
    c_records = analyze_scenario(
        'evaluation_results/trajectory_corridor_robot_present_eval_corridor.json',
        'evaluation_results/nominal_path_corridor_baseline_1.json',
        'CORRIDOR',
        goal_limit=-8.0,
        goal_axis='x'
    )
    
    d_records = analyze_scenario(
        'evaluation_results/trajectory_doorway_robot_present_eval_doorway.json',
        'evaluation_results/nominal_path_doorway_baseline_1.json',
        'DOORWAY',
        goal_limit=0.5,
        goal_axis='y'
    )

    def get_time_above(recs, min_angle):
        dt = 0.1
        t = 0.0
        for r in recs:
            if r['d_robot'] < 3.0 and r['is_valid'] and not r['at_goal']:
                if r['corrected_dev'] > min_angle:
                    t += dt
        return t

    print("\n=======================================================")
    print("=== PROFILE SEPARATION WITH TIME-IN-VIOLATION METRIC ===")
    print("=======================================================")
    
    profiles = {
        'Sensitive': {'d_pref': 1.20, 'v_max': 0.10, 'eta_th': 0.05, 'ang_th': 15.0, 'time_th': 1.0},
        'Balanced':  {'d_pref': 0.90, 'v_max': 0.16, 'eta_th': 0.10, 'ang_th': 20.0, 'time_th': 1.8},
        'Tolerant':  {'d_pref': 0.70, 'v_max': 0.22, 'eta_th': 0.20, 'ang_th': 25.0, 'time_th': 1.0},
    }
    
    for name, recs, ep_detour, min_d, contacts in [
        ('Corridor', c_records, 0.006, 0.963, 0),
        ('Doorway', d_records, 0.000, 0.387, 17)
    ]:
        print(f"\n--- Scenario: {name} (min_d={min_d}m, contacts={contacts}, detour={ep_detour}) ---")
        for p_name, p in profiles.items():
            t_ang = get_time_above(recs, p['ang_th'])
            irreg_viol = (ep_detour > p['eta_th']) or (t_ang > p['time_th'])
            prox_viol = (min_d < p['d_pref'])
            contact_viol = (contacts > 0)
            print(f"{p_name:10s}: Prox_Viol={str(prox_viol):5s} | Irreg_Viol={str(irreg_viol):5s} (T_ang={t_ang:4.2f}s > {p['time_th']}s @ th={p['ang_th']} deg)")

