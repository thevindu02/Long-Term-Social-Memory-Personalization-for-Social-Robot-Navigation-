#!/usr/bin/env python3
import os
import json
import math

RES_DIR = '/home/thevindu/SocialGym2/gazebo_workspace/evaluation_results'
scenarios = ['corridor', 'doorway']
jitters = ['j00', 'j05', 'j10', 'j15', 'j20']

print("=" * 95)
print(f"{'Episode':<15} | {'Log Start (sim s)':<18} | {'Departure (sim s)':<18} | {'Rel Dep (s)':<12} | {'Human x (m)':<12} | {'Human y (m)':<12}")
print("=" * 95)

for sc in scenarios:
    for jt in jitters:
        ep_name = f"{sc}_{jt}"
        fn = os.path.join(RES_DIR, f"trajectory_{sc}_robot_present_{ep_name}.json")
        with open(fn) as fp:
            d = json.load(fp)
        samps = d['samples']
        t_log_start = samps[0]['t']

        dep_sample = None
        for s in samps:
            rvx, rvy = s['robot_vel']
            if math.hypot(rvx, rvy) > 0.05:
                dep_sample = s
                break

        if dep_sample is not None:
            t_dep_sim = dep_sample['t']
            t_dep_rel = t_dep_sim - t_log_start
            hx = dep_sample['human_states']['agent1']['pos'][0]
            hy = dep_sample['human_states']['agent1']['pos'][1]
            print(f"{ep_name:<15} | {t_log_start:18.3f} | {t_dep_sim:18.3f} | {t_dep_rel:12.3f} | {hx:12.3f} | {hy:12.3f}")
        else:
            print(f"{ep_name:<15} | {t_log_start:18.3f} | {'N/A':<18} | {'N/A':<12} | {'N/A':<12} | {'N/A':<12}")
print("=" * 95)
