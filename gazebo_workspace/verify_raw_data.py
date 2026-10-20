#!/usr/bin/env python3
import os
import json
import math
import statistics

RES_DIR = '/home/thevindu/SocialGym2/gazebo_workspace/evaluation_results'

def task1_checks():
    print("=" * 70)
    print("TASK 1: RECOMPUTE FROM RAW DATA")
    print("=" * 70)

    # 1a: doorway_j00 contact analysis
    f_d00 = os.path.join(RES_DIR, 'trajectory_doorway_robot_present_doorway_j00.json')
    with open(f_d00) as fp:
        d00 = json.load(fp)

    samples_00 = d00['samples']
    contact_samples = []
    dists_00 = []
    times_00 = []
    d_contact = 0.60

    for idx, s in enumerate(samples_00):
        t = s['t']
        rx, ry = s['robot_pos']
        hx, hy = s['human_states']['agent1']['pos']
        d = math.hypot(rx - hx, ry - hy)
        dists_00.append(d)
        times_00.append(t)
        if d < d_contact:
            contact_samples.append((idx, t, d, rx, ry, hx, hy))

    print("\n--- TASK 1a: doorway_j00 Physical Contact ---")
    print(f"Total samples in run: {len(samples_00)}")
    print(f"Contact sample count (d < 0.60m): {len(contact_samples)}")
    min_dist_00 = min(dists_00)
    min_idx = dists_00.index(min_dist_00)
    print(f"Minimum distance: {min_dist_00:.6f} m (at t = {times_00[min_idx]:.3f}s, sample idx {min_idx})")

    # Time steps in contact
    c_indices = [c[0] for c in contact_samples]
    c_times = [c[1] for c in contact_samples]
    dts = [samples_00[i]['t'] - samples_00[i-1]['t'] for i in c_indices if i > 0]
    avg_dt = statistics.mean(dts) if dts else 0.1
    print(f"Sample periods during contact: min={min(dts):.4f}s, max={max(dts):.4f}s, mean={avg_dt:.4f}s")
    
    t_start = c_times[0]
    t_end = c_times[-1]
    t_span = t_end - t_start
    sum_dt = sum(dts)
    print(f"First contact sample: t = {t_start:.4f}s (idx {c_indices[0]}, d={contact_samples[0][2]:.4f}m)")
    print(f"Last contact sample:  t = {t_end:.4f}s (idx {c_indices[-1]}, d={contact_samples[-1][2]:.4f}m)")
    print(f"Timestamp difference (t_last - t_first): {t_span:.4f}s (across N - 1 = {len(contact_samples)-1} intervals)")
    print(f"Sum of sample durations (17 samples * dt): {len(contact_samples) * avg_dt:.4f}s (sum of actual dt = {sum_dt:.4f}s)")
    print(f"Explanation: 1.72s in report is N * dt = 17 * 0.1012s (sample duration accumulation in evaluator.py).")
    print(f"             1.62s in Fig 2b legend is t_last - t_first = {t_span:.4f}s (time elapsed between first and last sample).")

    # 1b: doorway_j15 stationary interval
    f_d15 = os.path.join(RES_DIR, 'trajectory_doorway_robot_present_doorway_j15.json')
    with open(f_d15) as fp:
        d15 = json.load(fp)

    samples_15 = d15['samples']
    t0_15 = samples_15[0]['t']
    v_thresh = 0.05

    print("\n--- TASK 1b: doorway_j15 Stationary Intervals (v < 0.05 m/s) ---")
    stationary_intervals = []
    in_stat = False
    cur_start = None

    for idx, s in enumerate(samples_15):
        t_rel = s['t'] - t0_15
        rx, ry = s['robot_pos']
        rvx, rvy = s['robot_vel']
        v = math.hypot(rvx, rvy)
        hx, hy = s['human_states']['agent1']['pos']
        d = math.hypot(rx - hx, ry - hy)

        if v < v_thresh:
            if not in_stat:
                in_stat = True
                cur_start = (t_rel, idx, rx, ry, d)
        else:
            if in_stat:
                in_stat = False
                cur_end = (t_rel, idx, rx, ry, d)
                stationary_intervals.append((cur_start, cur_end))
    if in_stat:
        cur_end = (samples_15[-1]['t'] - t0_15, len(samples_15)-1, rx, ry, d)
        stationary_intervals.append((cur_start, cur_end))

    for i, (start, end) in enumerate(stationary_intervals):
        dur = end[0] - start[0]
        print(f"Interval {i+1}: t = {start[0]:.2f}s to {end[0]:.2f}s (duration {dur:.2f}s, {end[1]-start[1]} samples)")
        print(f"  At start: robot=({start[2]:.3f}, {start[3]:.3f}), human-robot dist={start[4]:.3f}m")
        print(f"  At end:   robot=({end[2]:.3f}, {end[3]:.3f}), human-robot dist={end[4]:.3f}m")
        if start[0] < 2.0:
            print("  -> Interpretation: Pre-dispatch waiting (robot waiting for jitter dispatch), NOT yielding.")
        elif dur > 4.0:
            print("  -> Interpretation: Actual Doorway Yielding / Standoff outside doorway.")

    # 1c: Robot departure times for all 10 episodes
    print("\n--- TASK 1c: Robot Departure Times (v > 0.05 m/s) Across All 10 Episodes ---")
    scenarios = ['corridor', 'doorway']
    jitters = ['j00', 'j05', 'j10', 'j15', 'j20']

    departure_times = {}
    for sc in scenarios:
        departure_times[sc] = {}
        for jt in jitters:
            fn = os.path.join(RES_DIR, f'trajectory_{sc}_robot_present_{sc}_{jt}.json')
            with open(fn) as fp:
                d = json.load(fp)
            samps = d['samples']
            t0 = samps[0]['t']
            dep_t = None
            for s in samps:
                rvx, rvy = s['robot_vel']
                v = math.hypot(rvx, rvy)
                if v > 0.05:
                    dep_t = s['t'] - t0
                    break
            departure_times[sc][jt] = dep_t

    for sc in scenarios:
        print(f"\nScenario: {sc}")
        t_ref = departure_times[sc]['j00']
        for jt in jitters:
            t_dep = departure_times[sc][jt]
            delta = t_dep - t_ref if t_dep is not None else 0.0
            print(f"  {jt}: departure at t = {t_dep:5.2f}s (delta relative to j00: {delta:+5.2f}s)")

    # 1d: Human start positions
    print("\n--- TASK 1d: Human Start Positions in Trajectory JSONs ---")
    for sc in scenarios:
        for jt in jitters:
            fn = os.path.join(RES_DIR, f'trajectory_{sc}_robot_present_{sc}_{jt}.json')
            with open(fn) as fp:
                d = json.load(fp)
            s0 = d['samples'][0]
            rx, ry = s0['robot_pos']
            h = s0['human_states']['agent1']
            hx, hy = h['pos']
            d_init = math.hypot(rx - hx, ry - hy)
            print(f"  {sc}_{jt}: t={s0['t']:.2f}s | Robot=({rx:.2f}, {ry:.2f}) | Human=({hx:.3f}, {hy:.3f}) | initial dist={d_init:.2f}m")

def task2_checks():
    print("\n" + "=" * 70)
    print("TASK 2: AMCL LOGGING & CORRIDOR LATERAL ERROR")
    print("=" * 70)
    f_c00 = os.path.join(RES_DIR, 'trajectory_corridor_robot_present_corridor_j00.json')
    with open(f_c00) as fp:
        d00 = json.load(fp)

    s0 = d00['samples'][0]
    print("Sample 0 fields:", list(s0.keys()))
    has_amcl = any('amcl' in k.lower() for k in s0.keys())
    print(f"Was amcl_pose logged in the trajectory JSON? {'YES' if has_amcl else 'NO, amcl_pose was NOT logged in any trajectory JSON.'}")

    print("\nCorridor robot (x, y) trajectory progression:")
    for jt in ['j00', 'j05', 'j10', 'j15', 'j20']:
        fn = os.path.join(RES_DIR, f'trajectory_corridor_robot_present_corridor_{jt}.json')
        with open(fn) as fp:
            d = json.load(fp)
        samps = d['samples']
        t0 = samps[0]['t']
        xs = [s['robot_pos'][0] for s in samps]
        ys = [s['robot_pos'][1] for s in samps]
        print(f"\nCorridor {jt} (total {len(samps)} samples):")
        print(f"  Initial robot pose: x={xs[0]:.3f}, y={ys[0]:.3f}")
        print(f"  Final robot pose:   x={xs[-1]:.3f}, y={ys[-1]:.3f} (goal target was x=9.0, y=0.0)")
        print(f"  Y range across entire run: min y={min(ys):.3f}, max y={max(ys):.3f}")
        # Print a few key waypoints
        for frac in [0.0, 0.25, 0.5, 0.75, 1.0]:
            idx = int(frac * (len(samps) - 1))
            s = samps[idx]
            hx, hy = s['human_states']['agent1']['pos']
            print(f"    t={s['t']-t0:5.1f}s | Robot=({s['robot_pos'][0]:6.2f}, {s['robot_pos'][1]:6.2f}) | Human=({hx:6.2f}, {hy:6.2f})")

def task4_checks():
    print("\n" + "=" * 70)
    print("TASK 4: PROXEMICS VIOLATION TIMES & MARGINS")
    print("=" * 70)
    for sc in ['corridor', 'doorway']:
        for jt in ['j00', 'j05', 'j10', 'j15', 'j20']:
            fn = os.path.join(RES_DIR, f'evaluation_{sc}_{sc}_{jt}.json')
            with open(fn) as fp:
                ev = json.load(fp)
            p_sens = ev['profiles']['sensitive']['total_proxemics_violation_time']
            p_bal = ev['profiles']['balanced']['total_proxemics_violation_time']
            p_tol = ev['profiles']['tolerant']['total_proxemics_violation_time']
            min_d = ev['min_robot_human_dist']
            contacts = ev['contact_events']
            print(f"  {sc}_{jt}: min_d={min_d:.4f}m, contacts={contacts} | Proxemics: Sens={p_sens:.2f}s, Bal={p_bal:.2f}s, Tol={p_tol:.2f}s")

if __name__ == '__main__':
    task1_checks()
    task2_checks()
    task4_checks()
