#!/usr/bin/env python3
import json
import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams.update({
    'font.size': 14,
    'axes.labelsize': 15,
    'axes.titlesize': 16,
    'xtick.labelsize': 14,
    'ytick.labelsize': 14,
    'legend.fontsize': 13,
    'figure.titlesize': 18
})

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
RES_DIR = os.path.join(BASE_DIR, 'evaluation_results')

def generate_figure_2a():
    """Per-episode grouped bar chart of proxemics violation time per profile."""
    # Data from batch_results_all.csv / evaluations
    episodes = ['j00 (0.0s)', 'j05 (0.5s)', 'j10 (1.0s)', 'j15 (1.5s)', 'j20 (2.0s)']
    
    # Corridor proxemics violation times (s)
    corr_sens = [2.213, 2.314, 2.213, 3.019, 3.119]
    corr_bal  = [0.000, 0.000, 0.000, 0.402, 0.302]
    corr_tol  = [0.000, 0.000, 0.000, 0.000, 0.000]

    # Doorway proxemics violation times (s)
    door_sens = [4.239, 4.239, 4.239, 6.462, 4.341]
    door_bal  = [3.028, 3.028, 3.028, 4.139, 3.029]
    door_tol  = [2.222, 2.121, 2.222, 1.717, 2.221]

    fig, axes = plt.subplots(1, 2, figsize=(18, 7), dpi=150)

    x = np.arange(len(episodes))
    width = 0.26

    # 1. Corridor subplot
    ax1 = axes[0]
    r1 = ax1.bar(x - width, corr_sens, width, label='Sensitive (d_pref=1.20m)', color='#2b5c8f', edgecolor='black', linewidth=1)
    r2 = ax1.bar(x, corr_bal, width, label='Balanced (d_pref=0.90m)', color='#2ca02c', edgecolor='black', linewidth=1)
    r3 = ax1.bar(x + width, corr_tol, width, label='Tolerant (d_pref=0.70m)', color='#d95f02', edgecolor='black', linewidth=1)

    ax1.set_ylabel('Proxemics Violation Time (s)', fontsize=15, fontweight='bold')
    ax1.set_xlabel('Scenario Variation (Human Start Delay)', fontsize=15, fontweight='bold')
    ax1.set_title('Corridor Scenario: Proxemics Violation Time', fontsize=16, fontweight='bold', pad=12)
    ax1.set_xticks(x)
    ax1.set_xticklabels(episodes, fontweight='bold')
    ax1.set_ylim([0, 8.5])
    ax1.grid(axis='y', linestyle='--', alpha=0.7)
    ax1.legend(loc='upper left', frameon=True, facecolor='#f8f9fa', edgecolor='#cccccc')

    # Add value labels above bars in ax1
    for rects in [r1, r2]:
        for r in rects:
            h = r.get_height()
            if h > 0.05:
                ax1.annotate(f'{h:.2f}s',
                             xy=(r.get_x() + r.get_width() / 2, h),
                             xytext=(0, 4), textcoords="offset points",
                             ha='center', va='bottom', fontsize=11, fontweight='bold')

    # 2. Doorway subplot
    ax2 = axes[1]
    r4 = ax2.bar(x - width, door_sens, width, label='Sensitive (d_pref=1.20m)', color='#2b5c8f', edgecolor='black', linewidth=1)
    r5 = ax2.bar(x, door_bal, width, label='Balanced (d_pref=0.90m)', color='#2ca02c', edgecolor='black', linewidth=1)
    r6 = ax2.bar(x + width, door_tol, width, label='Tolerant (d_pref=0.70m)', color='#d95f02', edgecolor='black', linewidth=1)

    ax2.set_ylabel('Proxemics Violation Time (s)', fontsize=15, fontweight='bold')
    ax2.set_xlabel('Scenario Variation (Human Start Delay)', fontsize=15, fontweight='bold')
    ax2.set_title('Doorway Scenario: Proxemics Violation Time', fontsize=16, fontweight='bold', pad=12)
    ax2.set_xticks(x)
    ax2.set_xticklabels(episodes, fontweight='bold')
    ax2.set_ylim([0, 8.5])
    ax2.grid(axis='y', linestyle='--', alpha=0.7)
    ax2.legend(loc='upper right', frameon=True, facecolor='#f8f9fa', edgecolor='#cccccc')

    # Add value labels above bars in ax2
    for rects in [r4, r5, r6]:
        for r in rects:
            h = r.get_height()
            if h > 0.05:
                ax2.annotate(f'{h:.2f}s',
                             xy=(r.get_x() + r.get_width() / 2, h),
                             xytext=(0, 4), textcoords="offset points",
                             ha='center', va='bottom', fontsize=11, fontweight='bold')

    plt.tight_layout()
    out_path = os.path.join(RES_DIR, 'figure_2a_per_episode_proxemics.png')
    plt.savefig(out_path, dpi=180, bbox_inches='tight')
    plt.close()
    print(f"Saved Figure 2(a) to {out_path}")

def generate_figure_2b():
    """Timeline plot of robot-human distance vs sim time for doorway_j00 and doorway_j15."""
    f00 = os.path.join(RES_DIR, 'trajectory_doorway_robot_present_doorway_j00.json')
    f15 = os.path.join(RES_DIR, 'trajectory_doorway_robot_present_doorway_j15.json')

    with open(f00) as fp:
        d00 = json.load(fp)
    with open(f15) as fp:
        d15 = json.load(fp)

    def extract_timeline(d):
        samples = d['samples']
        t0 = samples[0]['t']
        t_arr, dist_arr, r_speed_arr = [], [], []
        for s in samples:
            t = s['t'] - t0
            rx, ry = s['robot_pos']
            rvx, rvy = s['robot_vel']
            hx, hy = s['human_states']['agent1']['pos']
            dist = math.hypot(rx - hx, ry - hy)
            speed = math.hypot(rvx, rvy)
            t_arr.append(t)
            dist_arr.append(dist)
            r_speed_arr.append(speed)
        return np.array(t_arr), np.array(dist_arr), np.array(r_speed_arr)

    t00, dist00, sp00 = extract_timeline(d00)
    t15, dist15, sp15 = extract_timeline(d15)

    fig, axes = plt.subplots(2, 1, figsize=(15, 11), dpi=150, sharey=True)

    # Threshold values
    d_contact = 0.60
    d_tol = 0.70
    d_bal = 0.90
    d_sens = 1.20

    # 1. Doorway j00 Subplot
    ax1 = axes[0]
    ax1.plot(t00, dist00, color='#1f77b4', linewidth=3.5, label='Robot-Human Center Distance', zorder=5)

    # Reference lines
    ax1.axhline(d_sens, color='#9467bd', linestyle='--', linewidth=2.0, label='d_pref Sensitive (1.20 m)', zorder=3)
    ax1.axhline(d_bal, color='#2ca02c', linestyle='-.', linewidth=2.0, label='d_pref Balanced (0.90 m)', zorder=3)
    ax1.axhline(d_tol, color='#ff7f0e', linestyle=':', linewidth=2.2, label='d_pref Tolerant (0.70 m)', zorder=3)
    ax1.axhline(d_contact, color='#d62728', linestyle='-', linewidth=2.5, label='Contact Boundary (0.60 m)', zorder=4)

    # Highlight contact region
    contact_mask00 = dist00 < d_contact
    if np.any(contact_mask00):
        t_c_start = t00[contact_mask00][0]
        t_c_end = t00[contact_mask00][-1]
        ax1.axvspan(t_c_start, t_c_end, color='#d62728', alpha=0.20, label=f'Physical Contact Episode ({t_c_end - t_c_start:.2f} s)')

    # Stationary mask
    stat_mask00 = sp00 < 0.03
    # Check stationary spans
    in_stat = False
    s_start = None
    for i, is_s in enumerate(stat_mask00):
        if is_s and not in_stat:
            in_stat = True
            s_start = t00[i]
        elif not is_s and in_stat:
            in_stat = False
            if t00[i] - s_start > 0.5:
                ax1.axvspan(s_start, t00[i], color='#7f7f7f', alpha=0.25, hatch='//')
    if in_stat and (t00[-1] - s_start > 0.5):
        ax1.axvspan(s_start, t00[-1], color='#7f7f7f', alpha=0.25, hatch='//')

    ax1.set_title('Episode doorway_j00 (Jitter = 0.0 s): Simultaneous Doorway Entry with Physical Contact',
                  fontsize=16, fontweight='bold', pad=10)
    ax1.set_ylabel('Distance (meters)', fontsize=15, fontweight='bold')
    ax1.set_xlim([0, 18.0])
    ax1.set_ylim([0.0, 5.0])
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right', frameon=True, facecolor='#ffffff', edgecolor='#cccccc', fontsize=12)

    # Annotation for min distance
    min_d00 = np.min(dist00)
    t_min00 = t00[np.argmin(dist00)]
    ax1.annotate(f'Min Distance: {min_d00:.2f} m\n(Contact < 0.60 m)',
                 xy=(t_min00, min_d00), xytext=(t_min00 + 1.2, min_d00 + 0.9),
                 arrowprops=dict(arrowstyle="->", color='#d62728', lw=2),
                 color='#d62728', fontsize=13, fontweight='bold',
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='#ffebee', edgecolor='#d62728'))

    # 2. Doorway j15 Subplot
    ax2 = axes[1]
    ax2.plot(t15, dist15, color='#1f77b4', linewidth=3.5, label='Robot-Human Center Distance', zorder=5)

    ax2.axhline(d_sens, color='#9467bd', linestyle='--', linewidth=2.0, label='d_pref Sensitive (1.20 m)', zorder=3)
    ax2.axhline(d_bal, color='#2ca02c', linestyle='-.', linewidth=2.0, label='d_pref Balanced (0.90 m)', zorder=3)
    ax2.axhline(d_tol, color='#ff7f0e', linestyle=':', linewidth=2.2, label='d_pref Tolerant (0.70 m)', zorder=3)
    ax2.axhline(d_contact, color='#d62728', linestyle='-', linewidth=2.5, label='Contact Boundary (0.60 m)', zorder=4)

    # Stationary span for j15
    stat_mask15 = sp15 < 0.03
    in_stat = False
    s_start = None
    first_stat_labeled = False
    for i, is_s in enumerate(stat_mask15):
        if is_s and not in_stat:
            in_stat = True
            s_start = t15[i]
        elif not is_s and in_stat:
            in_stat = False
            if t15[i] - s_start > 1.0:
                lbl = 'Robot Stationary / Yielding Outside Door' if not first_stat_labeled else None
                ax2.axvspan(s_start, t15[i], color='#ffbb33', alpha=0.35, hatch='\\\\', label=lbl)
                first_stat_labeled = True
    if in_stat and (t15[-1] - s_start > 1.0):
        lbl = 'Robot Stationary' if not first_stat_labeled else None
        ax2.axvspan(s_start, t15[-1], color='#ffbb33', alpha=0.35, hatch='\\\\', label=lbl)

    ax2.set_title('Episode doorway_j15 (Jitter = 1.5 s): Standoff Yielding (Zero Contact, Min Dist = 0.65 m)',
                  fontsize=16, fontweight='bold', pad=10)
    ax2.set_xlabel('Simulation Time (seconds)', fontsize=15, fontweight='bold')
    ax2.set_ylabel('Distance (meters)', fontsize=15, fontweight='bold')
    ax2.set_xlim([0, 38.0])
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', frameon=True, facecolor='#ffffff', edgecolor='#cccccc', fontsize=12)

    # Annotation for min distance
    min_d15 = np.min(dist15)
    t_min15 = t15[np.argmin(dist15)]
    ax2.annotate(f'Min Distance: {min_d15:.2f} m\n(Safe: > 0.60 m)',
                 xy=(t_min15, min_d15), xytext=(t_min15 + 2.5, min_d15 + 0.9),
                 arrowprops=dict(arrowstyle="->", color='#2ca02c', lw=2),
                 color='#2ca02c', fontsize=13, fontweight='bold',
                 bbox=dict(boxstyle='round,pad=0.3', facecolor='#e8f5e9', edgecolor='#2ca02c'))

    plt.tight_layout()
    out_path = os.path.join(RES_DIR, 'figure_2b_doorway_timelines.png')
    plt.savefig(out_path, dpi=180, bbox_inches='tight')
    plt.close()
    print(f"Saved Figure 2(b) to {out_path}")

if __name__ == '__main__':
    generate_figure_2a()
    generate_figure_2b()
