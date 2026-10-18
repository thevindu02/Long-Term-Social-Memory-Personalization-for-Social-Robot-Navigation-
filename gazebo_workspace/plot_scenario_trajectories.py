#!/usr/bin/env python3
import json
import math
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from PIL import Image

def load_map(map_pgm_path):
    if os.path.exists(map_pgm_path):
        im = Image.open(map_pgm_path)
        arr = np.array(im)
        arr = np.flipud(arr)
        return arr
    return None

def color_path(ax, points, times, cmap_name, label_prefix, lw=3.0):
    if len(points) < 2:
        return None
    pts = np.array(points)
    segments = np.concatenate([pts[:-1, np.newaxis, :], pts[1:, np.newaxis, :]], axis=1)
    norm = plt.Normalize(min(times), max(times))
    lc = LineCollection(segments, cmap=cmap_name, norm=norm, linewidth=lw, zorder=5)
    lc.set_array(np.array(times[:-1]))
    line = ax.add_collection(lc)
    return line

def plot_trajectory_scenario(scen_name, traj_file, map_pgm_path, out_png, title):
    with open(traj_file) as f:
        data = json.load(f)

    samples = data.get("samples", [])
    if not samples:
        print(f"Error: no samples in {traj_file}")
        return

    t0 = samples[0]["t"]
    r_pts = []
    r_times = []
    h_pts = []
    h_times = []

    for s in samples:
        t_rel = s["t"] - t0
        r_pts.append(s["robot_pos"])
        r_times.append(t_rel)

        h_info = s["human_states"].get("agent1", {})
        hx, hy = h_info.get("pos", (None, None))
        if hx is not None:
            h_pts.append((hx, hy))
            h_times.append(t_rel)

    map_arr = load_map(map_pgm_path)
    map_extent = [-25.0, 25.0, -15.0, 15.0]

    if scen_name == "corridor":
        fig, ax = plt.subplots(figsize=(15, 6), dpi=150)
        ax.set_xlim([-11.5, 11.5])
        ax.set_ylim([-3.5, 3.5])
    else: # doorway
        fig, ax = plt.subplots(figsize=(9, 10), dpi=150)
        ax.set_xlim([-0.8, 6.8])
        ax.set_ylim([-6.5, 3.0])

    if map_arr is not None:
        ax.imshow(map_arr, cmap='bone', extent=map_extent, origin='lower', alpha=0.85)
    else:
        ax.set_facecolor('#222222')

    # Draw Doors
    doors = [
        {"name": "top_1", "w": 2.0, "x": -9.0, "y": 2.0},
        {"name": "bot_1", "w": 1.5, "x": -9.0, "y": -2.0},
        {"name": "top_2", "w": 1.5, "x": -3.0, "y": 2.0},
        {"name": "bot_2", "w": 1.5, "x": -3.0, "y": -2.0},
        {"name": "top_3", "w": 1.5, "x": 3.0, "y": 2.0},
        {"name": "bot_3", "w": 1.0, "x": 3.0, "y": -2.0},
        {"name": "top_4", "w": 1.5, "x": 9.0, "y": 2.0},
        {"name": "bot_4", "w": 1.5, "x": 9.0, "y": -2.0},
    ]
    for d in doors:
        if ax.get_xlim()[0] <= d["x"] <= ax.get_xlim()[1] and ax.get_ylim()[0] <= d["y"] <= ax.get_ylim()[1]:
            ax.plot([d["x"] - d["w"]/2, d["x"] + d["w"]/2], [d["y"], d["y"]], color='#f39c12', linewidth=3, linestyle='--')
            ax.text(d["x"], d["y"] + (0.35 if d["y"] > 0 else -0.45),
                    f"{d['name']} ({d['w']:.1f}m)",
                    color='#f39c12', fontsize=9, fontweight='bold', ha='center', va='center',
                    bbox=dict(boxstyle='round,pad=0.2', facecolor='black', alpha=0.75, edgecolor='#f39c12'))

    # Color paths by time
    # Robot: 'winter' (cyan to blue/green)
    # Human: 'autumn' (red to yellow) or 'copper'
    lc_r = color_path(ax, r_pts, r_times, 'winter', 'Robot', lw=3.0)
    lc_h = color_path(ax, h_pts, h_times, 'autumn', 'Human', lw=3.0)

    # Start / End Markers
    ax.scatter([r_pts[0][0]], [r_pts[0][1]], color='#00ffcc', s=160, marker='o', edgecolor='black', linewidth=1.5, zorder=7, label='Robot Start')
    ax.scatter([r_pts[-1][0]], [r_pts[-1][1]], color='#0066ff', s=180, marker='X', edgecolor='white', linewidth=1.5, zorder=7, label='Robot End')

    if h_pts:
        ax.scatter([h_pts[0][0]], [h_pts[0][1]], color='#ff3300', s=160, marker='o', edgecolor='white', linewidth=1.5, zorder=7, label='Human Start')
        ax.scatter([h_pts[-1][0]], [h_pts[-1][1]], color='#ffcc00', s=180, marker='X', edgecolor='black', linewidth=1.5, zorder=7, label='Human End')

    ax.set_aspect('equal')
    ax.set_title(title, fontsize=12, fontweight='bold', pad=12)
    ax.set_xlabel("X (meters)", fontsize=10)
    ax.set_ylabel("Y (meters)", fontsize=10)
    ax.grid(color='#555555', linestyle=':', linewidth=0.5, alpha=0.5)

    # Colorbars
    if lc_r is not None and lc_h is not None:
        cbar_r = fig.colorbar(lc_r, ax=ax, orientation='horizontal', fraction=0.04, pad=0.12, aspect=35)
        cbar_r.set_label('Robot Trajectory Time (seconds)', fontsize=9)
        cbar_h = fig.colorbar(lc_h, ax=ax, orientation='horizontal', fraction=0.04, pad=0.10, aspect=35)
        cbar_h.set_label('Human Trajectory Time (seconds)', fontsize=9)

    ax.legend(loc='upper right', frameon=True, facecolor='#2c3e50', edgecolor='white', fontsize=8.5)

    plt.tight_layout()
    plt.savefig(out_png, dpi=180, bbox_inches='tight')
    plt.close(fig)
    print(f"Trajectory plot saved to: {out_png}")

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    map_p = os.path.join(base_dir, 'src/social_nav_env/maps/proxy_map.pgm')
    res_dir = os.path.join(base_dir, 'evaluation_results')

    # 1. Corridor
    traj_corr = os.path.join(res_dir, 'trajectory_corridor_robot_present_corridor_j00.json')
    out_corr = os.path.join(res_dir, 'trajectory_plot_corridor.png')
    plot_trajectory_scenario('corridor', traj_corr, map_p, out_corr,
                             "Corridor Scenario Trajectory (Episode: corridor_j00, Jitter=0.0s)\nRobot and Human Paths with Time Progression")

    # 2. Doorway
    traj_door = os.path.join(res_dir, 'trajectory_doorway_robot_present_doorway_j15.json')
    out_door = os.path.join(res_dir, 'trajectory_plot_doorway.png')
    plot_trajectory_scenario('doorway', traj_door, map_p, out_door,
                             "Doorway Scenario Trajectory (Episode: doorway_j15, Jitter=1.5s)\nRobot Yields Outside Doorway bot_3 (1.0m) to Avoid Conflict")

if __name__ == '__main__':
    main()
