#!/usr/bin/env python3
import json
import math
import os
import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from PIL import Image

def load_map(map_pgm_path):
    if os.path.exists(map_pgm_path):
        im = Image.open(map_pgm_path)
        # Flip vertically because image origin is top-left while map is bottom-left
        arr = np.array(im)
        arr = np.flipud(arr)
        return arr
    return None

def compute_approach_speed(rx, ry, rvx, rvy, hx, hy):
    dx = hx - rx
    dy = hy - ry
    d = math.hypot(dx, dy)
    if d < 1e-4:
        return 0.0
    ux = dx / d
    uy = dy / d
    closing_v = rvx * ux + rvy * uy
    return max(0.0, closing_v)

def render_animation(traj_file, map_pgm_path, output_gif, output_png, profile="balanced", fps=10, step_stride=1):
    with open(traj_file) as f:
        data = json.load(f)

    scenario = data.get("scenario", "corridor")
    run_id = data.get("run_id", "unknown")
    samples = data.get("samples", [])
    if not samples:
        print(f"Error: No samples in {traj_file}")
        return

    # Profile params
    prof_dict = {
        "sensitive": {"d_pref": 1.20, "v_max": 0.10, "theta_th": 15.0},
        "balanced": {"d_pref": 0.90, "v_max": 0.16, "theta_th": 20.0},
        "tolerant": {"d_pref": 0.70, "v_max": 0.22, "theta_th": 25.0}
    }
    p_params = prof_dict.get(profile.lower(), prof_dict["balanced"])
    d_pref = p_params["d_pref"]
    v_max = p_params["v_max"]
    theta_th = p_params["theta_th"]

    map_arr = load_map(map_pgm_path)
    # Map coordinates: resolution 0.05, origin [-25.0, -15.0]
    map_extent = [-25.0, 25.0, -15.0, 15.0]

    # Set view bounds per scenario
    if scenario == "corridor":
        xlim = [-11.5, 11.5]
        ylim = [-3.8, 4.2]
        figsize = (15, 6)
    else: # doorway
        xlim = [-0.8, 6.8]
        ylim = [-6.5, 3.2]
        figsize = (9, 10)

    fig, ax = plt.subplots(figsize=figsize, dpi=100)

    # Subsample if requested
    samples_sub = samples[::step_stride]
    t0 = samples[0]["t"]

    # All known doors
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

    def draw_background():
        ax.clear()
        if map_arr is not None:
            ax.imshow(map_arr, cmap='bone', extent=map_extent, origin='lower', alpha=0.85)
        else:
            ax.set_facecolor('#222222')

        # Draw door labels that fall within the view
        for d in doors:
            if xlim[0] <= d["x"] <= xlim[1] and ylim[0] <= d["y"] <= ylim[1]:
                ax.plot([d["x"] - d["w"]/2, d["x"] + d["w"]/2], [d["y"], d["y"]], color='#f39c12', linewidth=3, linestyle='--')
                ax.text(d["x"], d["y"] + (0.35 if d["y"] > 0 else -0.45),
                        f"{d['name']} ({d['w']:.1f}m)",
                        color='#f39c12', fontsize=9, fontweight='bold', ha='center', va='center',
                        bbox=dict(boxstyle='round,pad=0.2', facecolor='black', alpha=0.7, edgecolor='#f39c12'))

        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        ax.set_aspect('equal')
        ax.set_xlabel('X (meters)', fontsize=10)
        ax.set_ylabel('Y (meters)', fontsize=10)
        ax.grid(color='#555555', linestyle=':', linewidth=0.5, alpha=0.5)

    # Render frame function
    def render_frame(idx):
        draw_background()
        s = samples_sub[idx]
        sim_time = s["t"] - t0
        rx, ry = s["robot_pos"]
        rvx, rvy = s["robot_vel"]

        h_info = s["human_states"].get("agent1", {})
        hx, hy = h_info.get("pos", (None, None))
        hvx, hvy = h_info.get("vel", (0.0, 0.0))
        h_yaw = h_info.get("heading", 0.0)

        # Robot geometry (radius 0.20)
        robot_circ = patches.Circle((rx, ry), 0.20, facecolor='#3498db', edgecolor='white', linewidth=1.5, zorder=5)
        ax.add_patch(robot_circ)
        # Robot orientation line from velocity or orientation
        rv_mag = math.hypot(rvx, rvy)
        r_heading = math.atan2(rvy, rvx) if rv_mag > 0.02 else 0.0
        ax.plot([rx, rx + 0.35 * math.cos(r_heading)], [ry, ry + 0.35 * math.sin(r_heading)], color='#2980b9', linewidth=2.5, zorder=6)
        ax.text(rx, ry + 0.35, "Robot (r=0.20)", color='#3498db', fontsize=8, fontweight='bold', ha='center', zorder=7)

        dist = 999.0
        v_app = 0.0
        is_contact = False
        is_prox_viol = False
        is_app_viol = False

        if hx is not None:
            # Human geometry (radius 0.40)
            human_circ = patches.Circle((hx, hy), 0.40, facecolor='#e67e22', edgecolor='white', linewidth=1.5, zorder=5)
            ax.add_patch(human_circ)
            ax.plot([hx, hx + 0.50 * math.cos(h_yaw)], [hy, hy + 0.50 * math.sin(h_yaw)], color='#d35400', linewidth=2.5, zorder=6)
            ax.text(hx, hy + 0.55, "Human (r=0.40)", color='#e67e22', fontsize=8, fontweight='bold', ha='center', zorder=7)

            # d_pref ring around human
            pref_ring = patches.Circle((hx, hy), d_pref, fill=False, edgecolor='#9b59b6', linestyle='--', linewidth=1.5, zorder=4)
            ax.add_patch(pref_ring)

            # Distance line
            dist = math.hypot(rx - hx, ry - hy)
            ax.plot([rx, hx], [ry, hy], color='#bdc3c7', linestyle=':', linewidth=1.2, zorder=3)
            mid_x, mid_y = (rx + hx) * 0.5, (ry + hy) * 0.5
            ax.text(mid_x, mid_y + 0.2, f"d = {dist:.2f} m", color='#ffffff', fontsize=9, fontweight='bold',
                    ha='center', bbox=dict(boxstyle='round,pad=0.2', facecolor='#2c3e50', alpha=0.85, edgecolor='none'), zorder=8)

            v_app = compute_approach_speed(rx, ry, rvx, rvy, hx, hy)
            is_contact = dist < 0.60
            is_prox_viol = dist < d_pref
            is_app_viol = (dist < 3.0) and (v_app > v_max)

        # Title & Badges
        prox_col = '#e74c3c' if is_prox_viol else '#2ecc71'
        prox_txt = "PROXEMICS VIOLATION" if is_prox_viol else "PROXEMICS OK"

        app_col = '#e74c3c' if is_app_viol else '#2ecc71'
        app_txt = f"APPROACH VIOLATION ({v_app:.2f}>{v_max:.2f})" if is_app_viol else f"APPROACH OK ({v_app:.2f} m/s)"

        title_str = (f"Scenario: {scenario.capitalize()} ({run_id}) | Sim Time: {sim_time:5.1f} s | Profile: {profile.capitalize()} (d_pref={d_pref:.2f}m, v_max={v_max:.2f}m/s)\n"
                     f"[{prox_txt}]   [{app_txt}]" + ("   [CONTACT!]" if is_contact else ""))

        title_color = '#e74c3c' if is_contact else '#ffffff'
        banner_face = '#c0392b' if is_contact else '#2c3e50'
        ax.text(0.5, 0.97, title_str, transform=ax.transAxes, color=title_color, fontsize=9.5, fontweight='bold',
                ha='center', va='top', bbox=dict(boxstyle='round,pad=0.4', facecolor=banner_face, alpha=0.9, edgecolor='white'), zorder=20)

    # 1. Save first frame as PNG
    print(f"Generating first frame for {run_id}...", flush=True)
    render_frame(0)
    plt.savefig(output_png, dpi=120, bbox_inches='tight')
    print(f"First frame saved to: {output_png}", flush=True)

    # 2. Render animation
    print(f"Rendering {len(samples_sub)} frames to {output_gif} at {fps} fps...", flush=True)
    anim = animation.FuncAnimation(fig, render_frame, frames=len(samples_sub), repeat=False)
    anim.save(output_gif, writer='pillow', fps=fps)
    plt.close(fig)

    gif_size_mb = os.path.getsize(output_gif) / (1024 * 1024)
    print(f"Animation complete! Saved to {output_gif} ({gif_size_mb:.2f} MB)", flush=True)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--traj', required=True, help='Path to trajectory json')
    parser.add_argument('--map', default='/home/thevindu/SocialGym2/gazebo_workspace/src/social_nav_env/maps/proxy_map.pgm')
    parser.add_argument('--out-gif', required=True, help='Output gif file')
    parser.add_argument('--out-png', required=True, help='Output first frame png')
    parser.add_argument('--profile', default='balanced', choices=['sensitive', 'balanced', 'tolerant'])
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--stride', type=int, default=1)
    args = parser.parse_args()

    render_animation(args.traj, args.map, args.out_gif, args.out_png, args.profile, args.fps, args.stride)

if __name__ == '__main__':
    main()
