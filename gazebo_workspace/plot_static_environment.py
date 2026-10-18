#!/usr/bin/env python3
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image

def load_map(map_pgm_path):
    if os.path.exists(map_pgm_path):
        im = Image.open(map_pgm_path)
        arr = np.array(im)
        arr = np.flipud(arr)
        return arr
    return None

def plot_environment(map_pgm_path, out_png):
    map_arr = load_map(map_pgm_path)
    map_extent = [-25.0, 25.0, -15.0, 15.0]

    fig, ax = plt.subplots(figsize=(16, 9), dpi=150)

    if map_arr is not None:
        ax.imshow(map_arr, cmap='bone', extent=map_extent, origin='lower', alpha=0.9)
    else:
        ax.set_facecolor('#1a1a1a')

    # Zoom to the relevant region: corridor, rooms, and common area
    ax.set_xlim([-13.5, 21.0])
    ax.set_ylim([-9.0, 9.0])
    ax.set_aspect('equal')

    # Environment Area Labels
    # Corridor
    ax.text(0.0, 0.0, "CENTRAL CORRIDOR (Width: 3.7m, Length: 24m)",
            color='#ecf0f1', fontsize=12, fontweight='bold', ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#2c3e50', alpha=0.8, edgecolor='#34495e'))

    # Common Area
    ax.text(16.0, 0.0, "COMMON AREA\n(Dining / Lounge)",
            color='#ecf0f1', fontsize=11, fontweight='bold', ha='center', va='center',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#16a085', alpha=0.7, edgecolor='white'))

    # Room Labels
    room_labels = [
        ("Room top_1", -9.0, 4.0),
        ("Room top_2", -3.0, 4.0),
        ("Room top_3", 3.0, 4.0),
        ("Room top_4", 9.0, 4.0),
        ("Room bot_1", -9.0, -4.0),
        ("Room bot_2", -3.0, -4.0),
        ("Room bot_3 (Bed)", 3.0, -4.0),
        ("Room bot_4", 9.0, -4.0),
    ]
    for r_name, rx, ry in room_labels:
        ax.text(rx, ry, r_name, color='#bdc3c7', fontsize=9, ha='center', va='center',
                bbox=dict(boxstyle='round,pad=0.2', facecolor='#2c3e50', alpha=0.6, edgecolor='#7f8c8d'))

    # Doors with widths
    doors = [
        ("top_1", 2.0, -9.0, 2.0, "N"),
        ("bot_1", 1.5, -9.0, -2.0, "S"),
        ("top_2", 1.5, -3.0, 2.0, "N"),
        ("bot_2", 1.5, -3.0, -2.0, "S"),
        ("top_3", 1.5, 3.0, 2.0, "N"),
        ("bot_3", 1.0, 3.0, -2.0, "S"), # Key narrow doorway!
        ("top_4", 1.5, 9.0, 2.0, "N"),
        ("bot_4", 1.5, 9.0, -2.0, "S"),
    ]

    for dname, dw, dx, dy, orientation in doors:
        # Draw door threshold line
        ax.plot([dx - dw/2, dx + dw/2], [dy, dy], color='#f1c40f', linewidth=3.5, linestyle='-', zorder=6)
        label_y = dy + 0.45 if orientation == "N" else dy - 0.45
        is_bottleneck = (dname == "bot_3")
        box_edge = '#e74c3c' if is_bottleneck else '#f39c12'
        box_face = '#c0392b' if is_bottleneck else '#d35400'
        ax.text(dx, label_y, f"{dname}: {dw:.1f}m", color='white', fontsize=8.5, fontweight='bold',
                ha='center', va='center', zorder=7,
                bbox=dict(boxstyle='round,pad=0.25', facecolor=box_face, alpha=0.85, edgecolor=box_edge))

    # SCENARIO 1: CORRIDOR OVERLAY
    # Robot: start (-9.0, 0.0) -> goal (+9.0, 0.0)
    ax.annotate("", xy=(8.5, 0.5), xytext=(-8.5, 0.5),
                arrowprops=dict(arrowstyle="->", color="#3498db", lw=2.5, linestyle="--"), zorder=8)
    ax.scatter([-9.0], [0.5], color='#3498db', s=160, marker='o', edgecolor='white', linewidth=2, zorder=9, label='Corridor: Robot Start (-9, 0)')
    ax.scatter([9.0], [0.5], color='#2980b9', s=180, marker='*', edgecolor='white', linewidth=1.5, zorder=9, label='Corridor: Robot Goal (+9, 0)')

    # Human: start (+9.0, 0.0) -> goal (-9.0, 0.0)
    ax.annotate("", xy=(-8.5, -0.5), xytext=(8.5, -0.5),
                arrowprops=dict(arrowstyle="->", color="#e67e22", lw=2.5, linestyle="--"), zorder=8)
    ax.scatter([9.0], [-0.5], color='#e67e22', s=160, marker='o', edgecolor='white', linewidth=2, zorder=9, label='Corridor: Human Start (+9, 0)')
    ax.scatter([-9.0], [-0.5], color='#d35400', s=180, marker='*', edgecolor='white', linewidth=1.5, zorder=9, label='Corridor: Human Goal (-9, 0)')

    # SCENARIO 2: DOORWAY OVERLAY
    # Robot: start (3.0, 0.0) -> goal (3.0, -4.0)
    ax.annotate("", xy=(2.7, -3.8), xytext=(2.7, -0.2),
                arrowprops=dict(arrowstyle="->", color="#2ecc71", lw=2.5, linestyle="--"), zorder=8)
    ax.scatter([2.7], [0.0], color='#2ecc71', s=160, marker='o', edgecolor='white', linewidth=2, zorder=9, label='Doorway: Robot Start (3, 0)')
    ax.scatter([2.7], [-4.0], color='#27ae60', s=180, marker='*', edgecolor='white', linewidth=1.5, zorder=9, label='Doorway: Robot Goal (3, -4)')

    # Human: start (3.0, -5.0) -> goal (3.0, 1.0)
    ax.annotate("", xy=(3.3, 0.8), xytext=(3.3, -4.8),
                arrowprops=dict(arrowstyle="->", color="#9b59b6", lw=2.5, linestyle="--"), zorder=8)
    ax.scatter([3.3], [-5.0], color='#9b59b6', s=160, marker='o', edgecolor='white', linewidth=2, zorder=9, label='Doorway: Human Start (3, -5)')
    ax.scatter([3.3], [1.0], color='#8e44ad', s=180, marker='*', edgecolor='white', linewidth=1.5, zorder=9, label='Doorway: Human Goal (3, 1)')

    # Callout for Door bot_3
    ax.annotate("Door bot_3 (1.0m width)\nCritical Passing Bottleneck",
                xy=(3.0, -2.0), xytext=(4.5, -2.7),
                arrowprops=dict(arrowstyle="->", color="#e74c3c", lw=2.0),
                color="#e74c3c", fontsize=10, fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='#2c3e50', edgecolor='#e74c3c', alpha=0.9))

    ax.set_title("Elder Care Facility Simulation Environment\nTop-Down Architecture with Scenario Paths & Labeled Doors",
                 fontsize=14, fontweight='bold', pad=15)
    ax.set_xlabel("X (meters)", fontsize=11)
    ax.set_ylabel("Y (meters)", fontsize=11)
    ax.grid(color='#444444', linestyle=':', linewidth=0.5, alpha=0.5)

    # Clean legend outside
    ax.legend(loc='lower center', bbox_to_anchor=(0.5, -0.18), ncol=4, frameon=True,
              facecolor='#2c3e50', edgecolor='white', fontsize=8.5)

    plt.tight_layout()
    plt.savefig(out_png, dpi=180, bbox_inches='tight')
    plt.close(fig)
    print(f"Static environment figure saved: {out_png}")

if __name__ == '__main__':
    base_dir = os.path.dirname(os.path.abspath(__file__))
    map_p = os.path.join(base_dir, 'src/social_nav_env/maps/proxy_map.pgm')
    out_p = os.path.join(base_dir, 'evaluation_results/static_environment_layout.png')
    plot_environment(map_p, out_p)
