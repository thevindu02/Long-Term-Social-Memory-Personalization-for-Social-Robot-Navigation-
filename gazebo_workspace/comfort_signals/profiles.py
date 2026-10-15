"""
Comfort profile specifications for social robot navigation evaluation.
Defines Sensitive, Balanced, and Tolerant profiles with empirical thresholds.
"""

from dataclasses import dataclass
from typing import Dict

@dataclass(frozen=True)
class ComfortProfile:
    name: str
    d_pref: float                         # Preferred proxemic distance (m)
    v_max_approach: float                 # Maximum acceptable robot approach speed (m/s)
    detour_ratio_threshold: float         # Trajectory irregularity: path length increase ratio threshold
    angular_deviation_threshold_deg: float# Trajectory irregularity: angular deviation threshold (deg)
    d_active: float = 3.0                 # Distance gating for active interaction (m)
    robot_radius: float = 0.20            # SDF robot cylinder radius (m)
    human_radius: float = 0.40            # Human collision radius (m)
    contact_distance: float = 0.60        # Physical contact threshold: r_robot + r_human (m)
    min_nominal_path_length: float = 0.20 # Guard against near-zero division for stationary humans (m)

PROFILES: Dict[str, ComfortProfile] = {
    "sensitive": ComfortProfile(
        name="Sensitive",
        d_pref=1.20,
        v_max_approach=0.10,
        detour_ratio_threshold=0.05,
        angular_deviation_threshold_deg=15.0,
        d_active=3.0,
        robot_radius=0.20,
        human_radius=0.40,
        contact_distance=0.60,
        min_nominal_path_length=0.20,
    ),
    "balanced": ComfortProfile(
        name="Balanced",
        d_pref=0.90,
        v_max_approach=0.16,
        detour_ratio_threshold=0.10,
        angular_deviation_threshold_deg=30.0,
        d_active=3.0,
        robot_radius=0.20,
        human_radius=0.40,
        contact_distance=0.60,
        min_nominal_path_length=0.20,
    ),
    "tolerant": ComfortProfile(
        name="Tolerant",
        d_pref=0.70,
        v_max_approach=0.22,
        detour_ratio_threshold=0.20,
        angular_deviation_threshold_deg=45.0,
        d_active=3.0,
        robot_radius=0.20,
        human_radius=0.40,
        contact_distance=0.60,
        min_nominal_path_length=0.20,
    ),
}

def get_profile(name: str) -> ComfortProfile:
    key = name.strip().lower()
    if key not in PROFILES:
        raise ValueError(f"Unknown comfort profile '{name}'. Choose from: {list(PROFILES.keys())}")
    return PROFILES[key]
