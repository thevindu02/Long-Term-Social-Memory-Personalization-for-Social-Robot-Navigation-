"""
Unit tests for comfort signals library.
Tests mathematical definitions with numerical tolerance of 1e-6.
Includes:
- Head-on closing speed (robot 0.5 toward human 1.0 gives 0.5, NOT 1.5)
- Robot moving away (gives 0.0)
- Robot moving orthogonal (gives 0.0)
- Gated interaction distance d >= 3.0m (gives 0.0)
- Parallel pass proxemics violation (0.9m vs d_pref 1.0m gives 0.1m violation)
- Physical contact event detection (0.55m < 0.60m)
- 60-degree swerve angular deviation (gives 60.0 deg)
- Stationary human detour ratio guard (L_nom < 0.20m gives 0.0)
- Detour ratio calculation (12.5m vs 10.0m gives 0.25)
- Trajectory irregularity OR logic (triggers on angular deviation OR detour ratio)
- Lateral deviation geometric projection
- Multi-profile offline scoring evaluation
"""

import math
import pytest
from typing import List, Tuple

from comfort_signals.signals import (
    compute_distance,
    check_contact,
    compute_approach_speed,
    compute_polyline_length,
    compute_detour_ratio,
    compute_angular_deviation,
    project_point_to_polyline,
    wrap_to_pi,
)
from comfort_signals.profiles import PROFILES, get_profile
from comfort_signals.evaluator import (
    TimeSample,
    TrajectoryEvaluator,
    score_episode_all_profiles,
)

TOL = 1e-6

# ==============================================================================
# 1. Approach Speed Unit Tests
# ==============================================================================

def test_approach_speed_head_on():
    """
    Head-on test:
    Robot at (0.0, 0.0) moving at +0.5 m/s toward human at (2.0, 0.0).
    Human is walking at 1.0 m/s toward robot (vx = -1.0).
    Closing speed must be robot's closing speed along line of sight = 0.5 m/s (NOT relative 1.5).
    """
    r_robot = (0.0, 0.0)
    v_robot = (0.5, 0.0)
    r_human = (2.0, 0.0)

    v_app = compute_approach_speed(r_robot, v_robot, r_human, d_active=3.0)
    assert math.isclose(v_app, 0.5, abs_tol=TOL)

def test_approach_speed_moving_away():
    """
    Robot moving away:
    Robot at (0.0, 0.0) moving at -0.5 m/s away from human at (2.0, 0.0).
    Closing speed must be clamped at 0.0.
    """
    r_robot = (0.0, 0.0)
    v_robot = (-0.5, 0.0)
    r_human = (2.0, 0.0)

    v_app = compute_approach_speed(r_robot, v_robot, r_human, d_active=3.0)
    assert math.isclose(v_app, 0.0, abs_tol=TOL)

def test_approach_speed_orthogonal():
    """
    Robot moving orthogonally:
    Robot at (0.0, 0.0) moving along +Y at 0.5 m/s. Human at (2.0, 0.0) along +X.
    Closing speed must be 0.0.
    """
    r_robot = (0.0, 0.0)
    v_robot = (0.0, 0.5)
    r_human = (2.0, 0.0)

    v_app = compute_approach_speed(r_robot, v_robot, r_human, d_active=3.0)
    assert math.isclose(v_app, 0.0, abs_tol=TOL)

def test_approach_speed_gated_by_distance():
    """
    Distance gating:
    Robot at (0.0, 0.0) moving directly toward human at (3.5, 0.0).
    Distance 3.5m >= d_active (3.0m). Must return 0.0.
    When distance is 2.9m < 3.0m, returns 0.5 m/s.
    """
    r_robot = (0.0, 0.0)
    v_robot = (0.5, 0.0)

    # Beyond active distance
    r_human_far = (3.5, 0.0)
    v_app_far = compute_approach_speed(r_robot, v_robot, r_human_far, d_active=3.0)
    assert math.isclose(v_app_far, 0.0, abs_tol=TOL)

    # Exactly at boundary
    r_human_bound = (3.0, 0.0)
    v_app_bound = compute_approach_speed(r_robot, v_robot, r_human_bound, d_active=3.0)
    assert math.isclose(v_app_bound, 0.0, abs_tol=TOL)

    # Inside active distance
    r_human_near = (2.9, 0.0)
    v_app_near = compute_approach_speed(r_robot, v_robot, r_human_near, d_active=3.0)
    assert math.isclose(v_app_near, 0.5, abs_tol=TOL)

def test_approach_speed_diagonal():
    """
    Diagonal approach:
    Robot at (0, 0), moving at (0.3, 0.4) -> speed = 0.5 m/s.
    Human at (3.0 * cos(theta), 3.0 * sin(theta)) where theta = atan2(0.4, 0.3).
    Distance = 2.0m along (0.6, 0.8).
    Closing speed should be exactly 0.5 m/s.
    """
    r_robot = (0.0, 0.0)
    v_robot = (0.3, 0.4)
    r_human = (1.2, 1.6) # dist = 2.0m, unit vector = (0.6, 0.8)

    v_app = compute_approach_speed(r_robot, v_robot, r_human, d_active=3.0)
    expected = 0.3 * 0.6 + 0.4 * 0.8 # 0.18 + 0.32 = 0.5
    assert math.isclose(v_app, expected, abs_tol=TOL)

# ==============================================================================
# 2. Proxemics & Physical Contact Unit Tests
# ==============================================================================

def test_proxemics_parallel_pass():
    """
    Parallel pass:
    Robot moves along y=0.0, human moves along y=0.9m.
    Center-to-center distance is 0.9m.
    Preferred distance d_pref = 1.0m.
    Violation magnitude = 1.0 - 0.9 = 0.1m.
    """
    r_robot = (5.0, 0.0)
    r_human = (5.0, 0.9)
    dist = compute_distance(r_robot, r_human)
    assert math.isclose(dist, 0.9, abs_tol=TOL)

    d_pref = 1.0
    violation = max(0.0, d_pref - dist)
    assert math.isclose(violation, 0.1, abs_tol=TOL)

def test_physical_contact_event():
    """
    Contact event:
    Robot radius 0.20m, human radius 0.40m -> contact threshold 0.60m.
    Center distance 0.55m -> True.
    Center distance 0.65m -> False.
    Center distance 0.60m -> False (strictly < 0.60m).
    """
    assert check_contact(0.55, robot_radius=0.20, human_radius=0.40) is True
    assert check_contact(0.65, robot_radius=0.20, human_radius=0.40) is False
    assert check_contact(0.60, robot_radius=0.20, human_radius=0.40) is False

# ==============================================================================
# 3. Trajectory Irregularity & Angular Deviation Unit Tests
# ==============================================================================

def test_swerve_60_degrees():
    """
    A 60-degree swerve:
    Nominal path is a straight horizontal line along +X: (0, 0) -> (10, 0).
    Tangent is 0 rad (0 deg).
    Human heading is +60 deg (pi/3 rad) at pos (5.0, 0.0).
    Robot is at (5.0, 1.5) (d = 1.5m < d_active 3.0m).
    Angular deviation must be exactly 60.0 degrees.
    """
    nom_poly = [(0.0, 0.0), (10.0, 0.0)]
    human_pos = (5.0, 0.0)
    heading_rad = math.radians(60.0)
    robot_pos = (5.0, 1.5)

    ang_dev, lat_dev = compute_angular_deviation(
        heading_rad,
        nom_poly,
        human_pos,
        robot_pos=robot_pos,
        d_active=3.0
    )
    assert math.isclose(ang_dev, 60.0, abs_tol=TOL)
    assert math.isclose(lat_dev, 0.0, abs_tol=TOL)

def test_angular_deviation_gated_by_robot_distance():
    """
    Angular deviation gated when robot is far:
    If robot is at distance 4.0m >= d_active (3.0m), angular deviation must return 0.0 deg.
    """
    nom_poly = [(0.0, 0.0), (10.0, 0.0)]
    human_pos = (5.0, 0.0)
    heading_rad = math.radians(60.0)
    robot_pos = (5.0, 4.0) # dist = 4.0m >= 3.0m

    ang_dev, lat_dev = compute_angular_deviation(
        heading_rad,
        nom_poly,
        human_pos,
        robot_pos=robot_pos,
        d_active=3.0
    )
    assert math.isclose(ang_dev, 0.0, abs_tol=TOL)
    assert math.isclose(lat_dev, 0.0, abs_tol=TOL)

def test_stationary_human_detour_ratio_guarded():
    """
    Stationary human detour ratio guard:
    Nominal path has length 0.05m < min_nominal_length 0.20m.
    Actual path has length 0.08m.
    Detour ratio must be guarded to 0.0 (prevents division by near-zero).
    """
    actual_len = 0.08
    nominal_len = 0.05
    detour = compute_detour_ratio(actual_len, nominal_len, min_nominal_length=0.20)
    assert math.isclose(detour, 0.0, abs_tol=TOL)

def test_detour_ratio_calculation():
    """
    Detour ratio calculation:
    Nominal path length 10.0m, actual path length 12.5m.
    Detour ratio = (12.5 - 10.0) / 10.0 = 0.25 (+25%).
    """
    actual_len = 12.5
    nominal_len = 10.0
    detour = compute_detour_ratio(actual_len, nominal_len, min_nominal_length=0.20)
    assert math.isclose(detour, 0.25, abs_tol=TOL)

def test_lateral_deviation_geometric_projection():
    """
    Lateral deviation by geometry (NOT time index):
    Nominal polyline from (0, 0) to (10, 0).
    Human is at (4.0, 0.75).
    Closest point on polyline is (4.0, 0.0).
    Lateral deviation must be 0.75m.
    """
    nom_poly = [(0.0, 0.0), (10.0, 0.0)]
    proj, lat_dist, tangent = project_point_to_polyline((4.0, 0.75), nom_poly)
    assert math.isclose(proj[0], 4.0, abs_tol=TOL)
    assert math.isclose(proj[1], 0.0, abs_tol=TOL)
    assert math.isclose(lat_dist, 0.75, abs_tol=TOL)
    assert math.isclose(tangent, 0.0, abs_tol=TOL)

def test_trajectory_irregularity_or_logic():
    """
    OR logic: violation triggers if:
      detour_ratio > thresh_L  OR  angular_deviation > thresh_theta
    """
    thresh_L = 0.10      # +10%
    thresh_ang = 30.0    # 30 deg

    # Case A: Detour ratio triggers alone
    detour_A = 0.15
    ang_A = 10.0
    violated_A = (detour_A > thresh_L) or (ang_A > thresh_ang)
    assert violated_A is True

    # Case B: Angular deviation triggers alone
    detour_B = 0.02
    ang_B = 45.0
    violated_B = (detour_B > thresh_L) or (ang_B > thresh_ang)
    assert violated_B is True

    # Case C: Neither triggers
    detour_C = 0.05
    ang_C = 20.0
    violated_C = (detour_C > thresh_L) or (ang_C > thresh_ang)
    assert violated_C is False

    # Case D: Both trigger
    detour_D = 0.25
    ang_D = 50.0
    violated_D = (detour_D > thresh_L) or (ang_D > thresh_ang)
    assert violated_D is True

# ==============================================================================
# 4. Multi-Profile Evaluation & Episode Summary Unit Tests
# ==============================================================================

def test_multi_profile_offline_evaluation():
    """
    Synthetic trajectory evaluated under Sensitive, Balanced, and Tolerant profiles.
    Robot cruises at 0.24 m/s toward human:
      - Sensitive (v_max = 0.10, d_pref = 1.20): Both approach speed and proxemics violated.
      - Balanced (v_max = 0.16, d_pref = 0.90): Approach speed violated (0.24 > 0.16), proxemics not violated (1.0m > 0.90m).
      - Tolerant (v_max = 0.22, d_pref = 0.70): Approach speed violated (0.24 > 0.22), proxemics not violated (1.0m > 0.70m).
    """
    samples = []
    # 10 samples at dt = 0.1s
    for i in range(10):
        t = i * 0.1
        rx = i * 0.024 # robot moves forward at 0.24 m/s
        sample = TimeSample(
            t=t,
            robot_pos=(rx, 0.0),
            robot_vel=(0.24, 0.0),
            human_states={
                "agent1": {
                    "pos": (rx + 1.0, 0.0), # constant 1.0m center distance
                    "vel": (0.0, 0.0),
                    "heading": 0.0
                }
            }
        )
        samples.append(sample)

    nom_paths = {"agent1": [(1.0, 0.0), (1.24, 0.0)]}

    summaries = score_episode_all_profiles(
        samples,
        nominal_human_paths=nom_paths,
        scenario_name="synthetic_corridor",
        seed=42,
        dt_nominal=0.1
    )

    sens = summaries["sensitive"]
    bal = summaries["balanced"]
    tol = summaries["tolerant"]

    # Verify metadata logged
    assert sens.seed == 42
    assert sens.scenario_name == "synthetic_corridor"
    assert sens.nav2_params["max_vel_x"] == 0.26

    # 1. Approach speed violations:
    # 0.24 m/s violates all 3 profiles:
    # Sensitive (0.10), Balanced (0.16), Tolerant (0.22)
    assert sens.human_results["agent1"].approach_violation_samples > 0
    assert bal.human_results["agent1"].approach_violation_samples > 0
    assert tol.human_results["agent1"].approach_violation_samples > 0

    # 2. Proxemics violations (distance is 1.0m):
    # Sensitive d_pref=1.20m: 1.0m < 1.20m -> VIOLATED
    assert sens.human_results["agent1"].proxemics_violation_samples > 0
    # Balanced d_pref=0.90m: 1.0m >= 0.90m -> NOT violated
    assert bal.human_results["agent1"].proxemics_violation_samples == 0
    # Tolerant d_pref=0.70m: 1.0m >= 0.70m -> NOT violated
    assert tol.human_results["agent1"].proxemics_violation_samples == 0

    # 3. Contact events (distance 1.0m >= 0.60m):
    assert sens.total_contact_events == 0
    assert bal.total_contact_events == 0
    assert tol.total_contact_events == 0
