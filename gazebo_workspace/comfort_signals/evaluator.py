"""
Trajectory evaluation engine for comfort signal scoring.
Takes synchronized episode trajectories and scores them offline against
one or all comfort profiles (Sensitive, Balanced, Tolerant).
"""

from typing import List, Tuple, Dict, Any, Optional
from dataclasses import dataclass, asdict
import math
import subprocess

from .signals import (
    compute_distance,
    check_contact,
    compute_approach_speed,
    compute_polyline_length,
    compute_detour_ratio,
    compute_angular_deviation,
    project_point_to_polyline,
    wrap_to_pi,
)
from .profiles import ComfortProfile, PROFILES, get_profile

@dataclass
class TimeSample:
    t: float                              # Sim time (seconds)
    robot_pos: Tuple[float, float]        # (x, y) ground truth
    robot_vel: Tuple[float, float]        # (vx, vy)
    human_states: Dict[str, Dict[str, Any]] # agent_name -> {'pos': (x,y), 'vel': (vx,vy), 'heading': rad}

@dataclass
class HumanEvaluationResult:
    agent_name: str
    min_distance: float
    contact_events: int
    proxemics_violation_time: float
    proxemics_violation_samples: int
    max_approach_speed: float
    approach_violation_time: float
    approach_violation_samples: int
    actual_path_length: float
    nominal_path_length: float
    detour_ratio: float
    angular_violation_time: float
    angular_violation_samples: int
    max_angular_deviation_deg: float
    max_lateral_deviation_m: float
    irregularity_violated: bool
    proxemics_violated: bool
    approach_violated: bool
    contact_violated: bool
    overall_violated: bool

@dataclass
class EpisodeSummary:
    profile_name: str
    scenario_name: str
    sim_duration: float
    sample_count: int
    expected_samples: int
    seed: Optional[int]
    git_commit: str
    nav2_params: Dict[str, Any]
    human_results: Dict[str, HumanEvaluationResult]
    total_contact_events: int
    total_proxemics_violation_time: float
    total_approach_violation_time: float
    total_angular_violation_time: float
    any_irregularity_violated: bool
    overall_episode_violated: bool

def get_git_commit_hash() -> str:
    """Retrieves current git commit hash if available."""
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            stderr=subprocess.DEVNULL,
            text=True
        ).strip()
        return out
    except Exception:
        return "unknown"

def extract_smoothed_motion_headings(
    samples: List[TimeSample],
    agent_name: str,
    window_size: int = 5,
    min_speed: float = 0.10
) -> List[Tuple[float, float, bool]]:
    """
    Computes smoothed motion heading from displacement/velocity for agent_name.
    Returns list of (speed, smoothed_heading_rad, is_valid) for each sample.
    """
    speeds = []
    raw_headings = []

    for i, s in enumerate(samples):
        if agent_name not in s.human_states:
            speeds.append(0.0)
            raw_headings.append(0.0)
            continue
        h = s.human_states[agent_name]
        vx, vy = h.get('vel', (0.0, 0.0))
        spd = math.hypot(vx, vy)
        speeds.append(spd)

        if i > 0 and agent_name in samples[i - 1].human_states:
            prev_pos = samples[i - 1].human_states[agent_name]['pos']
            curr_pos = h['pos']
            dx = curr_pos[0] - prev_pos[0]
            dy = curr_pos[1] - prev_pos[1]
            dt = s.t - samples[i - 1].t
            if dt > 1e-4 and math.hypot(dx, dy) / dt > 0.05:
                heading = math.atan2(dy, dx)
            elif spd > 1e-4:
                heading = math.atan2(vy, vx)
            else:
                heading = h.get('heading', 0.0)
        elif spd > 1e-4:
            heading = math.atan2(vy, vx)
        else:
            heading = h.get('heading', 0.0)
        raw_headings.append(heading)

    n = len(samples)
    results = []
    half_w = window_size // 2

    for i in range(n):
        spd = speeds[i]
        is_valid = (spd > min_speed)
        w_start = max(0, i - half_w)
        w_end = min(n, i + half_w + 1)

        sin_sum = 0.0
        cos_sum = 0.0
        valid_cnt = 0
        for j in range(w_start, w_end):
            if speeds[j] > min_speed:
                sin_sum += math.sin(raw_headings[j])
                cos_sum += math.cos(raw_headings[j])
                valid_cnt += 1

        if valid_cnt > 0:
            smoothed = math.atan2(sin_sum, cos_sum)
        else:
            smoothed = raw_headings[i]
        results.append((spd, smoothed, is_valid))

    return results

class TrajectoryEvaluator:
    def __init__(self, profile: ComfortProfile):
        self.profile = profile

    def evaluate(
        self,
        samples: List[TimeSample],
        nominal_human_paths: Dict[str, List[Tuple[float, float]]],
        scenario_name: str = "default_scenario",
        seed: Optional[int] = None,
        nav2_params: Optional[Dict[str, Any]] = None,
        dt_nominal: float = 0.1
    ) -> EpisodeSummary:
        """
        Evaluates a complete episode trajectory under self.profile.
        """
        if not samples:
            raise ValueError("Cannot evaluate empty trajectory samples.")

        t_start = samples[0].t
        t_end = samples[-1].t
        sim_duration = max(0.0, t_end - t_start)
        expected_samples = max(1, int(round(sim_duration / dt_nominal))) if sim_duration > 0 else len(samples)

        nav_params = nav2_params or {
            "max_vel_x": 0.26,
            "max_speed_xy": 0.26,
            "inflation_radius": 0.55,
            "cost_scaling_factor": 3.0,
            "robot_radius": 0.22,
        }

        # Track per-human data
        agent_names = set()
        for s in samples:
            agent_names.update(s.human_states.keys())

        human_results: Dict[str, HumanEvaluationResult] = {}
        total_contacts = 0
        total_prox_time = 0.0
        total_app_time = 0.0
        total_ang_time = 0.0
        any_irreg = False

        is_corridor = scenario_name.lower().startswith("corridor")
        is_doorway = scenario_name.lower().startswith("doorway")

        for name in sorted(agent_names):
            min_dist = float('inf')
            contact_events = 0
            prox_violation_time = 0.0
            prox_violation_samples = 0
            max_app_speed = 0.0
            app_violation_time = 0.0
            app_violation_samples = 0
            angular_violation_time = 0.0
            angular_violation_samples = 0

            actual_polyline: List[Tuple[float, float]] = []
            max_ang_dev = 0.0
            max_lat_dev = 0.0

            nom_poly = nominal_human_paths.get(name, [])
            smoothed_headings = extract_smoothed_motion_headings(samples, name, window_size=5, min_speed=0.10)

            for idx, s in enumerate(samples):
                if name not in s.human_states:
                    continue

                h_data = s.human_states[name]
                h_pos = h_data['pos']
                actual_polyline.append(h_pos)

                # Time step weight
                if idx > 0:
                    dt = max(0.0, s.t - samples[idx - 1].t)
                else:
                    dt = dt_nominal

                # 1. Proxemics & Contact
                dist = compute_distance(s.robot_pos, h_pos)
                if dist < min_dist:
                    min_dist = dist

                if check_contact(dist, self.profile.robot_radius, self.profile.human_radius):
                    contact_events += 1

                if dist < self.profile.d_pref:
                    prox_violation_time += dt
                    prox_violation_samples += 1

                # 2. Approach speed
                v_app = compute_approach_speed(
                    s.robot_pos,
                    s.robot_vel,
                    h_pos,
                    d_active=self.profile.d_active
                )
                if v_app > max_app_speed:
                    max_app_speed = v_app

                if v_app > self.profile.v_max_approach:
                    app_violation_time += dt
                    app_violation_samples += 1

                # 3. Angular & Lateral deviation (geometric, gated by d < d_active and speed > 0.10, before goal)
                spd, smoothed_heading, is_valid_speed = smoothed_headings[idx]
                at_goal = False
                if is_corridor and h_pos[0] <= -8.0:
                    at_goal = True
                elif is_doorway and h_pos[1] >= 0.5:
                    at_goal = True

                if len(nom_poly) >= 2:
                    _, lat_dev, nom_tang = project_point_to_polyline(h_pos, nom_poly)
                    if lat_dev > max_lat_dev:
                        max_lat_dev = lat_dev

                    if dist < self.profile.d_active and is_valid_speed and not at_goal:
                        ang_dev = abs(math.degrees(wrap_to_pi(smoothed_heading - nom_tang)))
                        if ang_dev > max_ang_dev:
                            max_ang_dev = ang_dev

                        if ang_dev > self.profile.angular_deviation_threshold_deg:
                            angular_violation_time += dt
                            angular_violation_samples += 1

            # Polyline lengths & Detour ratio
            actual_len = compute_polyline_length(actual_polyline)
            nom_len = compute_polyline_length(nom_poly) if len(nom_poly) >= 2 else 0.0
            detour = compute_detour_ratio(
                actual_len,
                nom_len,
                min_nominal_length=self.profile.min_nominal_path_length
            )

            # Irregularity violation: OR logic between Detour ratio and Time-in-violation
            irreg_violated = (
                detour > self.profile.detour_ratio_threshold or
                angular_violation_time > self.profile.angular_time_threshold_sec
            )

            prox_violated = (prox_violation_samples > 0)
            approach_violated = (app_violation_samples > 0)
            contact_violated = (contact_events > 0)
            overall_violated = (
                prox_violated or
                approach_violated or
                irreg_violated or
                contact_violated
            )

            res = HumanEvaluationResult(
                agent_name=name,
                min_distance=min_dist,
                contact_events=contact_events,
                proxemics_violation_time=prox_violation_time,
                proxemics_violation_samples=prox_violation_samples,
                max_approach_speed=max_app_speed,
                approach_violation_time=app_violation_time,
                approach_violation_samples=app_violation_samples,
                actual_path_length=actual_len,
                nominal_path_length=nom_len,
                detour_ratio=detour,
                angular_violation_time=angular_violation_time,
                angular_violation_samples=angular_violation_samples,
                max_angular_deviation_deg=max_ang_dev,
                max_lateral_deviation_m=max_lat_dev,
                irregularity_violated=irreg_violated,
                proxemics_violated=prox_violated,
                approach_violated=approach_violated,
                contact_violated=contact_violated,
                overall_violated=overall_violated
            )
            human_results[name] = res

            total_contacts += contact_events
            total_prox_time += prox_violation_time
            total_app_time += app_violation_time
            total_ang_time += angular_violation_time
            if irreg_violated:
                any_irreg = True

        overall_ep_violated = any(hr.overall_violated for hr in human_results.values())

        return EpisodeSummary(
            profile_name=self.profile.name,
            scenario_name=scenario_name,
            sim_duration=sim_duration,
            sample_count=len(samples),
            expected_samples=expected_samples,
            seed=seed,
            git_commit=get_git_commit_hash(),
            nav2_params=nav_params,
            human_results=human_results,
            total_contact_events=total_contacts,
            total_proxemics_violation_time=total_prox_time,
            total_approach_violation_time=total_app_time,
            total_angular_violation_time=total_ang_time,
            any_irregularity_violated=any_irreg,
            overall_episode_violated=overall_ep_violated
        )

def score_episode_all_profiles(
    samples: List[TimeSample],
    nominal_human_paths: Dict[str, List[Tuple[float, float]]],
    scenario_name: str = "default_scenario",
    seed: Optional[int] = None,
    nav2_params: Optional[Dict[str, Any]] = None,
    dt_nominal: float = 0.1
) -> Dict[str, EpisodeSummary]:
    """
    Offline Multi-Profile Scorer:
    Evaluates the SAME trajectory against Sensitive, Balanced, and Tolerant profiles.
    """
    summaries = {}
    for p_name, profile in PROFILES.items():
        evaluator = TrajectoryEvaluator(profile)
        summaries[p_name] = evaluator.evaluate(
            samples=samples,
            nominal_human_paths=nominal_human_paths,
            scenario_name=scenario_name,
            seed=seed,
            nav2_params=nav2_params,
            dt_nominal=dt_nominal
        )
    return summaries
