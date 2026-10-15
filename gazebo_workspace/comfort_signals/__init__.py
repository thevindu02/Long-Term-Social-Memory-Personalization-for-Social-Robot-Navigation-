"""
Comfort signals package for social robot navigation evaluation.
"""

from .signals import (
    compute_distance,
    check_contact,
    compute_approach_speed,
    compute_polyline_length,
    compute_detour_ratio,
    compute_angular_deviation,
    project_point_to_polyline,
    wrap_to_pi
)
from .profiles import ComfortProfile, PROFILES, get_profile
from .evaluator import (
    TimeSample,
    HumanEvaluationResult,
    EpisodeSummary,
    TrajectoryEvaluator,
    score_episode_all_profiles,
    get_git_commit_hash
)

__all__ = [
    "compute_distance",
    "check_contact",
    "compute_approach_speed",
    "compute_polyline_length",
    "compute_detour_ratio",
    "compute_angular_deviation",
    "project_point_to_polyline",
    "wrap_to_pi",
    "ComfortProfile",
    "PROFILES",
    "get_profile",
    "TimeSample",
    "HumanEvaluationResult",
    "EpisodeSummary",
    "TrajectoryEvaluator",
    "score_episode_all_profiles",
    "get_git_commit_hash"
]
