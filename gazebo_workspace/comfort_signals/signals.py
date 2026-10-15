"""
Comfort signal computation library for social robot navigation.
Implements:
1. Proxemics violation & physical contact event detection.
2. Robot approach speed closing velocity gated by active interaction distance.
3. Trajectory irregularity comparing human actual path against robot-absent nominal baseline
   using geometric polyline projection (OR logic between angular deviation and detour ratio).
"""

import math
from typing import List, Tuple, Optional, Dict, Any

def wrap_to_pi(angle_rad: float) -> float:
    """Wraps an angle in radians to [-pi, pi]."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi

def compute_distance(pos_a: Tuple[float, float], pos_b: Tuple[float, float]) -> float:
    """Computes Euclidean 2D distance between two points."""
    return math.hypot(pos_a[0] - pos_b[0], pos_a[1] - pos_b[1])

def check_contact(
    distance: float,
    robot_radius: float = 0.20,
    human_radius: float = 0.40,
    tol: float = 1e-6
) -> bool:
    """
    Checks if physical contact occurs (center distance < sum of physical radii).
    Default: 0.20m (SDF robot cylinder) + 0.40m (HuNavSim human) = 0.60m.
    Strictly less than (robot_radius + human_radius) with floating-point tolerance 1e-6.
    Contact is logged as an event and does not terminate navigation.
    """
    threshold = robot_radius + human_radius
    return distance < (threshold - tol)

def compute_approach_speed(
    r_robot: Tuple[float, float],
    v_robot: Tuple[float, float],
    r_human: Tuple[float, float],
    d_active: float = 3.0
) -> float:
    """
    Computes the robot's closing speed toward the human:
      v_approach = max(0.0, v_r . u_{r->h})
    where u_{r->h} is the unit vector from robot to human.
    Clamped at 0.0 when the robot is moving away or orthogonal.
    Gated strictly by distance: returns 0.0 if d >= d_active.
    """
    dx = r_human[0] - r_robot[0]
    dy = r_human[1] - r_robot[1]
    dist = math.hypot(dx, dy)

    # Gating by interaction distance threshold
    if dist >= d_active or dist < 1e-9:
        return 0.0

    # Unit vector pointing from robot to human
    ux = dx / dist
    uy = dy / dist

    # Dot product of robot velocity with line of sight unit vector
    closing_speed = v_robot[0] * ux + v_robot[1] * uy

    # Clamped at 0.0 when robot moves away or orthogonal
    return max(0.0, float(closing_speed))

def compute_polyline_length(polyline: List[Tuple[float, float]]) -> float:
    """Computes total 2D path length (arc length) of a polyline."""
    if len(polyline) < 2:
        return 0.0
    total_len = 0.0
    for i in range(len(polyline) - 1):
        total_len += math.hypot(
            polyline[i + 1][0] - polyline[i][0],
            polyline[i + 1][1] - polyline[i][1]
        )
    return total_len

def project_point_to_polyline(
    point: Tuple[float, float],
    polyline: List[Tuple[float, float]]
) -> Tuple[Tuple[float, float], float, float]:
    """
    Projects a 2D point onto a polyline geometrically (NOT by time index).
    Returns:
      (closest_point, lateral_distance, tangent_angle_rad)
    """
    if len(polyline) < 2:
        if len(polyline) == 1:
            dist = compute_distance(point, polyline[0])
            return (polyline[0], dist, 0.0)
        return (point, 0.0, 0.0)

    best_dist_sq = float('inf')
    best_point = polyline[0]
    best_tangent = 0.0

    px, py = point

    for i in range(len(polyline) - 1):
        x1, y1 = polyline[i]
        x2, y2 = polyline[i + 1]

        vx = x2 - x1
        vy = y2 - y1
        seg_len_sq = vx * vx + vy * vy

        if seg_len_sq < 1e-12:
            proj_x, proj_y = x1, y1
            tangent = 0.0
        else:
            wx = px - x1
            wy = py - y1
            t = (wx * vx + wy * vy) / seg_len_sq
            t = max(0.0, min(1.0, t))
            proj_x = x1 + t * vx
            proj_y = y1 + t * vy
            tangent = math.atan2(vy, vx)

        dist_sq = (px - proj_x) ** 2 + (py - proj_y) ** 2
        if dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            best_point = (proj_x, proj_y)
            best_tangent = tangent

    return (best_point, math.sqrt(best_dist_sq), best_tangent)

def compute_detour_ratio(
    actual_length: float,
    nominal_length: float,
    min_nominal_length: float = 0.20
) -> float:
    """
    Computes path length increase ratio relative to nominal path:
      eta_L = (L_actual - L_nominal) / L_nominal
    Guarded: if L_nominal < min_nominal_length (e.g. stationary human), returns 0.0.
    Clamped at 0.0 (no negative detour ratio).
    """
    if nominal_length < min_nominal_length:
        return 0.0
    detour = (actual_length - nominal_length) / nominal_length
    return max(0.0, float(detour))

def compute_angular_deviation(
    human_heading_rad: float,
    nominal_polyline: List[Tuple[float, float]],
    human_pos: Tuple[float, float],
    robot_pos: Optional[Tuple[float, float]] = None,
    d_active: float = 3.0
) -> Tuple[float, float]:
    """
    Computes angular deviation between human heading and nominal path tangent at closest point.
    Gated: only evaluated when distance to robot < d_active.
    Returns:
      (angular_deviation_deg, lateral_deviation_m)
      If robot_pos is provided and distance >= d_active, angular_deviation_deg is 0.0.
    """
    _, lateral_dist, nom_tangent = project_point_to_polyline(human_pos, nominal_polyline)

    if robot_pos is not None:
        d_robot = compute_distance(human_pos, robot_pos)
        if d_robot >= d_active:
            return (0.0, lateral_dist)

    diff = wrap_to_pi(human_heading_rad - nom_tangent)
    ang_dev_deg = abs(math.degrees(diff))
    return (ang_dev_deg, lateral_dist)
