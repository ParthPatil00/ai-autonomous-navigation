"""Quantitative navigation performance evaluation metrics."""

from dataclasses import asdict, dataclass
import math
from typing import Any, Dict, List, Tuple

from autonomous_navigation.planning.path_utils import calculate_path_length, euclidean_distance


@dataclass
class NavigationMetrics:
    """Consolidated metrics summary for a single navigation run."""
    success: bool
    path_length: float
    straight_line_distance: float
    path_efficiency: float
    execution_time_ms: float
    steps_taken: int
    collision_count: int
    smoothness_score: float

    def to_dict(self) -> Dict[str, Any]:
        """Convert metrics to dictionary."""
        return asdict(self)


def compute_path_smoothness(path: List[Tuple[float, float]]) -> float:
    """Compute path smoothness as the cumulative angular deviation along the path.

    Lower score indicates smoother path with fewer abrupt heading transitions.

    Args:
        path: Ordered list of 2D coordinates.

    Returns:
        Cumulative absolute heading change in radians.
    """
    if len(path) < 3:
        return 0.0

    total_angle_change = 0.0
    for i in range(len(path) - 2):
        p1, p2, p3 = path[i], path[i + 1], path[i + 2]
        theta1 = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        theta2 = math.atan2(p3[1] - p2[1], p3[0] - p2[0])

        diff = abs(theta2 - theta1)
        # Normalize to [0, pi]
        while diff > math.pi:
            diff = abs(diff - 2.0 * math.pi)
        total_angle_change += diff

    return total_angle_change


def evaluate_run(
    trajectory: List[Tuple[float, float]],
    start: Tuple[float, float],
    goal: Tuple[float, float],
    success: bool,
    execution_time_ms: float,
    collision_count: int = 0,
) -> NavigationMetrics:
    """Evaluate trajectory against navigational objectives.

    Args:
        trajectory: Recorded list of robot coordinates.
        start: Starting coordinate.
        goal: Target destination coordinate.
        success: Whether the robot arrived at the goal safely.
        execution_time_ms: Elapsed planning/simulation time in milliseconds.
        collision_count: Number of collisions detected.

    Returns:
        Populated NavigationMetrics dataclass.
    """
    path_len = calculate_path_length(trajectory)
    straight_dist = euclidean_distance(start, goal)
    efficiency = (straight_dist / path_len) if path_len > 0 else 0.0
    smoothness = compute_path_smoothness(trajectory)

    return NavigationMetrics(
        success=bool(success),
        path_length=round(path_len, 3),
        straight_line_distance=round(straight_dist, 3),
        path_efficiency=round(efficiency, 3),
        execution_time_ms=round(execution_time_ms, 2),
        steps_taken=len(trajectory),
        collision_count=int(collision_count),
        smoothness_score=round(smoothness, 3),
    )
