"""Collision-safe geometric path smoothing module.

Reduces redundant turns and waypoints from discrete grid paths using shortcutting
and line-of-sight segment verification against ground-truth collision checkers.
All waypoint inputs and outputs are in continuous world coordinates (meters).
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple

from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.navigation.collision_checker import CollisionChecker
from autonomous_navigation.planning.path_utils import calculate_path_length


@dataclass
class SmoothingResult:
    """Summary of a path smoothing operation."""
    smoothed_path: List[Tuple[float, float]]
    original_path: List[Tuple[float, float]]
    is_smoothed: bool
    is_path_valid: bool
    original_length: float
    smoothed_length: float
    original_waypoint_count: int
    smoothed_waypoint_count: int
    length_reduction_ratio: float


class PathSmoother:
    """Performs collision-safe shortcutting and path smoothing on world-meter waypoints."""

    def __init__(
        self,
        collision_checker: Optional[CollisionChecker] = None,
        step_size: float = 0.1,
    ) -> None:
        """Initialize PathSmoother.

        Args:
            collision_checker: CollisionChecker instance for continuous segment validation.
            step_size: Sub-step resolution in meters for swept continuous segment checks.
                       Must be a finite, positive number.

        Raises:
            ValueError: If step_size is <= 0, NaN, or infinite.
        """
        self.collision_checker = collision_checker or CollisionChecker()
        try:
            step_size_f = float(step_size)
        except (TypeError, ValueError) as err:
            raise ValueError(f"step_size must be a valid float, got: {step_size!r}") from err

        if not math.isfinite(step_size_f) or step_size_f <= 0.0:
            raise ValueError(f"step_size must be a positive finite float, got: {step_size_f}")

        self.step_size = step_size_f

    def smooth_path(
        self,
        path: List[Tuple[float, float]],
        grid_map: GridMap,
    ) -> SmoothingResult:
        """Smooth a sequence of continuous world-meter waypoints.

        If the path is invalid, empty, or cannot be safely shortened, the original
        path is preserved without modification and is_smoothed is False.
        The safety status of the resulting path is reported via is_path_valid.

        Note:
            Retaining an invalid input path (e.g. empty, out-of-bounds, or collision-intersecting)
            is a fallback to avoid mutating planner output, not an assertion that the path is safe.
            In such cases, is_path_valid is explicitly False.

        Args:
            path: List of (x, y) waypoints in world meters.
            grid_map: Occupancy grid map (provides resolution and obstacle geometries).

        Returns:
            SmoothingResult containing original and smoothed paths with comparative metrics and validity.
        """
        orig_count = len(path) if path else 0
        orig_len = calculate_path_length(path) if path else 0.0

        # Edge case: empty path
        if not path:
            return SmoothingResult(
                smoothed_path=[],
                original_path=[],
                is_smoothed=False,
                is_path_valid=False,
                original_length=0.0,
                smoothed_length=0.0,
                original_waypoint_count=0,
                smoothed_waypoint_count=0,
                length_reduction_ratio=0.0,
            )

        # Validate input path safety.
        # Single-point path: check state validity.
        # Multi-point path: check all segments.
        if len(path) == 1:
            is_valid = self.collision_checker.is_state_valid(path[0][0], path[0][1], grid_map)
        else:
            is_valid = self.collision_checker.is_path_valid(path, grid_map)

        if not is_valid:
            # Fallback immediately to the original path without smoothing; report is_path_valid=False
            return SmoothingResult(
                smoothed_path=list(path),
                original_path=list(path),
                is_smoothed=False,
                is_path_valid=False,
                original_length=orig_len,
                smoothed_length=orig_len,
                original_waypoint_count=orig_count,
                smoothed_waypoint_count=orig_count,
                length_reduction_ratio=0.0,
            )

        # If path has 1 or 2 points and is valid, it is already minimal and cannot be shortened.
        # Preserve original path as smoothed_path, is_smoothed=False, is_path_valid=True.
        if len(path) <= 2:
            return SmoothingResult(
                smoothed_path=list(path),
                original_path=list(path),
                is_smoothed=False,
                is_path_valid=True,
                original_length=orig_len,
                smoothed_length=orig_len,
                original_waypoint_count=orig_count,
                smoothed_waypoint_count=orig_count,
                length_reduction_ratio=0.0,
            )

        # Greedy shortcutting: start from first waypoint, find furthest reachable waypoint
        current_smoothed = [path[0]]
        current_idx = 0
        n = len(path)

        while current_idx < n - 1:
            furthest_reachable = current_idx + 1
            # Search backwards from the end of the path towards current_idx + 1
            for target_idx in range(n - 1, current_idx + 1, -1):
                p_start = path[current_idx]
                p_end = path[target_idx]
                if self.collision_checker.is_path_segment_valid(
                    p_start, p_end, grid_map, step_size=self.step_size
                ):
                    furthest_reachable = target_idx
                    break
            current_smoothed.append(path[furthest_reachable])
            current_idx = furthest_reachable

        # Validate complete smoothed path continuously
        if not self.collision_checker.is_path_valid(current_smoothed, grid_map):
            # Fall back strictly to original path on any validation failure.
            # Since the original path was already verified valid above, it is safe.
            return SmoothingResult(
                smoothed_path=list(path),
                original_path=list(path),
                is_smoothed=False,
                is_path_valid=True,
                original_length=orig_len,
                smoothed_length=orig_len,
                original_waypoint_count=orig_count,
                smoothed_waypoint_count=orig_count,
                length_reduction_ratio=0.0,
            )

        # Check start and goal positions preserved within floating point tolerance
        start_dist = math.hypot(current_smoothed[0][0] - path[0][0], current_smoothed[0][1] - path[0][1])
        goal_dist = math.hypot(current_smoothed[-1][0] - path[-1][0], current_smoothed[-1][1] - path[-1][1])
        if start_dist > 1e-5 or goal_dist > 1e-5:
            return SmoothingResult(
                smoothed_path=list(path),
                original_path=list(path),
                is_smoothed=False,
                is_path_valid=True,
                original_length=orig_len,
                smoothed_length=orig_len,
                original_waypoint_count=orig_count,
                smoothed_waypoint_count=orig_count,
                length_reduction_ratio=0.0,
            )

        smoothed_len = calculate_path_length(current_smoothed)
        reduction = (orig_len - smoothed_len) / orig_len if orig_len > 1e-6 else 0.0

        return SmoothingResult(
            smoothed_path=current_smoothed,
            original_path=list(path),
            is_smoothed=(len(current_smoothed) < len(path) or smoothed_len < orig_len - 1e-4),
            is_path_valid=True,
            original_length=round(orig_len, 4),
            smoothed_length=round(smoothed_len, 4),
            original_waypoint_count=orig_count,
            smoothed_waypoint_count=len(current_smoothed),
            length_reduction_ratio=round(reduction, 4),
        )
