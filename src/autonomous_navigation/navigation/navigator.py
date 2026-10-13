"""High-level trajectory tracking and navigation coordinator.

Coordinate convention
---------------------
All continuous positions (robot pose, waypoints, goal centre) are expressed in
**world metres**.  Grid-cell indices (integer ``(gx, gy)`` pairs) are converted
to world metres via ``(gx + 0.5) * resolution`` before being stored as path
waypoints or used in distance comparisons.

Tolerance parameters (``waypoint_tolerance``, goal tolerance) are in
**world metres** and should scale with ``resolution`` if the caller changes the
grid resolution.  The defaults (0.15 m waypoint, 0.4 m goal) are appropriate
for resolution = 1.0 m/cell.
"""

import math
from typing import Any, List, Optional, Tuple

from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.navigation.collision_checker import CollisionChecker
from autonomous_navigation.planning.astar import PlanResult


def _resolution_of(map_obj: Any) -> float:
    """Return map resolution, searching GridMap and Costmap attribute names."""
    res = getattr(map_obj, "resolution", None)
    if res is not None:
        return float(res)
    # Costmap wraps a GridMap under .grid_map
    inner = getattr(map_obj, "grid_map", None)
    if inner is not None:
        res = getattr(inner, "resolution", None)
        if res is not None:
            return float(res)
    return 1.0


class Navigator:
    """Manages path tracking, waypoint progression, and dynamic replanning."""

    def __init__(
        self,
        planner: Any,
        collision_checker: Optional[CollisionChecker] = None,
        waypoint_tolerance: float = 0.15,
    ) -> None:
        """Initialize Navigator.

        Args:
            planner: Planner instance implementing a .plan(grid_map, start, goal) method.
            collision_checker: Optional CollisionChecker (world-metre radius).
            waypoint_tolerance: Distance threshold in **world metres** to advance to the
                next waypoint (default 0.15 m).
        """
        self.planner = planner
        self.collision_checker = collision_checker or CollisionChecker()
        self.waypoint_tolerance = float(waypoint_tolerance)

        self.current_path: List[Tuple[float, float]] = []
        self.original_path: List[Tuple[float, float]] = []
        self.waypoint_index: int = 0
        self.goal: Optional[Tuple[int, int]] = None
        self._goal_resolution: float = 1.0   # resolution used when goal was set
        self.last_plan_result: Optional[PlanResult] = None
        self.last_smoothing_result: Optional[Any] = None

    def plan_to_goal(
        self,
        grid_map: Any,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        cost_weight: float = 1.0,
        smooth_path: bool = False,
    ) -> bool:
        """Plan a path from start to goal and initialise tracking state.

        Args:
            grid_map: Current occupancy grid or Costmap.
            start: Start (gx, gy) grid-cell indices.
            goal: Target (gx, gy) grid-cell indices.
            cost_weight: Scaling multiplier for soft clearance penalties.
            smooth_path: If True, applies collision-safe shortcutting while
                         retaining raw path in self.original_path.

        Returns:
            True if a valid path was found, False otherwise.
        """
        self.goal = goal
        self._goal_resolution = _resolution_of(grid_map)
        self.last_plan_result = self.planner.plan(
            grid_map, start, goal, cost_weight=cost_weight
        )

        if self.last_plan_result.path:
            # Convert grid-cell indices to world-metre cell-centre coordinates.
            res = self._goal_resolution
            raw_path = [
                ((float(x) + 0.5) * res, (float(y) + 0.5) * res)
                for x, y in self.last_plan_result.path
            ]
            self.original_path = list(raw_path)
            self.current_path = list(raw_path)

            if smooth_path:
                from autonomous_navigation.planning.path_smoother import PathSmoother
                # Underlying GridMap instance needed for collision checking if wrapped in Costmap
                underlying_grid = getattr(grid_map, "grid_map", grid_map)
                smoother = PathSmoother(collision_checker=self.collision_checker)
                smooth_res = smoother.smooth_path(raw_path, underlying_grid)
                self.last_smoothing_result = smooth_res

                if not smooth_res.is_path_valid:
                    self.current_path = []
                    self.original_path = []
                    self.waypoint_index = 0
                    return False

                self.current_path = smooth_res.smoothed_path
            else:
                self.last_smoothing_result = None

            self.waypoint_index = 0
            return True

        self.current_path = []
        self.original_path = []
        self.last_smoothing_result = None
        self.waypoint_index = 0
        return False

    def is_goal_reached(
        self,
        current_pos: Tuple[float, float],
        tolerance: Optional[float] = None,
    ) -> bool:
        """Check whether robot world position is within tolerance of the target goal.

        Args:
            current_pos: Robot position in world metres.
            tolerance: Distance threshold in world metres (default 0.4 m).
        """
        if not self.goal:
            return False

        tol = tolerance if tolerance is not None else 0.4
        res = self._goal_resolution
        goal_center = (
            (float(self.goal[0]) + 0.5) * res,
            (float(self.goal[1]) + 0.5) * res,
        )
        dist = math.hypot(current_pos[0] - goal_center[0], current_pos[1] - goal_center[1])
        return dist <= tol

    def get_current_waypoint(self) -> Optional[Tuple[float, float]]:
        """Return the world-metre coordinate of the current active waypoint."""
        if not self.current_path or self.waypoint_index >= len(self.current_path):
            return None
        return self.current_path[self.waypoint_index]

    def update_waypoint_progress(
        self,
        current_pos: Tuple[float, float],
        grid_map: Optional[GridMap] = None,
    ) -> None:
        """Advance waypoint index if current waypoint has been reached and turn is safe.

        Args:
            current_pos: Robot's current world-metre position.
            grid_map: Optional grid map to verify that advancing early does not cut
                      corners into obstacles.
        """
        while self.waypoint_index < len(self.current_path):
            target = self.current_path[self.waypoint_index]
            dist = math.hypot(current_pos[0] - target[0], current_pos[1] - target[1])
            if dist <= self.waypoint_tolerance:
                # If grid_map is provided and there is a subsequent waypoint, ensure
                # shortcut does not clip obstacle.
                if grid_map is not None and self.waypoint_index + 1 < len(self.current_path):
                    next_target = self.current_path[self.waypoint_index + 1]
                    if not self.collision_checker.is_path_segment_valid(
                        current_pos, next_target, grid_map
                    ):
                        break
                self.waypoint_index += 1
            else:
                break

    def is_path_blocked(self, grid_map: GridMap) -> bool:
        """Check if any upcoming segment in current path is now obstructed."""
        if not self.current_path or self.waypoint_index >= len(self.current_path):
            return False

        remaining_path = self.current_path[self.waypoint_index:]
        return not self.collision_checker.is_path_valid(remaining_path, grid_map)

    def replan(
        self,
        grid_map: GridMap,
        current_pos: Tuple[float, float],
    ) -> bool:
        """Attempt to replan from current world position to goal.

        Args:
            grid_map: Environment map (GridMap or Costmap).
            current_pos: Robot's current world-metre coordinates.

        Returns:
            True if replan succeeded with a collision-free path, False otherwise.
        """
        if self.goal is None:
            return False

        res = _resolution_of(grid_map)
        # Convert world position to grid cell index
        start_cell = (int(math.floor(current_pos[0] / res)), int(math.floor(current_pos[1] / res)))
        if not grid_map.is_valid(*start_cell) or grid_map.is_obstacle(*start_cell):
            return False

        plan_res = self.planner.plan(grid_map, start_cell, self.goal)
        if not plan_res.path:
            return False

        candidate_path = [
            ((float(x) + 0.5) * res, (float(y) + 0.5) * res)
            for x, y in plan_res.path
        ]
        full_path = [current_pos] + candidate_path
        if not self.collision_checker.is_path_valid(full_path, grid_map):
            return False

        self.last_plan_result = plan_res
        self.current_path = candidate_path
        self._goal_resolution = res
        self.waypoint_index = 0
        return True
