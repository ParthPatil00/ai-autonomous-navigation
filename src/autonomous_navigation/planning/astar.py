"""A* path planning algorithm implementation."""

from dataclasses import dataclass
import heapq
import time
from typing import Any, Dict, List, Optional, Set, Tuple

from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.planning.path_utils import (
    euclidean_distance,
    get_neighbors,
    manhattan_distance,
    octile_distance,
    reconstruct_path,
)
import numpy as np
import warnings


@dataclass
class PlanResult:
    """Outcome of a path planning query."""
    path: Optional[List[Tuple[int, int]]]
    cost: float
    nodes_explored: int
    execution_time_ms: float
    explored_nodes: Set[Tuple[int, int]]


class AStarPlanner:
    """A* search algorithm for 2D grid maps."""

    def __init__(
        self,
        heuristic: str = "euclidean",
        allow_diagonal: bool = True,
    ) -> None:
        """Initialize AStarPlanner.

        Args:
            heuristic: 'euclidean', 'octile', or 'manhattan'.
            allow_diagonal: If True, uses 8-connected grid search.
        """
        self.heuristic_type = heuristic.lower()
        self.allow_diagonal = allow_diagonal

        if self.allow_diagonal and self.heuristic_type == "manhattan":
            warnings.warn(
                "Manhattan heuristic is inadmissible on 8-connected grids (allow_diagonal=True) "
                "and may result in suboptimal paths. Use 'octile' or 'euclidean' for optimal planning.",
                UserWarning,
                stacklevel=2,
            )

    def _heuristic(self, p1: Tuple[int, int], p2: Tuple[int, int]) -> float:
        if self.heuristic_type == "manhattan":
            return manhattan_distance(p1, p2)
        elif self.heuristic_type == "octile":
            return octile_distance(p1, p2)
        return euclidean_distance(p1, p2)

    def plan(
        self,
        grid_map: Any,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        cost_weight: float = 1.0,
    ) -> PlanResult:
        """Find the shortest obstacle-free path from start to goal.

        Supports both binary GridMap and layered Costmap. When a Costmap is supplied,
        impassable cells (physical and inscribed) are avoided and soft clearance
        penalties are factored into the traversal cost.

        Args:
            grid_map: GridMap or Costmap instance.
            start: (x, y) start cell coordinates.
            goal: (x, y) goal cell coordinates.
            cost_weight: Scaling factor for soft costmap penalties (default 1.0).

        Returns:
            PlanResult detailing discovered path, cost, and search stats.
        """
        t0 = time.perf_counter()

        # Robust input validation
        try:
            if len(start) != 2 or len(goal) != 2:
                return PlanResult(None, float("inf"), 0, 0.0, set())
            start_x, start_y = start[0], start[1]
            goal_x, goal_y = goal[0], goal[1]
            if (
                isinstance(start_x, bool)
                or isinstance(start_y, bool)
                or isinstance(goal_x, bool)
                or isinstance(goal_y, bool)
            ):
                return PlanResult(None, float("inf"), 0, 0.0, set())
            if not isinstance(start_x, (int, np.integer)) or not isinstance(start_y, (int, np.integer)):
                return PlanResult(None, float("inf"), 0, 0.0, set())
            if not isinstance(goal_x, (int, np.integer)) or not isinstance(goal_y, (int, np.integer)):
                return PlanResult(None, float("inf"), 0, 0.0, set())
            start_cell = (int(start_x), int(start_y))
            goal_cell = (int(goal_x), int(goal_y))
        except (TypeError, ValueError, IndexError):
            return PlanResult(None, float("inf"), 0, 0.0, set())

        if not grid_map.is_valid(*start_cell) or not grid_map.is_valid(*goal_cell):
            return PlanResult(None, float("inf"), 0, 0.0, set())

        is_blocked = (
            (grid_map.is_lethal(*start_cell) or grid_map.is_lethal(*goal_cell))
            if hasattr(grid_map, "is_lethal")
            else (grid_map.is_obstacle(*start_cell) or grid_map.is_obstacle(*goal_cell))
        )
        if is_blocked:
            return PlanResult(None, float("inf"), 0, 0.0, set())

        if start_cell == goal_cell:
            elapsed = (time.perf_counter() - t0) * 1000.0
            return PlanResult([start_cell], 0.0, 1, elapsed, {start_cell})

        # Priority queue stores tuples: (f_score, h_score, counter, current_node)
        counter = 0
        h_start = self._heuristic(start_cell, goal_cell)
        open_set: List[Tuple[float, float, int, Tuple[int, int]]] = []
        heapq.heappush(open_set, (h_start, h_start, counter, start_cell))

        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score: Dict[Tuple[int, int], float] = {start_cell: 0.0}
        explored_nodes: Set[Tuple[int, int]] = set()

        while open_set:
            _, _, _, current = heapq.heappop(open_set)

            if current in explored_nodes:
                continue

            explored_nodes.add(current)

            if current == goal_cell:
                path = reconstruct_path(came_from, current)
                elapsed = (time.perf_counter() - t0) * 1000.0
                return PlanResult(
                    path=path,
                    cost=g_score[goal_cell],
                    nodes_explored=len(explored_nodes),
                    execution_time_ms=elapsed,
                    explored_nodes=explored_nodes,
                )

            current_g = g_score[current]
            for neighbor, step_cost in get_neighbors(
                current, grid_map, self.allow_diagonal, cost_weight=cost_weight
            ):
                tentative_g = current_g + step_cost
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h_val = self._heuristic(neighbor, goal_cell)
                    f_score = tentative_g + h_val
                    counter += 1
                    heapq.heappush(open_set, (f_score, h_val, counter, neighbor))

        # No path found
        elapsed = (time.perf_counter() - t0) * 1000.0
        return PlanResult(
            path=None,
            cost=float("inf"),
            nodes_explored=len(explored_nodes),
            execution_time_ms=elapsed,
            explored_nodes=explored_nodes,
        )
