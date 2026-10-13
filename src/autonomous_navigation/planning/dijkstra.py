"""Dijkstra's shortest path planning algorithm implementation."""

import heapq
import time
from typing import Any, Dict, List, Optional, Set, Tuple
import numpy as np

from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.planning.astar import PlanResult
from autonomous_navigation.planning.path_utils import get_neighbors, reconstruct_path


class DijkstraPlanner:
    """Dijkstra's uniform-cost search algorithm for 2D grid maps."""

    def __init__(self, allow_diagonal: bool = True) -> None:
        """Initialize DijkstraPlanner.

        Args:
            allow_diagonal: If True, uses 8-connected grid search.
        """
        self.allow_diagonal = allow_diagonal

    def plan(
        self,
        grid_map: Any,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        cost_weight: float = 1.0,
    ) -> PlanResult:
        """Find the optimal shortest path using uniform-cost exploration.

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

        counter = 0
        open_set: List[Tuple[float, int, Tuple[int, int]]] = []
        heapq.heappush(open_set, (0.0, counter, start_cell))

        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        dist: Dict[Tuple[int, int], float] = {start_cell: 0.0}
        explored_nodes: Set[Tuple[int, int]] = set()

        while open_set:
            cost, _, current = heapq.heappop(open_set)

            if current in explored_nodes:
                continue

            explored_nodes.add(current)

            if current == goal_cell:
                path = reconstruct_path(came_from, current)
                elapsed = (time.perf_counter() - t0) * 1000.0
                return PlanResult(
                    path=path,
                    cost=dist[goal_cell],
                    nodes_explored=len(explored_nodes),
                    execution_time_ms=elapsed,
                    explored_nodes=explored_nodes,
                )

            current_cost = dist[current]
            for neighbor, step_cost in get_neighbors(
                current, grid_map, self.allow_diagonal, cost_weight=cost_weight
            ):
                new_cost = current_cost + step_cost
                if new_cost < dist.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    dist[neighbor] = new_cost
                    counter += 1
                    heapq.heappush(open_set, (new_cost, counter, neighbor))

        # No path found
        elapsed = (time.perf_counter() - t0) * 1000.0
        return PlanResult(
            path=None,
            cost=float("inf"),
            nodes_explored=len(explored_nodes),
            execution_time_ms=elapsed,
            explored_nodes=explored_nodes,
        )
