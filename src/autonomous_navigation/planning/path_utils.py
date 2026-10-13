"""Path calculations, distance heuristics, and graph neighborhood utilities."""

import math
from typing import Any, Dict, List, Tuple

from autonomous_navigation.environment.grid_map import GridMap


def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Calculate standard Euclidean (L2) distance."""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def manhattan_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Calculate Manhattan (L1) distance."""
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def octile_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """Calculate Octile distance (admissible and consistent for 8-connected grids)."""
    dx = abs(p1[0] - p2[0])
    dy = abs(p1[1] - p2[1])
    return max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy)


def calculate_path_length(path: List[Tuple[float, float]]) -> float:
    """Calculate cumulative Euclidean length of a waypoint sequence."""
    if len(path) < 2:
        return 0.0
    length = 0.0
    for i in range(len(path) - 1):
        length += euclidean_distance(path[i], path[i + 1])
    return length


def reconstruct_path(
    came_from: Dict[Tuple[int, int], Tuple[int, int]],
    current: Tuple[int, int],
) -> List[Tuple[int, int]]:
    """Reconstruct path list from predecessor mapping."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def is_passable(map_obj: Any, x: int, y: int) -> bool:
    """Check traversability on either GridMap or Costmap."""
    if hasattr(map_obj, "is_traversable"):
        return map_obj.is_traversable(x, y)
    return map_obj.is_free(x, y)


def get_neighbors(
    node: Tuple[int, int],
    grid_map: Any,
    allow_diagonal: bool = True,
    cost_weight: float = 1.0,
) -> List[Tuple[Tuple[int, int], float]]:
    """Retrieve traversable neighboring cells and movement step costs.

    Supports both binary GridMap and layered Costmap instances. When evaluated
    on a Costmap, soft clearance penalties are added to the step distance.

    Args:
        node: (x, y) coordinates of the current cell.
        grid_map: GridMap or Costmap instance.
        allow_diagonal: If True, returns 8-connected neighbors; otherwise 4-connected.
        cost_weight: Weighting multiplier for soft costmap penalties.

    Returns:
        List of ((nx, ny), step_cost) pairs.
    """
    x, y = node
    neighbors: List[Tuple[Tuple[int, int], float]] = []

    has_costmap = hasattr(grid_map, "get_traversal_penalty")

    # 4-connected cardinal movements (cost = 1.0)
    cardinal_moves = [
        ((x + 1, y), 1.0),
        ((x - 1, y), 1.0),
        ((x, y + 1), 1.0),
        ((x, y - 1), 1.0),
    ]

    for (nx, ny), base_cost in cardinal_moves:
        if grid_map.is_valid(nx, ny) and is_passable(grid_map, nx, ny):
            step_cost = base_cost
            if has_costmap:
                step_cost += grid_map.get_traversal_penalty(nx, ny, weight=cost_weight)
            neighbors.append(((nx, ny), step_cost))

    if allow_diagonal:
        # Diagonal moves (cost = sqrt(2) approx 1.41421356)
        diag_moves = [
            ((x + 1, y + 1), math.sqrt(2.0), (x + 1, y), (x, y + 1)),
            ((x - 1, y + 1), math.sqrt(2.0), (x - 1, y), (x, y + 1)),
            ((x + 1, y - 1), math.sqrt(2.0), (x + 1, y), (x, y - 1)),
            ((x - 1, y - 1), math.sqrt(2.0), (x - 1, y), (x, y - 1)),
        ]
        for (nx, ny), base_cost, adj1, adj2 in diag_moves:
            if grid_map.is_valid(nx, ny) and is_passable(grid_map, nx, ny):
                # Avoid cutting corners through obstacles or lethal zones
                if is_passable(grid_map, adj1[0], adj1[1]) and is_passable(grid_map, adj2[0], adj2[1]):
                    step_cost = base_cost
                    if has_costmap:
                        step_cost += grid_map.get_traversal_penalty(nx, ny, weight=cost_weight)
                    neighbors.append(((nx, ny), step_cost))

    return neighbors
