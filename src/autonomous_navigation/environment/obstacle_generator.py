"""Procedural obstacle generation for grid environments."""

import math
from typing import List, Optional, Tuple
import numpy as np

from autonomous_navigation.environment.grid_map import GridMap


class ObstacleGenerator:
    """Generates procedural static obstacles on a GridMap."""

    @staticmethod
    def add_boundary_walls(grid_map: GridMap) -> None:
        """Add boundary obstacles along the entire outer perimeter of the map."""
        for x in range(grid_map.width):
            grid_map.set_obstacle(x, 0)
            grid_map.set_obstacle(x, grid_map.height - 1)
        for y in range(grid_map.height):
            grid_map.set_obstacle(0, y)
            grid_map.set_obstacle(grid_map.width - 1, y)

    @staticmethod
    def add_random_obstacles(
        grid_map: GridMap,
        density: float = 0.20,
        seed: Optional[int] = None,
        safe_positions: Optional[List[Tuple[int, int]]] = None,
        safe_radius: int = 2,
    ) -> None:
        """Populate the grid map with random obstacles.

        Args:
            grid_map: Target GridMap instance.
            density: Fraction of cells to populate with obstacles (0.0 to 1.0).
            seed: Optional random generator seed for reproducibility.
            safe_positions: List of (x, y) coordinates where obstacles must not be placed.
            safe_radius: Clearance radius around safe positions.
        """
        rng = np.random.default_rng(seed)
        safe_positions = safe_positions or []

        # Mask safe zones around specified positions
        safe_mask = np.zeros((grid_map.height, grid_map.width), dtype=bool)
        for sx, sy in safe_positions:
            min_x = max(0, sx - safe_radius)
            max_x = min(grid_map.width - 1, sx + safe_radius)
            min_y = max(0, sy - safe_radius)
            max_y = min(grid_map.height - 1, sy + safe_radius)
            safe_mask[min_y : max_y + 1, min_x : max_x + 1] = True

        random_matrix = rng.random((grid_map.height, grid_map.width))
        obstacle_indices = np.where((random_matrix < density) & (~safe_mask))

        for y, x in zip(obstacle_indices[0], obstacle_indices[1]):
            grid_map.set_obstacle(int(x), int(y))

    @staticmethod
    def add_box_obstacle(grid_map: GridMap, x: int, y: int, width: int, height: int) -> None:
        """Add a rectangular obstacle."""
        for cx in range(x, min(x + width, grid_map.width)):
            for cy in range(y, min(y + height, grid_map.height)):
                grid_map.set_obstacle(cx, cy)

    @staticmethod
    def add_circle_obstacle(grid_map: GridMap, cx: int, cy: int, radius: int) -> None:
        """Add a circular obstacle."""
        for x in range(max(0, cx - radius), min(grid_map.width, cx + radius + 1)):
            for y in range(max(0, cy - radius), min(grid_map.height, cy + radius + 1)):
                if math.hypot(x - cx, y - cy) <= radius:
                    grid_map.set_obstacle(x, y)
