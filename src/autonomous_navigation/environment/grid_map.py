"""2D Occupancy Grid Map representation."""

import math
from typing import Tuple
import numpy as np


class GridMap:
    """Represents a 2D discrete occupancy grid map."""

    def __init__(self, width: int, height: int, resolution: float = 1.0) -> None:
        """Initialize empty grid map where 0 represents free space, 1 represents obstacle.

        Args:
            width: Number of columns (cells along X-axis).
            height: Number of rows (cells along Y-axis).
            resolution: Real-world scale in meters per cell.
        """
        if width <= 0 or height <= 0:
            raise ValueError(f"Width and height must be positive, got ({width}, {height})")
        if resolution <= 0:
            raise ValueError(f"Resolution must be positive, got {resolution}")

        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)

    def is_valid(self, x: int, y: int) -> bool:
        """Check whether cell coordinates are valid integer indices within the grid bounds."""
        if isinstance(x, bool) or isinstance(y, bool):
            return False
        if not isinstance(x, (int, np.integer)) or not isinstance(y, (int, np.integer)):
            return False
        return 0 <= int(x) < self.width and 0 <= int(y) < self.height

    def is_free(self, x: int, y: int) -> bool:
        """Check whether cell is within bounds and unoccupied."""
        return self.is_valid(x, y) and self.grid[y, x] == 0

    def is_obstacle(self, x: int, y: int) -> bool:
        """Check whether cell is occupied or out of bounds."""
        if not self.is_valid(x, y):
            return True
        return bool(self.grid[y, x] == 1)

    def set_obstacle(self, x: int, y: int) -> None:
        """Mark cell as occupied obstacle if within bounds."""
        if self.is_valid(x, y):
            self.grid[y, x] = 1

    def set_free(self, x: int, y: int) -> None:
        """Mark cell as free space if within bounds."""
        if self.is_valid(x, y):
            self.grid[y, x] = 0

    def clear(self) -> None:
        """Reset all cells to free space."""
        self.grid.fill(0)

    def to_numpy(self) -> np.ndarray:
        """Return a copy of the underlying 2D NumPy array."""
        return self.grid.copy()

    @classmethod
    def from_numpy(cls, array: np.ndarray, resolution: float = 1.0) -> "GridMap":
        """Instantiate a GridMap from a 2D NumPy array."""
        if array.ndim != 2:
            raise ValueError(f"Array must be 2D, got shape {array.shape}")
        h, w = array.shape
        grid_map = cls(width=w, height=h, resolution=resolution)
        grid_map.grid = (array > 0).astype(np.uint8)
        return grid_map

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert continuous world coordinates (meters) to discrete grid indices."""
        gx = int(math.floor(wx / self.resolution))
        gy = int(math.floor(wy / self.resolution))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert discrete grid indices to continuous world coordinates (cell center)."""
        wx = (gx + 0.5) * self.resolution
        wy = (gy + 0.5) * self.resolution
        return wx, wy
