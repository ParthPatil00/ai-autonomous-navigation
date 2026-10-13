"""2D Costmap layer with distance transform and configurable obstacle inflation.

Cost Layer Semantics:
---------------------
1. Physical Obstacle Occupancy (LETHAL_OBSTACLE = 254):
   Cells that contain physical obstacles/walls directly from GridMap.
   These are strictly impassable.

2. Robot-Footprint Clearance (INSCRIBED_INFLATED = 253):
   Free cells where the distance to the nearest physical obstacle is
   less than or equal to the robot's physical radius + safety_margin.
   Placing the robot center here causes geometric collision. Thus,
   these cells are impassable for path planning.

3. Soft Traversal Penalties (1 <= cost <= max_non_lethal_cost):
   Traversable cells within the inflation radius (robot_radius < dist < inflation_radius).
   Traversal penalties decay smoothly with increasing clearance from obstacles,
   encouraging global planners to select centered, safe routes.

4. Free Space (FREE_SPACE = 0):
   Traversable cells with clearance >= inflation_radius. Zero extra penalty.
"""

from collections import deque
from dataclasses import dataclass
import math
from typing import Optional, Tuple
import numpy as np


from autonomous_navigation.environment.grid_map import GridMap


@dataclass
class CostmapConfig:
    """Hyperparameters for costmap inflation and clearance cost generation."""
    robot_radius: float = 0.35          # Inscribed footprint radius in meters
    safety_margin: float = 0.0          # Additional hard forbidden buffer beyond robot radius
    inflation_radius: float = 1.5       # Outer radius where soft decay penalties drop to zero
    cost_scaling_factor: float = 3.0    # Exponential decay rate parameter
    max_non_lethal_cost: int = 199      # Maximum soft traversal penalty (1..199)
    decay_model: str = "exponential"    # "exponential" or "linear"

    def __post_init__(self) -> None:
        """Validate costmap configuration parameters."""
        if self.robot_radius < 0.0:
            raise ValueError(f"robot_radius must be non-negative, got {self.robot_radius}")
        if self.safety_margin < 0.0:
            raise ValueError(f"safety_margin must be non-negative, got {self.safety_margin}")
        if self.inflation_radius < 0.0:
            raise ValueError(f"inflation_radius must be non-negative, got {self.inflation_radius}")
        if self.cost_scaling_factor <= 0.0:
            raise ValueError(f"cost_scaling_factor must be positive, got {self.cost_scaling_factor}")
        if not (1 <= self.max_non_lethal_cost <= 252):
            raise ValueError(
                f"max_non_lethal_cost must be between 1 and 252, got {self.max_non_lethal_cost}"
            )
        if self.decay_model.lower() not in ("exponential", "linear"):
            raise ValueError(
                f"decay_model must be 'exponential' or 'linear', got {self.decay_model}"
            )


class Costmap:
    """Layered 2D costmap maintaining clearance and inflation costs."""

    LETHAL_OBSTACLE = 254
    INSCRIBED_INFLATED = 253
    FREE_SPACE = 0
    NO_INFORMATION = 255

    def __init__(
        self,
        grid_map: GridMap,
        config: Optional[CostmapConfig] = None,
    ) -> None:
        """Initialize Costmap.

        Args:
            grid_map: Reference to underlying binary GridMap.
            config: Optional CostmapConfig instance.
        """
        self.grid_map = grid_map
        self.config = config or CostmapConfig()
        self.width = grid_map.width
        self.height = grid_map.height
        self.resolution = grid_map.resolution

        self.clearance_map = np.zeros((self.height, self.width), dtype=np.float32)
        self.cost = np.zeros((self.height, self.width), dtype=np.uint8)

        self.update()

    def update(self) -> None:
        """Recompute Euclidean distance transform and cost layers from grid_map."""
        obstacle_mask = (self.grid_map.grid == 1)
        has_obstacles = bool(np.any(obstacle_mask))

        # 1. Compute exact distance from cell centers to square obstacle areas (in physical meters)
        if not has_obstacles:
            d_obs = np.full((self.height, self.width), np.inf, dtype=np.float32)
        else:
            d_obs = self._compute_obstacle_clearance(obstacle_mask)

        # 2. Compute clearance to map boundaries (in physical meters)
        gy, gx = np.indices((self.height, self.width))
        cx = (gx + 0.5) * self.resolution
        cy = (gy + 0.5) * self.resolution
        map_w_m = self.width * self.resolution
        map_h_m = self.height * self.resolution

        d_boundary = np.minimum(
            np.minimum(cx, map_w_m - cx),
            np.minimum(cy, map_h_m - cy),
        ).astype(np.float32)

        # 3. Overall clearance is the minimum of clearance to obstacles and map boundaries
        self.clearance_map = np.minimum(d_obs, d_boundary).astype(np.float32)

        # 4. Compute cost values based on clearance and inflation parameters
        r_inscribed = self.config.robot_radius + self.config.safety_margin
        r_inflation = max(r_inscribed + 1e-4, self.config.inflation_radius)

        costs = np.zeros((self.height, self.width), dtype=np.uint8)

        # Soft inflation zone: r_inscribed < clearance < r_inflation
        inflation_mask = (self.clearance_map > r_inscribed) & (self.clearance_map < r_inflation)
        if np.any(inflation_mask):
            norm_dist = (self.clearance_map[inflation_mask] - r_inscribed) / (r_inflation - r_inscribed)
            if self.config.decay_model.lower() == "linear":
                factors = 1.0 - norm_dist
            else:  # exponential
                factors = np.exp(-self.config.cost_scaling_factor * norm_dist)

            soft_costs = 1 + np.round((self.config.max_non_lethal_cost - 1) * factors)
            costs[inflation_mask] = np.clip(soft_costs, 1, self.config.max_non_lethal_cost).astype(np.uint8)

        # Inscribed inflated zone: clearance <= r_inscribed (forbidden for robot center)
        inscribed_mask = (self.clearance_map <= r_inscribed) & (~obstacle_mask)
        costs[inscribed_mask] = self.INSCRIBED_INFLATED

        # Physical lethal obstacles
        costs[obstacle_mask] = self.LETHAL_OBSTACLE

        self.cost = costs

    def _compute_obstacle_clearance(self, obstacle_mask: np.ndarray) -> np.ndarray:
        """Compute exact shortest Euclidean distance from each cell center to square obstacle cells."""
        h, w = self.height, self.width
        obstacle_indices = np.argwhere(obstacle_mask)
        if len(obstacle_indices) == 0:
            return np.full((h, w), np.inf, dtype=np.float32)

        gy, gx = np.indices((h, w))
        pts_x = ((gx + 0.5) * self.resolution).ravel()
        pts_y = ((gy + 0.5) * self.resolution).ravel()

        obs_x = obstacle_indices[:, 1] * self.resolution
        obs_y = obstacle_indices[:, 0] * self.resolution

        min_dists = np.full(pts_x.shape[0], np.inf, dtype=np.float32)
        chunk_size = 500
        for i in range(0, len(obs_x), chunk_size):
            ch_ox = obs_x[i : i + chunk_size]
            ch_oy = obs_y[i : i + chunk_size]
            clamped_x = np.clip(pts_x[:, None], ch_ox[None, :], ch_ox[None, :] + self.resolution)
            clamped_y = np.clip(pts_y[:, None], ch_oy[None, :], ch_oy[None, :] + self.resolution)
            dx = pts_x[:, None] - clamped_x
            dy = pts_y[:, None] - clamped_y
            dists = np.sqrt(dx * dx + dy * dy)
            min_dists = np.minimum(min_dists, np.min(dists, axis=1))

        return min_dists.reshape((h, w)).astype(np.float32)

    def _compute_distance_transform_fallback(self, obstacle_mask: np.ndarray) -> None:
        """NumPy fallback interface maintaining backward-compatible method signature."""
        self.clearance_map = self._compute_obstacle_clearance(obstacle_mask)

    def is_valid(self, x: int, y: int) -> bool:
        """Check whether cell coordinate is within costmap bounds."""
        return self.grid_map.is_valid(x, y)

    def is_physical_obstacle(self, x: int, y: int) -> bool:
        """Check whether cell is occupied by a physical wall/obstacle."""
        return self.grid_map.is_obstacle(x, y)

    def is_lethal(self, x: int, y: int) -> bool:
        """Check whether cell is impassable (physical obstacle or inscribed collision zone)."""
        if not self.is_valid(x, y):
            return True
        return bool(self.cost[y, x] >= self.INSCRIBED_INFLATED)

    def is_traversable(self, x: int, y: int) -> bool:
        """Check whether cell is safely traversable by the robot center."""
        if not self.is_valid(x, y):
            return False
        return bool(self.cost[y, x] < self.INSCRIBED_INFLATED)

    def get_cost(self, x: int, y: int) -> int:
        """Retrieve cost value (0..254) for cell (x, y)."""
        if not self.is_valid(x, y):
            return self.NO_INFORMATION
        return int(self.cost[y, x])

    def get_clearance(self, x: int, y: int) -> float:
        """Retrieve Euclidean clearance distance to nearest obstacle or boundary in world metres."""
        if not self.is_valid(x, y):
            return 0.0
        return float(self.clearance_map[y, x])

    def get_traversal_penalty(self, x: int, y: int, weight: float = 1.0) -> float:
        """Calculate soft traversal penalty for graph edge calculations.

        Args:
            x: Cell X coordinate.
            y: Cell Y coordinate.
            weight: Cost scaling weight.

        Returns:
            Non-negative float traversal penalty.
        """
        if not self.is_traversable(x, y):
            return float("inf")
        cell_cost = self.get_cost(x, y)
        return (cell_cost / 254.0) * float(weight)

    def to_numpy(self) -> np.ndarray:
        """Return a copy of the underlying 2D cost array."""
        return self.cost.copy()
