"""Sensor perception simulation and obstacle detection."""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple

from autonomous_navigation.environment.grid_map import GridMap


@dataclass
class ScanReading:
    """Represents an individual range sensor ray result."""
    angle_rad: float
    distance: float
    hit_point: Optional[Tuple[float, float]]
    hit_obstacle: bool


class ObstacleDetector:
    """Simulates a 2D planar range sensor (LiDAR-like) on a discrete grid map."""

    def __init__(self, max_range: float = 6.0, num_rays: int = 36, step_size: float = 0.2) -> None:
        """Initialize ObstacleDetector.

        Args:
            max_range: Maximum sensor detection range in grid units.
            num_rays: Number of angular scan rays distributed over 360 degrees.
            step_size: Ray marching increment for obstacle intersection checking.
        """
        self.max_range = float(max_range)
        self.num_rays = int(num_rays)
        self.step_size = float(step_size)

    def scan(
        self,
        grid_map: GridMap,
        robot_x: float,
        robot_y: float,
        heading_rad: float = 0.0,
    ) -> List[ScanReading]:
        """Perform simulated planar raycast scan around robot location.

        Args:
            grid_map: Ground truth environment grid map.
            robot_x: Current robot X coordinate.
            robot_y: Current robot Y coordinate.
            heading_rad: Robot orientation angle in radians.

        Returns:
            List of ScanReading instances for all configured rays.
        """
        readings: List[ScanReading] = []
        angle_step = (2.0 * math.pi) / self.num_rays

        for i in range(self.num_rays):
            angle = heading_rad + i * angle_step
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)

            curr_dist = 0.0
            hit = False
            hit_pos: Optional[Tuple[float, float]] = None

            while curr_dist < self.max_range:
                curr_dist += self.step_size
                test_x = robot_x + curr_dist * cos_a
                test_y = robot_y + curr_dist * sin_a

                cell_x = int(math.floor(test_x))
                cell_y = int(math.floor(test_y))

                if not grid_map.is_valid(cell_x, cell_y) or grid_map.is_obstacle(cell_x, cell_y):
                    hit = True
                    hit_pos = (test_x, test_y)
                    break

            if not hit:
                # Max range reached with no obstacle hit
                end_x = robot_x + self.max_range * cos_a
                end_y = robot_y + self.max_range * sin_a
                readings.append(
                    ScanReading(
                        angle_rad=angle,
                        distance=self.max_range,
                        hit_point=(end_x, end_y),
                        hit_obstacle=False,
                    )
                )
            else:
                readings.append(
                    ScanReading(
                        angle_rad=angle,
                        distance=min(curr_dist, self.max_range),
                        hit_point=hit_pos,
                        hit_obstacle=True,
                    )
                )

        return readings

    def get_detected_cells(
        self,
        grid_map: GridMap,
        robot_x: float,
        robot_y: float,
        radius: Optional[float] = None,
    ) -> List[Tuple[int, int]]:
        """Return all occupied obstacle cells within a circular neighborhood.

        Args:
            grid_map: Environment map.
            robot_x: Robot center X.
            robot_y: Robot center Y.
            radius: Detection radius (defaults to sensor max_range).

        Returns:
            List of (x, y) occupied grid cells detected within radius.
        """
        r = radius if radius is not None else self.max_range
        detected: List[Tuple[int, int]] = []

        min_x = max(0, int(math.floor(robot_x - r)))
        max_x = min(grid_map.width - 1, int(math.ceil(robot_x + r)))
        min_y = max(0, int(math.floor(robot_y - r)))
        max_y = min(grid_map.height - 1, int(math.ceil(robot_y + r)))

        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                if math.hypot(x + 0.5 - robot_x, y + 0.5 - robot_y) <= r:
                    if grid_map.is_obstacle(x, y):
                        detected.append((x, y))

        return detected
