"""Collision validation and safety geometry checking.

Coordinate Conventions
----------------------
All positions (x, y) are in **world metres**.  A grid cell (gx, gy) occupies
the square area  [gx * res, (gx+1) * res) x [gy * res, (gy+1) * res)  where
``res = grid_map.resolution``.  The world-coordinate boundary of the map is
[0, width * res] x [0, height * res].

robot_radius is in **world metres** (same as CollisionChecker's x, y arguments).

The simulator sets robot world positions as:
    start_x = (cell_x + 0.5) * resolution   (cell centre in metres)

Any caller must convert from grid indices to world metres before calling
``is_state_valid`` or ``is_path_segment_valid``.  The planners operate in grid
indices and pass grid-cell centres expressed in metres to these methods after
the Navigator converts them with ``grid_to_world``.
"""

import math
from typing import List, Tuple

from autonomous_navigation.environment.grid_map import GridMap


def distance_segment_to_point(
    p1: Tuple[float, float],
    p2: Tuple[float, float],
    pt: Tuple[float, float],
) -> float:
    """Calculate the shortest Euclidean distance from a 2D point to a line segment.

    All coordinates are in world metres.

    Args:
        p1: Segment start (world metres).
        p2: Segment end (world metres).
        pt: Query point (world metres).

    Returns:
        Minimum Euclidean distance from pt to segment [p1, p2] in world metres.
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-12:
        return math.hypot(pt[0] - p1[0], pt[1] - p1[1])

    t = ((pt[0] - p1[0]) * dx + (pt[1] - p1[1]) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    proj_x = p1[0] + t * dx
    proj_y = p1[1] + t * dy
    return math.hypot(pt[0] - proj_x, pt[1] - proj_y)


class CollisionChecker:
    """Verifies robot geometry safety against environment occupancy grids.

    All positions and distances are in **world metres**.  The grid map's
    ``resolution`` attribute is used to convert between grid-cell indices and
    world coordinates internally.
    """

    def __init__(self, robot_radius: float = 0.35) -> None:
        """Initialise CollisionChecker.

        Args:
            robot_radius: Robot footprint bounding radius in **world metres**.
                          Must be positive.  The default (0.35 m) matches the
                          default resolution-1.0 configuration.
        """
        self.robot_radius = float(robot_radius)

    def is_state_valid(self, x: float, y: float, grid_map: GridMap) -> bool:
        """Check whether placing a circular robot footprint at world position (x, y)
        is collision-free.

        Args:
            x: Robot centre X in **world metres**.
            y: Robot centre Y in **world metres**.
            grid_map: Occupancy grid map (provides resolution and cell data).

        Returns:
            True if position is collision-free and fully within map boundaries,
            False otherwise.
        """
        res = grid_map.resolution
        map_w_m = grid_map.width * res   # map width in metres
        map_h_m = grid_map.height * res  # map height in metres

        # --- Boundary envelope check (world metres) ---
        if (
            x - self.robot_radius < 0.0
            or x + self.robot_radius > map_w_m
            or y - self.robot_radius < 0.0
            or y + self.robot_radius > map_h_m
        ):
            return False

        # --- Candidate grid cells to inspect ---
        # Convert robot footprint AABB from world metres to grid indices.
        # Subtract a small epsilon before floor so that a robot edge exactly
        # touching a cell boundary (e.g. x - radius == n * res) still scans
        # the cell whose far edge coincides with that boundary.
        _eps = 1e-9
        min_gx = max(0, int(math.floor((x - self.robot_radius - _eps) / res)))
        max_gx = min(grid_map.width - 1, int(math.ceil((x + self.robot_radius + _eps) / res)))
        min_gy = max(0, int(math.floor((y - self.robot_radius - _eps) / res)))
        max_gy = min(grid_map.height - 1, int(math.ceil((y + self.robot_radius + _eps) / res)))

        for gx in range(min_gx, max_gx + 1):
            for gy in range(min_gy, max_gy + 1):
                if grid_map.is_obstacle(gx, gy):
                    # Obstacle cell occupies world area [gx*res, (gx+1)*res] x [gy*res, (gy+1)*res]
                    box_x0 = float(gx) * res
                    box_x1 = float(gx + 1) * res
                    box_y0 = float(gy) * res
                    box_y1 = float(gy + 1) * res
                    # Closest point on obstacle square to robot centre
                    closest_x = max(box_x0, min(x, box_x1))
                    closest_y = max(box_y0, min(y, box_y1))
                    dist = math.hypot(x - closest_x, y - closest_y)
                    if dist <= self.robot_radius:
                        return False

        return True

    def is_path_segment_valid(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        grid_map: GridMap,
        step_size: float = 0.2,
    ) -> bool:
        """Check whether the continuous swept volume between two world positions is
        collision-free.

        Combines analytical obstacle corner checks with fine-grained sub-stepping
        to ensure no gaps exist in the swept capsule footprint.

        Args:
            p1: Segment start in world metres.
            p2: Segment end in world metres.
            grid_map: Occupancy grid map.
            step_size: Sub-step size in world metres (default 0.2 m).
        """
        # 1. Endpoint validity
        if not self.is_state_valid(p1[0], p1[1], grid_map):
            return False
        if not self.is_state_valid(p2[0], p2[1], grid_map):
            return False

        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if dist < 1e-6:
            return True

        res = grid_map.resolution
        map_w_m = grid_map.width * res
        map_h_m = grid_map.height * res

        # 2. Segment bounding box clearance check against obstacle cells
        min_x = min(p1[0], p2[0]) - self.robot_radius
        max_x = max(p1[0], p2[0]) + self.robot_radius
        min_y = min(p1[1], p2[1]) - self.robot_radius
        max_y = max(p1[1], p2[1]) + self.robot_radius

        if min_x < 0.0 or max_x > map_w_m or min_y < 0.0 or max_y > map_h_m:
            return False

        _eps = 1e-9
        min_gx = max(0, int(math.floor((min_x - _eps) / res)))
        max_gx = min(grid_map.width - 1, int(math.ceil((max_x + _eps) / res)))
        min_gy = max(0, int(math.floor((min_y - _eps) / res)))
        max_gy = min(grid_map.height - 1, int(math.ceil((max_y + _eps) / res)))

        for gx in range(min_gx, max_gx + 1):
            for gy in range(min_gy, max_gy + 1):
                if grid_map.is_obstacle(gx, gy):
                    # Check all 4 corners of obstacle cell (world metres)
                    box_x0 = float(gx) * res
                    box_x1 = float(gx + 1) * res
                    box_y0 = float(gy) * res
                    box_y1 = float(gy + 1) * res
                    corners = [
                        (box_x0, box_y0),
                        (box_x1, box_y0),
                        (box_x0, box_y1),
                        (box_x1, box_y1),
                    ]
                    for cx, cy in corners:
                        if distance_segment_to_point(p1, p2, (cx, cy)) <= self.robot_radius:
                            return False

        # 3. Dense sub-stepping to guarantee intermediate body coverage
        effective_step = min(step_size, max(0.02, self.robot_radius / 3.0))
        steps = max(1, int(math.ceil(dist / effective_step)))
        for i in range(1, steps):
            alpha = i / steps
            interp_x = p1[0] + alpha * (p2[0] - p1[0])
            interp_y = p1[1] + alpha * (p2[1] - p1[1])
            if not self.is_state_valid(interp_x, interp_y, grid_map):
                return False

        return True

    def is_path_valid(
        self,
        path: List[Tuple[float, float]],
        grid_map: GridMap,
    ) -> bool:
        """Check whether an entire waypoint path (world metres) is collision free."""
        if not path:
            return False

        if len(path) == 1:
            return self.is_state_valid(path[0][0], path[0][1], grid_map)

        for i in range(len(path) - 1):
            if not self.is_path_segment_valid(path[i], path[i + 1], grid_map):
                return False

        return True
