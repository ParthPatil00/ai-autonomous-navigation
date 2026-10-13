"""Unit and consistency tests for world-metre coordinate convention.

Covers resolutions 1.0, 0.5 and 0.25 for:
  - Obstacle-adjacent states that must be rejected
  - States near each map boundary
  - Empty maps
  - Narrow corridors that cannot fit the robot
  - Corridors that can fit the robot
  - World-to-grid conversions
  - CollisionChecker and Costmap agreement
  - A* and Dijkstra using the corrected semantics
"""

import math

import pytest

from autonomous_navigation.environment.costmap import Costmap, CostmapConfig
from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.navigation.collision_checker import CollisionChecker
from autonomous_navigation.planning.astar import AStarPlanner
from autonomous_navigation.planning.dijkstra import DijkstraPlanner


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def cell_center_world(gx: int, gy: int, res: float):
    """Return world-metre centre of grid cell (gx, gy)."""
    return (gx + 0.5) * res, (gy + 0.5) * res


RESOLUTIONS = [1.0, 0.5, 0.25]


# ---------------------------------------------------------------------------
# 1. World-to-grid conversions
# ---------------------------------------------------------------------------

class TestWorldGridConversions:
    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_grid_to_world_cell_centre(self, res):
        grid = GridMap(10, 10, resolution=res)
        wx, wy = grid.grid_to_world(3, 7)
        assert math.isclose(wx, (3 + 0.5) * res, rel_tol=1e-6)
        assert math.isclose(wy, (7 + 0.5) * res, rel_tol=1e-6)

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_world_to_grid_cell_index(self, res):
        grid = GridMap(10, 10, resolution=res)
        # Centre of cell (3, 7) should map back to (3, 7)
        wx, wy = grid.grid_to_world(3, 7)
        gx, gy = grid.world_to_grid(wx, wy)
        assert gx == 3
        assert gy == 7

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_map_world_dimensions(self, res):
        """Map world extent must be width*res x height*res."""
        grid = GridMap(8, 6, resolution=res)
        assert math.isclose(grid.width * res, 8 * res)
        assert math.isclose(grid.height * res, 6 * res)

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_negative_coordinates_map_to_negative_indices(self, res):
        """Small negative world coordinates must floor to -1, never truncating to cell 0."""
        grid = GridMap(10, 10, resolution=res)
        # Small negative coordinates just outside origin
        assert grid.world_to_grid(-1e-6, 0.5 * res) == (-1, 0)
        assert grid.world_to_grid(0.5 * res, -1e-6) == (0, -1)
        assert grid.world_to_grid(-1e-6, -1e-6) == (-1, -1)
        assert grid.world_to_grid(-0.01 * res, -0.01 * res) == (-1, -1)
        assert grid.world_to_grid(-0.5 * res, 0.0) == (-1, 0)
        assert grid.world_to_grid(-res, 0.0) == (-1, 0)
        assert grid.world_to_grid(-1.0001 * res, 0.0) == (-2, 0)

        # Confirm they are recognized as out-of-bounds via grid_map API
        gx, gy = grid.world_to_grid(-1e-6, 0.0)
        assert not grid.is_valid(gx, gy)
        assert grid.is_obstacle(gx, gy)

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_origin_coordinates(self, res):
        """Coordinates at exact world origin (0.0, 0.0) map to cell (0, 0)."""
        grid = GridMap(10, 10, resolution=res)
        gx, gy = grid.world_to_grid(0.0, 0.0)
        assert gx == 0 and gy == 0
        assert grid.is_valid(gx, gy)
        assert not grid.is_obstacle(gx, gy)

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_just_inside_map_boundaries(self, res):
        """Coordinates just inside map boundaries map to valid extreme cells."""
        grid = GridMap(10, 10, resolution=res)
        max_x_m = 10 * res
        max_y_m = 10 * res

        # Near bottom-left (0, 0)
        assert grid.world_to_grid(1e-6, 1e-6) == (0, 0)

        # Just inside top-right (9, 9)
        gx, gy = grid.world_to_grid(max_x_m - 1e-6, max_y_m - 1e-6)
        assert gx == 9 and gy == 9
        assert grid.is_valid(gx, gy)

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_exactly_at_and_outside_maximum_extent(self, res):
        """Coordinates at or exceeding max extent (width*res, height*res) map to width/height (out of bounds)."""
        grid = GridMap(10, 10, resolution=res)
        max_x_m = 10 * res
        max_y_m = 10 * res

        # Exactly at max extent
        gx, gy = grid.world_to_grid(max_x_m, max_y_m)
        assert gx == 10 and gy == 10
        assert not grid.is_valid(gx, gy)
        assert grid.is_obstacle(gx, gy)

        # Just outside max extent
        gx_out, gy_out = grid.world_to_grid(max_x_m + 1e-6, max_y_m + 1e-6)
        assert gx_out == 10 and gy_out == 10
        assert not grid.is_valid(gx_out, gy_out)
        assert grid.is_obstacle(gx_out, gy_out)

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_round_trip_for_all_valid_cell_centers(self, res):
        """Round-trip conversion (grid -> world center -> grid) holds for all valid cells."""
        grid = GridMap(6, 6, resolution=res)
        for gy in range(6):
            for gx in range(6):
                wx, wy = grid.grid_to_world(gx, gy)
                conv_gx, conv_gy = grid.world_to_grid(wx, wy)
                assert (conv_gx, conv_gy) == (gx, gy)


# ---------------------------------------------------------------------------
# 2. Obstacle-adjacent states
# ---------------------------------------------------------------------------

class TestObstacleAdjacent:
    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_cell_adjacent_to_obstacle_clearance(self, res):
        """Cell immediately east of an obstacle has clearance == res/2 (half a cell width)."""
        grid = GridMap(10, 10, resolution=res)
        grid.set_obstacle(4, 4)
        cm = Costmap(grid, CostmapConfig(robot_radius=0.1 * res))

        # Cell (5,4) centre is at world (5.5*res, 4.5*res).
        # Obstacle box right edge is at 5*res. Distance = 5.5*res - 5*res = 0.5*res.
        expected_clearance = 0.5 * res
        actual_clearance = cm.get_clearance(5, 4)
        assert math.isclose(actual_clearance, expected_clearance, rel_tol=1e-4), (
            f"res={res}: expected clearance {expected_clearance}, got {actual_clearance}"
        )

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_adjacent_cell_rejected_when_radius_exceeds_clearance(self, res):
        """Robot with radius > 0.5*res cannot occupy cell adjacent to obstacle."""
        grid = GridMap(10, 10, resolution=res)
        grid.set_obstacle(4, 4)

        robot_radius = 0.6 * res  # radius > clearance (0.5*res) -> must be rejected
        cm = Costmap(grid, CostmapConfig(robot_radius=robot_radius))
        cc = CollisionChecker(robot_radius=robot_radius)

        wx, wy = cell_center_world(5, 4, res)
        assert cm.is_traversable(5, 4) is False, f"res={res}: Costmap should forbid cell (5,4)"
        assert cc.is_state_valid(wx, wy, grid) is False, f"res={res}: CC should reject (5,4) centre"

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_adjacent_cell_accepted_when_radius_below_clearance(self, res):
        """Robot with radius < 0.5*res can occupy cell adjacent to obstacle."""
        grid = GridMap(10, 10, resolution=res)
        grid.set_obstacle(4, 4)

        robot_radius = 0.35 * res  # radius < clearance (0.5*res) -> must be accepted
        cm = Costmap(grid, CostmapConfig(robot_radius=robot_radius))
        cc = CollisionChecker(robot_radius=robot_radius)

        wx, wy = cell_center_world(5, 4, res)
        assert cm.is_traversable(5, 4) is True, f"res={res}: Costmap should allow cell (5,4)"
        assert cc.is_state_valid(wx, wy, grid) is True, f"res={res}: CC should accept (5,4) centre"

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_costmap_cc_agree_on_adjacent_cell(self, res):
        """Costmap and CollisionChecker must give identical accept/reject for adjacent cell."""
        grid = GridMap(10, 10, resolution=res)
        grid.set_obstacle(4, 4)
        wx, wy = cell_center_world(5, 4, res)

        for robot_radius in [0.3 * res, 0.5 * res, 0.7 * res]:
            cm = Costmap(grid, CostmapConfig(robot_radius=robot_radius))
            cc = CollisionChecker(robot_radius=robot_radius)
            cm_ok = cm.is_traversable(5, 4)
            cc_ok = cc.is_state_valid(wx, wy, grid)
            assert cm_ok == cc_ok, (
                f"res={res} r={robot_radius:.4f}: Costmap={cm_ok} vs CC={cc_ok} disagree"
            )


# ---------------------------------------------------------------------------
# 3. Map-boundary states
# ---------------------------------------------------------------------------

class TestMapBoundaries:
    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_edge_cell_clearance_equals_half_res(self, res):
        """Each edge-cell centre is exactly 0.5*res from the nearest map boundary."""
        grid = GridMap(10, 10, resolution=res)
        cm = Costmap(grid, CostmapConfig(robot_radius=0.1 * res))

        edge_cells = [(0, 5), (9, 5), (5, 0), (5, 9)]
        for gx, gy in edge_cells:
            clr = cm.get_clearance(gx, gy)
            assert math.isclose(clr, 0.5 * res, rel_tol=1e-4), (
                f"res={res} cell ({gx},{gy}): expected clearance {0.5*res:.4f}, got {clr:.4f}"
            )

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_all_four_boundaries_agree_cc_and_costmap(self, res):
        """All four map edges must be rejected for robot_radius = 0.6*res."""
        grid = GridMap(10, 10, resolution=res)
        robot_radius = 0.6 * res  # > clearance of 0.5*res
        cm = Costmap(grid, CostmapConfig(robot_radius=robot_radius))
        cc = CollisionChecker(robot_radius=robot_radius)

        edge_cells = [(0, 5), (9, 5), (5, 0), (5, 9)]
        for gx, gy in edge_cells:
            wx, wy = cell_center_world(gx, gy, res)
            assert cm.is_traversable(gx, gy) is False, (
                f"res={res} Costmap should forbid edge ({gx},{gy})"
            )
            assert cc.is_state_valid(wx, wy, grid) is False, (
                f"res={res} CC should reject edge ({gx},{gy})"
            )

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_boundary_accepted_when_radius_below_clearance(self, res):
        """Edge cell must be accepted for robot_radius = 0.35*res."""
        grid = GridMap(10, 10, resolution=res)
        robot_radius = 0.35 * res  # < clearance of 0.5*res
        cm = Costmap(grid, CostmapConfig(robot_radius=robot_radius))
        cc = CollisionChecker(robot_radius=robot_radius)

        wx, wy = cell_center_world(0, 5, res)
        assert cm.is_traversable(0, 5) is True
        assert cc.is_state_valid(wx, wy, grid) is True


# ---------------------------------------------------------------------------
# 4. Empty maps
# ---------------------------------------------------------------------------

class TestEmptyMap:
    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_interior_always_traversable(self, res):
        """Interior cell of empty map must be traversable even for large radii."""
        grid = GridMap(20, 20, resolution=res)
        robot_radius = 3.0 * res  # large but interior has plenty of clearance
        cm = Costmap(grid, CostmapConfig(robot_radius=robot_radius))
        cc = CollisionChecker(robot_radius=robot_radius)

        # Cell (10,10) has clearance = 10.5*res (to nearest boundary)
        wx, wy = cell_center_world(10, 10, res)
        assert cm.is_traversable(10, 10) is True
        assert cc.is_state_valid(wx, wy, grid) is True

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_both_planners_find_path_on_empty_map(self, res):
        """A* and Dijkstra find valid paths on empty map at each resolution."""
        grid = GridMap(10, 10, resolution=res)
        robot_radius = 0.35 * res
        cm = Costmap(grid, CostmapConfig(robot_radius=robot_radius))
        cc = CollisionChecker(robot_radius=robot_radius)

        for PlannerCls in [AStarPlanner, DijkstraPlanner]:
            planner = PlannerCls()
            result = planner.plan(cm, (1, 1), (8, 8))
            assert result.path is not None, f"res={res}: {PlannerCls.__name__} found no path"
            # All waypoints must be collision-free
            for gx, gy in result.path:
                wx, wy = cell_center_world(gx, gy, res)
                assert cc.is_state_valid(wx, wy, grid), (
                    f"res={res}: Path cell ({gx},{gy}) world ({wx:.4f},{wy:.4f}) fails CC"
                )


# ---------------------------------------------------------------------------
# 5. Narrow corridors
# ---------------------------------------------------------------------------

class TestNarrowCorridors:
    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_corridor_too_narrow_rejected_by_costmap(self, res):
        """1-cell-wide corridor must be rejected when robot_radius > 0.5*res."""
        grid = GridMap(12, 12, resolution=res)
        for x in range(12):
            grid.set_obstacle(x, 4)
            grid.set_obstacle(x, 6)

        robot_radius = 0.6 * res  # clearance in corridor is 0.5*res < robot_radius
        cm = Costmap(grid, CostmapConfig(robot_radius=robot_radius))

        # All corridor cells at y=5 must be forbidden
        for x in range(12):
            assert cm.is_traversable(x, 5) is False, (
                f"res={res}: corridor cell ({x},5) should be forbidden"
            )

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_corridor_passable_for_smaller_robot(self, res):
        """1-cell-wide corridor must be traversable when robot_radius < 0.5*res."""
        grid = GridMap(12, 12, resolution=res)
        for x in range(1, 12):   # leave x=0 open so start/goal can reach corridor
            grid.set_obstacle(x, 4)
            grid.set_obstacle(x, 6)

        robot_radius = 0.35 * res  # clearance 0.5*res > robot_radius
        cm = Costmap(grid, CostmapConfig(robot_radius=robot_radius))
        cc = CollisionChecker(robot_radius=robot_radius)

        # Corridor cells at y=5, x>=1 should be traversable
        for x in range(1, 11):
            assert cm.is_traversable(x, 5) is True, (
                f"res={res}: corridor cell ({x},5) should be traversable"
            )
            wx, wy = cell_center_world(x, 5, res)
            assert cc.is_state_valid(wx, wy, grid) is True, (
                f"res={res}: CC should accept corridor ({x},5) centre"
            )

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_planners_cannot_route_through_forbidden_corridor(self, res):
        """A* and Dijkstra must return None when the only corridor is forbidden."""
        grid = GridMap(12, 12, resolution=res)
        for x in range(12):
            grid.set_obstacle(x, 4)
            grid.set_obstacle(x, 6)

        robot_radius = 0.6 * res
        cm = Costmap(grid, CostmapConfig(robot_radius=robot_radius))

        for PlannerCls in [AStarPlanner, DijkstraPlanner]:
            planner = PlannerCls()
            result = planner.plan(cm, (2, 2), (2, 9))
            assert result.path is None, (
                f"res={res}: {PlannerCls.__name__} should find no path through forbidden corridor"
            )

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_planners_route_through_passable_corridor(self, res):
        """A* and Dijkstra find collision-free paths through a passable corridor."""
        grid = GridMap(12, 12, resolution=res)
        for x in range(1, 12):
            grid.set_obstacle(x, 4)
            grid.set_obstacle(x, 6)

        robot_radius = 0.35 * res
        cm = Costmap(grid, CostmapConfig(robot_radius=robot_radius))
        cc = CollisionChecker(robot_radius=robot_radius)

        for PlannerCls in [AStarPlanner, DijkstraPlanner]:
            planner = PlannerCls()
            result = planner.plan(cm, (2, 2), (2, 9))
            assert result.path is not None, (
                f"res={res}: {PlannerCls.__name__} should find path through passable corridor"
            )
            for gx, gy in result.path:
                wx, wy = cell_center_world(gx, gy, res)
                assert cc.is_state_valid(wx, wy, grid), (
                    f"res={res}: Path cell ({gx},{gy}) world ({wx:.4f},{wy:.4f}) fails CC"
                )


# ---------------------------------------------------------------------------
# 6. Costmap and CollisionChecker physical agreement
# ---------------------------------------------------------------------------

class TestCostmapCCPhysicalAgreement:
    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_clearance_is_physical_metres(self, res):
        """Costmap clearance must equal the physical (metre) distance to the obstacle box edge."""
        grid = GridMap(10, 10, resolution=res)
        grid.set_obstacle(4, 4)
        cm = Costmap(grid, CostmapConfig(robot_radius=0.1 * res))

        # Cell (6, 4): centre = (6.5*res, 4.5*res). Obstacle box east edge at 5*res.
        # Horizontal distance = 6.5*res - 5*res = 1.5*res.
        expected = 1.5 * res
        actual = cm.get_clearance(6, 4)
        assert math.isclose(actual, expected, rel_tol=1e-4), (
            f"res={res}: expected clearance {expected:.6f} m, got {actual:.6f} m"
        )

    @pytest.mark.parametrize("res", RESOLUTIONS)
    def test_cc_and_costmap_agree_on_all_cells(self, res):
        """For a grid with one obstacle, CC and Costmap must agree on every free cell."""
        grid = GridMap(8, 8, resolution=res)
        grid.set_obstacle(4, 4)
        robot_radius = 0.45 * res  # chosen so some cells are forbidden
        cm = Costmap(grid, CostmapConfig(robot_radius=robot_radius))
        cc = CollisionChecker(robot_radius=robot_radius)

        for gy in range(8):
            for gx in range(8):
                if grid.is_obstacle(gx, gy):
                    continue
                wx, wy = cell_center_world(gx, gy, res)
                cm_ok = cm.is_traversable(gx, gy)
                cc_ok = cc.is_state_valid(wx, wy, grid)
                assert cm_ok == cc_ok, (
                    f"res={res} r={robot_radius:.4f}: ({gx},{gy}) "
                    f"Costmap={cm_ok} CC={cc_ok} disagree"
                )


# ---------------------------------------------------------------------------
# 7. Planner paths are collision-free (physical check)
# ---------------------------------------------------------------------------

class TestPlannerPathCollisionFree:
    @pytest.mark.parametrize("res", RESOLUTIONS)
    @pytest.mark.parametrize("planner_cls", [AStarPlanner, DijkstraPlanner])
    def test_planned_path_passes_cc(self, res, planner_cls):
        """Every cell-centre waypoint of a planned path must pass the CollisionChecker."""
        grid = GridMap(12, 12, resolution=res)
        grid.set_obstacle(6, 3)
        grid.set_obstacle(6, 4)
        grid.set_obstacle(6, 5)

        robot_radius = 0.35 * res
        cm = Costmap(grid, CostmapConfig(robot_radius=robot_radius))
        cc = CollisionChecker(robot_radius=robot_radius)

        planner = planner_cls()
        result = planner.plan(cm, (1, 1), (10, 10))
        assert result.path is not None, (
            f"res={res}: {planner_cls.__name__} found no path"
        )
        for gx, gy in result.path:
            wx, wy = cell_center_world(gx, gy, res)
            assert cc.is_state_valid(wx, wy, grid), (
                f"res={res}: path waypoint ({gx},{gy}) world ({wx:.4f},{wy:.4f}) fails CC"
            )
