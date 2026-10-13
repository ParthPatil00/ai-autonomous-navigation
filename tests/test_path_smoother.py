"""Unit and regression tests for collision-safe PathSmoother."""

import math
import pytest

from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.navigation.collision_checker import CollisionChecker
from autonomous_navigation.planning.astar import AStarPlanner
from autonomous_navigation.planning.dijkstra import DijkstraPlanner
from autonomous_navigation.planning.path_smoother import PathSmoother, SmoothingResult


class TestPathSmoother:
    """Comprehensive test suite for PathSmoother."""

    def test_step_size_validation(self):
        """Reject non-positive, NaN, infinite, or non-numeric step sizes with ValueError."""
        # Valid default step size works
        smoother = PathSmoother()
        assert smoother.step_size == 0.1

        # Valid custom step size works
        smoother_custom = PathSmoother(step_size=0.05)
        assert smoother_custom.step_size == 0.05

        # Rejections
        invalid_values = [0, 0.0, -0.1, -1.0, float("nan"), float("inf"), float("-inf"), "invalid"]
        for val in invalid_values:
            with pytest.raises(ValueError, match="step_size"):
                PathSmoother(step_size=val)

    def test_empty_and_short_paths(self):
        """Empty, 1-point, and 2-point paths return gracefully without modification."""
        grid = GridMap(10, 10, resolution=1.0)
        smoother = PathSmoother()

        # Empty path
        res_empty = smoother.smooth_path([], grid)
        assert res_empty.smoothed_path == []
        assert not res_empty.is_smoothed
        assert not res_empty.is_path_valid
        assert res_empty.original_length == 0.0
        assert res_empty.smoothed_length == 0.0

        # Valid 1-point path
        p1 = [(1.5, 1.5)]
        res_p1 = smoother.smooth_path(p1, grid)
        assert res_p1.smoothed_path == p1
        assert not res_p1.is_smoothed
        assert res_p1.is_path_valid

        # Valid 2-point path (already minimal, preserves endpoints and length)
        p2 = [(1.5, 1.5), (4.5, 4.5)]
        res_p2 = smoother.smooth_path(p2, grid)
        assert res_p2.smoothed_path == p2
        assert not res_p2.is_smoothed
        assert res_p2.is_path_valid
        assert res_p2.original_length == res_p2.smoothed_length

    def test_single_point_outside_valid_map_bounds(self):
        """Single point outside map bounds or in collision retains original path as fallback with is_path_valid=False."""
        grid = GridMap(10, 10, resolution=1.0)
        smoother = PathSmoother()

        # Point outside map boundaries (negative coordinates)
        out_point_neg = [(-1.0, 5.0)]
        res_neg = smoother.smooth_path(out_point_neg, grid)
        assert res_neg.smoothed_path == out_point_neg
        assert not res_neg.is_smoothed
        assert not res_neg.is_path_valid

        # Point outside map boundaries (exceeds width*res)
        out_point_pos = [(12.0, 5.0)]
        res_pos = smoother.smooth_path(out_point_pos, grid)
        assert res_pos.smoothed_path == out_point_pos
        assert not res_pos.is_smoothed
        assert not res_pos.is_path_valid

        # Point inside an obstacle cell
        grid.set_obstacle(3, 3)
        obs_point = [(3.5, 3.5)]
        res_obs = smoother.smooth_path(obs_point, grid)
        assert res_obs.smoothed_path == obs_point
        assert not res_obs.is_smoothed
        assert not res_obs.is_path_valid

    def test_two_point_path_intersecting_obstacle(self):
        """Two-point path intersecting an obstacle retains original path without claiming safety (is_path_valid=False)."""
        grid = GridMap(10, 10, resolution=1.0)
        # Place obstacle at (3, 3)
        grid.set_obstacle(3, 3)
        # Two-point path directly crossing obstacle (3, 3)
        intersecting_p2 = [(1.5, 3.5), (5.5, 3.5)]

        cc = CollisionChecker(robot_radius=0.35)
        # Confirm segment is genuinely invalid
        assert not cc.is_path_valid(intersecting_p2, grid)

        smoother = PathSmoother(collision_checker=cc)
        res = smoother.smooth_path(intersecting_p2, grid)

        # Fallback retains original path and explicitly marks invalid
        assert res.smoothed_path == intersecting_p2
        assert not res.is_smoothed
        assert not res.is_path_valid
        assert res.original_waypoint_count == 2
        assert res.smoothed_waypoint_count == 2

    def test_valid_two_point_path(self):
        """Valid 2-point path in free space returns endpoints untouched with is_path_valid=True."""
        grid = GridMap(10, 10, resolution=1.0)
        valid_p2 = [(1.5, 1.5), (2.5, 2.5)]
        cc = CollisionChecker(robot_radius=0.35)
        assert cc.is_path_valid(valid_p2, grid)

        smoother = PathSmoother(collision_checker=cc)
        res = smoother.smooth_path(valid_p2, grid)

        assert res.smoothed_path == valid_p2
        assert not res.is_smoothed
        assert res.is_path_valid
        assert res.original_path == valid_p2
        assert math.isclose(res.smoothed_length, math.hypot(1.0, 1.0), rel_tol=1e-5)

    def test_straight_path_preserves_endpoints_and_length(self):
        """A collinear path with redundant intermediate points is simplified to start and end."""
        grid = GridMap(10, 10, resolution=1.0)
        smoother = PathSmoother()

        # Collinear path along y = 2.5 from x = 1.5 to 5.5 (5 waypoints)
        straight_path = [(1.5, 2.5), (2.5, 2.5), (3.5, 2.5), (4.5, 2.5), (5.5, 2.5)]
        res = smoother.smooth_path(straight_path, grid)

        assert res.is_smoothed
        assert res.smoothed_path == [(1.5, 2.5), (5.5, 2.5)]
        assert res.smoothed_waypoint_count == 2
        assert math.isclose(res.smoothed_length, 4.0, rel_tol=1e-5)
        assert math.isclose(res.original_length, 4.0, rel_tol=1e-5)

    def test_path_with_unnecessary_turns_shortened(self):
        """A path with staircase or detour turns in open space is safely shortcut."""
        grid = GridMap(10, 10, resolution=1.0)
        # Empty space: robot can shortcut a right-angle detour (0,0) -> (0,4) -> (4,4)
        detour_path = [
            (1.5, 1.5),
            (1.5, 2.5),
            (1.5, 3.5),
            (1.5, 4.5),
            (2.5, 4.5),
            (3.5, 4.5),
            (4.5, 4.5),
        ]
        # Original L-shaped path length: 3 + 3 = 6.0m
        smoother = PathSmoother(collision_checker=CollisionChecker(robot_radius=0.2))
        res = smoother.smooth_path(detour_path, grid)

        assert res.is_smoothed
        # Direct line from (1.5, 1.5) to (4.5, 4.5) length is sqrt(3^2 + 3^2) ~ 4.2426m
        expected_len = math.hypot(3.0, 3.0)
        assert math.isclose(res.smoothed_length, expected_len, rel_tol=1e-4)
        assert res.smoothed_length < res.original_length
        assert res.smoothed_waypoint_count == 2
        assert res.length_reduction_ratio > 0.25

    def test_path_near_obstacle_respects_clearance(self):
        """Shortcut that would clip an obstacle or violate robot radius is rejected."""
        grid = GridMap(10, 10, resolution=1.0)
        # Place obstacle at (3, 3) spanning [3.0, 4.0] x [3.0, 4.0]
        grid.set_obstacle(3, 3)

        # Path goes around the obstacle: (1.5, 1.5) -> (1.5, 4.5) -> (4.5, 4.5)
        # Direct shortcut from (1.5, 1.5) to (4.5, 4.5) passes directly through (3.0, 3.0) obstacle
        path_around = [
            (1.5, 1.5),
            (1.5, 2.5),
            (1.5, 5.5),
            (3.5, 5.5),
            (5.5, 5.5),
        ]
        cc = CollisionChecker(robot_radius=0.35)
        smoother = PathSmoother(collision_checker=cc)
        res = smoother.smooth_path(path_around, grid)

        # Verify that all segments in the smoothed path are collision free
        assert cc.is_path_valid(res.smoothed_path, grid)
        # Direct shortcut (1.5, 1.5) to (5.5, 5.5) must NOT be taken because it clips (3,3)
        assert (1.5, 1.5) == res.smoothed_path[0]
        assert (5.5, 5.5) == res.smoothed_path[-1]
        assert len(res.smoothed_path) >= 3

    def test_narrow_corridor_retains_safety(self):
        """In a narrow 1-cell corridor between walls, smoothing does not cut into wall cells."""
        grid = GridMap(10, 10, resolution=1.0)
        for x in range(1, 8):
            grid.set_obstacle(x, 2)
            grid.set_obstacle(x, 4)

        # Corridor runs along y = 3.5 from x = 1.5 to 6.5
        corridor_path = [
            (1.5, 3.5),
            (2.5, 3.5),
            (3.5, 3.5),
            (4.5, 3.5),
            (5.5, 3.5),
            (6.5, 3.5),
        ]
        cc = CollisionChecker(robot_radius=0.3)
        smoother = PathSmoother(collision_checker=cc)
        res = smoother.smooth_path(corridor_path, grid)

        assert cc.is_path_valid(res.smoothed_path, grid)
        assert res.smoothed_path[0] == (1.5, 3.5)
        assert res.smoothed_path[-1] == (6.5, 3.5)
        # The straight corridor line is safe
        assert math.isclose(res.smoothed_length, res.original_length, rel_tol=1e-5)

    @pytest.mark.parametrize("res", [1.0, 0.5, 0.25])
    def test_smoothing_across_multiple_resolutions(self, res):
        """Smoother handles coordinates and boundary checks consistently across resolutions."""
        grid = GridMap(10, 10, resolution=res)
        # Staircase path in free space
        path = [
            ((1 + 0.5) * res, (1 + 0.5) * res),
            ((1 + 0.5) * res, (3 + 0.5) * res),
            ((3 + 0.5) * res, (3 + 0.5) * res),
        ]
        cc = CollisionChecker(robot_radius=0.2 * res)
        smoother = PathSmoother(collision_checker=cc)
        smooth_res = smoother.smooth_path(path, grid)

        assert cc.is_path_valid(smooth_res.smoothed_path, grid)
        assert smooth_res.smoothed_length < smooth_res.original_length
        assert smooth_res.smoothed_path[0] == path[0]
        assert smooth_res.smoothed_path[-1] == path[-1]

    def test_smoothing_rejection_falls_back_to_original(self):
        """If a path is already un-shortcuttable or shortcut is unsafe, original path is preserved."""
        grid = GridMap(10, 10, resolution=1.0)
        # Obstacle between points
        grid.set_obstacle(2, 2)
        u_shaped_path = [
            (1.5, 1.5),
            (1.5, 3.5),
            (3.5, 3.5),
            (3.5, 1.5),
        ]
        cc = CollisionChecker(robot_radius=0.4)
        smoother = PathSmoother(collision_checker=cc)
        res = smoother.smooth_path(u_shaped_path, grid)

        assert cc.is_path_valid(res.smoothed_path, grid)
        # Cannot connect (1.5, 1.5) directly to (3.5, 1.5) because obstacle at (2,2) blocks it
        assert res.smoothed_path[0] == (1.5, 1.5)
        assert res.smoothed_path[-1] == (3.5, 1.5)

    def test_navigator_integration_with_smoothing(self):
        """Navigator successfully integrates PathSmoother and retains original_path."""
        from autonomous_navigation.navigation.navigator import Navigator

        grid = GridMap(20, 20, resolution=1.0)
        grid.set_obstacle(5, 5)
        planner = AStarPlanner()
        cc = CollisionChecker(robot_radius=0.35)

        navigator = Navigator(planner=planner, collision_checker=cc)
        # Plan with smooth_path=True
        ok = navigator.plan_to_goal(grid, (2, 2), (8, 8), smooth_path=True)

        assert ok
        assert navigator.last_smoothing_result is not None
        assert navigator.last_smoothing_result.is_path_valid
        assert len(navigator.original_path) > len(navigator.current_path)
        # Verify smoothed current_path is completely collision-free
        assert cc.is_path_valid(navigator.current_path, grid)
        assert navigator.is_goal_reached(navigator.current_path[-1])

    def test_successfully_smoothed_path_is_reported_valid(self):
        """A successfully smoothed path sets is_smoothed=True and is_path_valid=True."""
        grid = GridMap(10, 10, resolution=1.0)
        # Staircase path in free space
        staircase = [(1.5, 1.5), (1.5, 3.5), (3.5, 3.5)]
        cc = CollisionChecker(robot_radius=0.2)
        smoother = PathSmoother(collision_checker=cc)

        res = smoother.smooth_path(staircase, grid)
        assert res.is_smoothed
        assert res.is_path_valid
        assert cc.is_path_valid(res.smoothed_path, grid)
        assert res.smoothed_path == [(1.5, 1.5), (3.5, 3.5)]

    def test_failed_final_smoothing_validation_falls_back_to_safe_original(self):
        """If shortcutting somehow produces an invalid segment, it falls back to original path."""
        grid = GridMap(10, 10, resolution=1.0)
        # 3-point valid path
        path = [(1.5, 1.5), (2.5, 1.5), (3.5, 1.5)]
        cc = CollisionChecker(robot_radius=0.2)
        smoother = PathSmoother(collision_checker=cc)

        # Mock segment valid returning True during shortcutting, but is_path_valid returning False for smoothed
        original_is_path_valid = cc.is_path_valid
        call_count = [0]

        def selective_path_valid(p, g):
            call_count[0] += 1
            if call_count[0] == 1:
                # Initial validation of original input path
                return original_is_path_valid(p, g)
            # Final validation of shortcut path fails
            return False

        cc.is_path_valid = selective_path_valid

        res = smoother.smooth_path(path, grid)
        # Must fall back strictly to original path, is_smoothed=False, is_path_valid=True (original path is safe)
        assert not res.is_smoothed
        assert res.smoothed_path == path
        assert res.is_path_valid
        assert original_is_path_valid(res.smoothed_path, grid)

    def test_smoothing_result_dataclass_preserves_invalid_flag(self):
        """Direct construction of SmoothingResult preserves explicit is_path_valid=False."""
        result = SmoothingResult(
            smoothed_path=[(-1.0, -1.0)],
            original_path=[(-1.0, -1.0)],
            is_smoothed=False,
            is_path_valid=False,
            original_length=0.0,
            smoothed_length=0.0,
            original_waypoint_count=1,
            smoothed_waypoint_count=1,
            length_reduction_ratio=0.0,
        )
        assert not result.is_path_valid
        assert not result.is_smoothed

    def test_short_path_metrics_and_rounding(self):
        """Ensure metrics remain accurate and non-misleading for short paths."""
        grid = GridMap(10, 10, resolution=1.0)
        smoother = PathSmoother()

        # Very short 2-point path: 1mm (0.001m)
        micro_path = [(1.500, 1.500), (1.501, 1.500)]
        res_micro = smoother.smooth_path(micro_path, grid)
        assert res_micro.is_path_valid
        assert not res_micro.is_smoothed
        assert math.isclose(res_micro.original_length, 0.001, rel_tol=1e-5)
        assert math.isclose(res_micro.smoothed_length, 0.001, rel_tol=1e-5)
        assert res_micro.length_reduction_ratio == 0.0

        # Sub-millimeter path: length < 1e-4 preserves endpoints and reports valid
        sub_micro = [(1.50000, 1.50000), (1.50005, 1.50000)]
        res_sub = smoother.smooth_path(sub_micro, grid)
        assert res_sub.is_path_valid
        assert not res_sub.is_smoothed
        assert res_sub.smoothed_path == sub_micro
        assert math.isclose(res_sub.smoothed_length, 0.00005, rel_tol=1e-4)
