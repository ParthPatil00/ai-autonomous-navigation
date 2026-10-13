"""Unit tests for Navigator."""

from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.navigation.navigator import Navigator
from autonomous_navigation.planning.astar import AStarPlanner


def test_navigator_plan_and_waypoint_tracking():
    grid = GridMap(width=20, height=20)
    planner = AStarPlanner(heuristic="manhattan", allow_diagonal=False)
    nav = Navigator(planner=planner, waypoint_tolerance=0.3)

    success = nav.plan_to_goal(grid, (2, 2), (5, 2))
    assert success
    assert len(nav.current_path) == 4  # (2, 2), (3, 2), (4, 2), (5, 2)

    # First waypoint should be cell center (2.5, 2.5)
    assert nav.get_current_waypoint() == (2.5, 2.5)

    # Move close to (2.5, 2.5)
    nav.update_waypoint_progress((2.55, 2.5))
    # Should have advanced to (3.5, 2.5)
    assert nav.get_current_waypoint() == (3.5, 2.5)


def test_navigator_goal_reached():
    grid = GridMap(width=20, height=20)
    planner = AStarPlanner()
    nav = Navigator(planner=planner, waypoint_tolerance=0.5)

    nav.plan_to_goal(grid, (1, 1), (10, 10))
    assert not nav.is_goal_reached((1.5, 1.5))
    # Goal center is (10.5, 10.5)
    assert nav.is_goal_reached((10.4, 10.5))


def test_navigator_replan_when_path_blocked():
    grid = GridMap(width=20, height=20)
    planner = AStarPlanner(heuristic="euclidean", allow_diagonal=True)
    nav = Navigator(planner=planner)

    assert nav.plan_to_goal(grid, (2, 2), (10, 2))
    assert not nav.is_path_blocked(grid)

    # Dynamically inject obstacle blocking cell (6, 2)
    grid.set_obstacle(6, 2)
    assert nav.is_path_blocked(grid)

    # Replan from current robot position (e.g. at 2.5, 2.5)
    replan_success = nav.replan(grid, (2.5, 2.5))
    assert replan_success
    assert not nav.is_path_blocked(grid)
    # New path must avoid cell (6, 2)
    for wp in nav.current_path:
        cell_x = int(wp[0])
        cell_y = int(wp[1])
        assert not grid.is_obstacle(cell_x, cell_y)


def test_navigator_replan_fails_when_unreachable():
    grid = GridMap(width=10, height=10)
    planner = AStarPlanner()
    nav = Navigator(planner=planner)

    assert nav.plan_to_goal(grid, (2, 2), (8, 8))
    # Block goal completely
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            grid.set_obstacle(8 + dx, 8 + dy)

    replan_success = nav.replan(grid, (2.5, 2.5))
    assert not replan_success


def test_robot_motion_collision_rejection():
    """Verify that candidate motion towards obstacle is rejected leaving robot safe."""
    from autonomous_navigation.navigation.collision_checker import CollisionChecker
    from autonomous_navigation.robot.robot_model import RobotModel

    grid = GridMap(width=10, height=10)
    grid.set_obstacle(5, 5)
    robot = RobotModel(start_x=4.0, start_y=5.5, radius=0.35)
    checker = CollisionChecker(robot_radius=0.35)

    # Moving directly towards center of obstacle (5.5, 5.5)
    cand_x, cand_y, cand_theta = robot.compute_next_position(5.5, 5.5, dt=1.0)
    assert not checker.is_path_segment_valid(robot.position, (cand_x, cand_y), grid)

    # Candidate motion is rejected; robot remains at (4.0, 5.5)
    assert robot.position == (4.0, 5.5)
    assert checker.is_state_valid(robot.position[0], robot.position[1], grid)


def test_navigator_accepts_valid_smoothed_path():
    """Valid smoothed path is accepted as active navigation path."""
    from autonomous_navigation.navigation.collision_checker import CollisionChecker

    grid = GridMap(width=20, height=20)
    planner = AStarPlanner()
    cc = CollisionChecker(robot_radius=0.3)
    nav = Navigator(planner=planner, collision_checker=cc)

    ok = nav.plan_to_goal(grid, (2, 2), (8, 8), smooth_path=True)
    assert ok
    assert nav.last_smoothing_result is not None
    assert nav.last_smoothing_result.is_path_valid
    assert len(nav.current_path) > 0
    assert cc.is_path_valid(nav.current_path, grid)


def test_navigator_accepts_valid_unsmoothed_path():
    """Valid unsmoothed path remains accepted when smooth_path=False."""
    grid = GridMap(width=20, height=20)
    planner = AStarPlanner()
    nav = Navigator(planner=planner)

    ok = nav.plan_to_goal(grid, (2, 2), (5, 5), smooth_path=False)
    assert ok
    assert nav.last_smoothing_result is None
    assert len(nav.current_path) > 0


def test_navigator_rejects_invalid_smoothing_result():
    """If smoothing result is invalid (is_path_valid=False), navigator rejects it."""
    from unittest.mock import MagicMock
    from autonomous_navigation.navigation.collision_checker import CollisionChecker
    from autonomous_navigation.planning.path_smoother import SmoothingResult

    grid = GridMap(width=10, height=10)
    planner = AStarPlanner()
    cc = CollisionChecker(robot_radius=0.3)
    nav = Navigator(planner=planner, collision_checker=cc)

    # First plan normally to verify baseline works
    assert nav.plan_to_goal(grid, (1, 1), (5, 1), smooth_path=False)
    assert len(nav.current_path) > 0

    # Mock smoother to simulate an invalid smoothing result
    mock_invalid_result = SmoothingResult(
        smoothed_path=[(-1.0, -1.0), (100.0, 100.0)],
        original_path=[(1.5, 1.5), (5.5, 1.5)],
        is_smoothed=False,
        is_path_valid=False,
        original_length=4.0,
        smoothed_length=200.0,
        original_waypoint_count=2,
        smoothed_waypoint_count=2,
        length_reduction_ratio=0.0,
    )

    from unittest.mock import patch
    with patch("autonomous_navigation.planning.path_smoother.PathSmoother.smooth_path", return_value=mock_invalid_result):
        ok = nav.plan_to_goal(grid, (1, 1), (5, 1), smooth_path=True)
        # Must fail and clear active navigation paths
        assert not ok
        assert nav.current_path == []
        assert nav.original_path == []
        assert nav.last_smoothing_result is mock_invalid_result
        assert not nav.last_smoothing_result.is_path_valid


def test_navigator_is_path_blocked_detects_invalid_single_point_remaining_path():
    """An obstacle appearing at the final remaining single-point waypoint is detected as blocked."""
    grid = GridMap(width=10, height=10)
    planner = AStarPlanner(heuristic="manhattan", allow_diagonal=False)
    nav = Navigator(planner=planner, waypoint_tolerance=0.2)

    # Plan from (2, 2) to (4, 2) -> waypoints: (2.5, 2.5), (3.5, 2.5), (4.5, 2.5)
    assert nav.plan_to_goal(grid, (2, 2), (4, 2))
    assert not nav.is_path_blocked(grid)

    # Advance to the last waypoint index
    nav.waypoint_index = len(nav.current_path) - 1
    # Single remaining waypoint: (4.5, 2.5)
    assert len(nav.current_path[nav.waypoint_index:]) == 1
    assert not nav.is_path_blocked(grid)

    # Now inject an obstacle at the final cell (4, 2)
    grid.set_obstacle(4, 2)
    # Navigator must detect that the single remaining point is now in collision
    assert nav.is_path_blocked(grid)
