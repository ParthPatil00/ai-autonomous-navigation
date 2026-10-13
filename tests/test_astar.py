"""Unit tests for A* path planning algorithm."""

from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.planning.astar import AStarPlanner


def test_astar_straight_line():
    grid = GridMap(width=10, height=10)
    planner = AStarPlanner(heuristic="euclidean", allow_diagonal=False)
    res = planner.plan(grid, (1, 1), (5, 1))

    assert res.path is not None
    assert len(res.path) == 5
    assert res.path[0] == (1, 1)
    assert res.path[-1] == (5, 1)
    assert res.cost == 4.0


def test_astar_diagonal_movement():
    grid = GridMap(width=10, height=10)
    planner = AStarPlanner(heuristic="euclidean", allow_diagonal=True)
    res = planner.plan(grid, (0, 0), (2, 2))

    assert res.path is not None
    # Diagonal path should be 3 steps: (0,0), (1,1), (2,2)
    assert len(res.path) == 3
    assert res.path == [(0, 0), (1, 1), (2, 2)]


def test_astar_obstacle_avoidance():
    grid = GridMap(width=10, height=10)
    # Place vertical wall at x=3, y from 0 to 7
    for y in range(8):
        grid.set_obstacle(3, y)

    planner = AStarPlanner(heuristic="manhattan", allow_diagonal=False)
    res = planner.plan(grid, (1, 3), (5, 3))

    assert res.path is not None
    assert res.path[0] == (1, 3)
    assert res.path[-1] == (5, 3)
    # Ensure path did not cross any wall cell
    for pt in res.path:
        assert not grid.is_obstacle(*pt)


def test_astar_blocked_path():
    grid = GridMap(width=10, height=10)
    # Completely seal x=5 from y=0 to y=9
    for y in range(10):
        grid.set_obstacle(5, y)

    planner = AStarPlanner()
    res = planner.plan(grid, (1, 1), (8, 8))
    assert res.path is None
    assert res.cost == float("inf")


def test_astar_start_equals_goal():
    grid = GridMap(width=10, height=10)
    planner = AStarPlanner()
    res = planner.plan(grid, (4, 4), (4, 4))
    assert res.path == [(4, 4)]
    assert res.cost == 0.0


def test_astar_start_equals_goal_blocked():
    grid = GridMap(width=10, height=10)
    grid.set_obstacle(4, 4)
    planner = AStarPlanner()
    res = planner.plan(grid, (4, 4), (4, 4))
    assert res.path is None
    assert res.cost == float("inf")


def test_astar_invalid_coordinates():
    grid = GridMap(width=10, height=10)
    planner = AStarPlanner()

    # Negative coordinates
    assert planner.plan(grid, (-1, 0), (5, 5)).path is None
    # Out of bounds coordinates
    assert planner.plan(grid, (0, 0), (10, 5)).path is None
    # Float / non-integer coordinates
    assert planner.plan(grid, (1.5, 2.0), (5, 5)).path is None
    # Malformed tuple
    assert planner.plan(grid, (1,), (5, 5)).path is None
    assert planner.plan(grid, "invalid", (5, 5)).path is None


def test_astar_blocked_start_or_goal():
    grid = GridMap(width=10, height=10)
    grid.set_obstacle(1, 1)
    grid.set_obstacle(8, 8)
    planner = AStarPlanner()

    # Start blocked
    assert planner.plan(grid, (1, 1), (5, 5)).path is None
    # Goal blocked
    assert planner.plan(grid, (2, 2), (8, 8)).path is None
    # Both blocked
    assert planner.plan(grid, (1, 1), (8, 8)).path is None


def test_astar_unreachable_goal_enclosed():
    grid = GridMap(width=10, height=10)
    # Surround goal (5, 5) with a 3x3 obstacle ring
    for x in range(4, 7):
        for y in range(4, 7):
            if (x, y) != (5, 5):
                grid.set_obstacle(x, y)

    planner = AStarPlanner()
    res = planner.plan(grid, (0, 0), (5, 5))
    assert res.path is None
    assert res.cost == float("inf")


def test_astar_diagonal_corner_cutting_prevention():
    grid = GridMap(width=5, height=5)
    # Place obstacles at (1, 2) and (2, 1) - touching corners at (2, 2)
    grid.set_obstacle(1, 2)
    grid.set_obstacle(2, 1)

    # Robot wants to go from (1, 1) to (2, 2)
    planner = AStarPlanner(heuristic="octile", allow_diagonal=True)
    res = planner.plan(grid, (1, 1), (2, 2))

    # Diagonal move (1,1) -> (2,2) MUST NOT occur because (1,2) and (2,1) are obstacles
    assert res.path is not None
    # Must take a detour around the corner
    assert (1, 1) in res.path
    assert (2, 2) in res.path
    for i in range(len(res.path) - 1):
        step_from = res.path[i]
        step_to = res.path[i + 1]
        # Should not be direct diagonal through the blocked diagonal pinch point
        assert not (step_from == (1, 1) and step_to == (2, 2))


def test_astar_octile_heuristic():
    grid = GridMap(width=10, height=10)
    planner = AStarPlanner(heuristic="octile", allow_diagonal=True)
    res = planner.plan(grid, (0, 0), (3, 4))
    assert res.path is not None
    # 3 diagonal steps (3 * sqrt(2)) + 1 straight step (1 * 1.0) = ~5.2426
    import math
    expected_cost = 3 * math.sqrt(2.0) + 1.0
    assert math.isclose(res.cost, expected_cost, rel_tol=1e-5)


def test_astar_manhattan_diagonal_warning():
    import pytest
    with pytest.warns(UserWarning, match="Manhattan heuristic is inadmissible"):
        AStarPlanner(heuristic="manhattan", allow_diagonal=True)
