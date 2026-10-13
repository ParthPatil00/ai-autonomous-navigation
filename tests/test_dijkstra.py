"""Unit tests for Dijkstra path planning algorithm."""

from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.planning.dijkstra import DijkstraPlanner


def test_dijkstra_straight_line():
    grid = GridMap(width=10, height=10)
    planner = DijkstraPlanner(allow_diagonal=False)
    res = planner.plan(grid, (2, 2), (6, 2))

    assert res.path is not None
    assert len(res.path) == 5
    assert res.cost == 4.0
    assert res.path[0] == (2, 2)
    assert res.path[-1] == (6, 2)


def test_dijkstra_obstacle_detour():
    grid = GridMap(width=10, height=10)
    # Wall from (4, 1) to (4, 8)
    for y in range(1, 9):
        grid.set_obstacle(4, y)

    planner = DijkstraPlanner(allow_diagonal=True)
    res = planner.plan(grid, (2, 4), (7, 4))

    assert res.path is not None
    assert res.path[0] == (2, 4)
    assert res.path[-1] == (7, 4)
    for pt in res.path:
        assert not grid.is_obstacle(*pt)


def test_dijkstra_no_path():
    grid = GridMap(width=10, height=10)
    for y in range(10):
        grid.set_obstacle(3, y)

    planner = DijkstraPlanner()
    res = planner.plan(grid, (1, 1), (5, 5))
    assert res.path is None
    assert res.cost == float("inf")


def test_dijkstra_start_equals_goal():
    grid = GridMap(width=10, height=10)
    planner = DijkstraPlanner()
    res = planner.plan(grid, (3, 3), (3, 3))
    assert res.path == [(3, 3)]
    assert res.cost == 0.0


def test_dijkstra_start_equals_goal_blocked():
    grid = GridMap(width=10, height=10)
    grid.set_obstacle(3, 3)
    planner = DijkstraPlanner()
    res = planner.plan(grid, (3, 3), (3, 3))
    assert res.path is None
    assert res.cost == float("inf")


def test_dijkstra_invalid_coordinates():
    grid = GridMap(width=10, height=10)
    planner = DijkstraPlanner()
    assert planner.plan(grid, (-2, 0), (5, 5)).path is None
    assert planner.plan(grid, (0, 0), (10, 10)).path is None
    assert planner.plan(grid, (1.2, 3.4), (5, 5)).path is None
    assert planner.plan(grid, None, (5, 5)).path is None


def test_dijkstra_vs_astar_cost_equivalence_8conn():
    """Verify Dijkstra and A* (Euclidean and Octile) find equivalent optimal path costs on 8-conn grids."""
    import math
    from autonomous_navigation.environment.obstacle_generator import ObstacleGenerator
    from autonomous_navigation.planning.astar import AStarPlanner

    for seed in [42, 101, 2024]:
        grid = GridMap(width=25, height=25)
        ObstacleGenerator.add_boundary_walls(grid)
        ObstacleGenerator.add_random_obstacles(grid, density=0.15, seed=seed, safe_positions=[(3, 3), (21, 21)])

        dijkstra = DijkstraPlanner(allow_diagonal=True)
        astar_euc = AStarPlanner(heuristic="euclidean", allow_diagonal=True)
        astar_oct = AStarPlanner(heuristic="octile", allow_diagonal=True)

        res_dijk = dijkstra.plan(grid, (3, 3), (21, 21))
        res_euc = astar_euc.plan(grid, (3, 3), (21, 21))
        res_oct = astar_oct.plan(grid, (3, 3), (21, 21))

        if res_dijk.path is not None:
            assert res_euc.path is not None
            assert res_oct.path is not None
            assert math.isclose(res_euc.cost, res_dijk.cost, rel_tol=1e-5)
            assert math.isclose(res_oct.cost, res_dijk.cost, rel_tol=1e-5)
            # A* explores fewer or equal nodes compared to Dijkstra
            assert res_euc.nodes_explored <= res_dijk.nodes_explored
            assert res_oct.nodes_explored <= res_dijk.nodes_explored


def test_dijkstra_vs_astar_cost_equivalence_4conn():
    """Verify Dijkstra and A* (Manhattan) find equivalent optimal path costs on 4-conn grids."""
    import math
    from autonomous_navigation.environment.obstacle_generator import ObstacleGenerator
    from autonomous_navigation.planning.astar import AStarPlanner

    for seed in [12, 55, 99]:
        grid = GridMap(width=20, height=20)
        ObstacleGenerator.add_boundary_walls(grid)
        ObstacleGenerator.add_random_obstacles(grid, density=0.10, seed=seed, safe_positions=[(2, 2), (17, 17)])

        dijkstra = DijkstraPlanner(allow_diagonal=False)
        astar = AStarPlanner(heuristic="manhattan", allow_diagonal=False)

        res_dijk = dijkstra.plan(grid, (2, 2), (17, 17))
        res_astar = astar.plan(grid, (2, 2), (17, 17))

        if res_dijk.path is not None:
            assert res_astar.path is not None
            assert math.isclose(res_astar.cost, res_dijk.cost, rel_tol=1e-5)
            assert res_astar.nodes_explored <= res_dijk.nodes_explored
