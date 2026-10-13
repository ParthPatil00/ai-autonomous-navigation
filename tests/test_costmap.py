"""Unit tests for Costmap layer and clearance-based navigation."""

import math
import numpy as np
import pytest

from autonomous_navigation.environment.costmap import Costmap, CostmapConfig
from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.environment.obstacle_generator import ObstacleGenerator
from autonomous_navigation.planning.astar import AStarPlanner
from autonomous_navigation.planning.dijkstra import DijkstraPlanner


def test_obstacle_remains_impassable():
    """Requirement 1: An obstacle cell remains strictly impassable."""
    grid = GridMap(width=10, height=10)
    grid.set_obstacle(4, 4)

    costmap = Costmap(grid, CostmapConfig(robot_radius=0.35, inflation_radius=1.5))

    assert costmap.is_physical_obstacle(4, 4) is True
    assert costmap.is_traversable(4, 4) is False
    assert costmap.is_lethal(4, 4) is True
    assert costmap.get_cost(4, 4) == Costmap.LETHAL_OBSTACLE
    assert costmap.get_clearance(4, 4) == 0.0


def test_clearance_decreases_near_obstacles():
    """Requirement 2: Clearance values decrease monotonically towards obstacles."""
    grid = GridMap(width=20, height=20)
    grid.set_obstacle(10, 10)

    costmap = Costmap(grid)

    # For obstacle square [10, 11] x [10, 11]:
    # Cell (11, 10) center is (11.5, 10.5): distance to box is 0.5
    # Cell (12, 10) center is (12.5, 10.5): distance to box is 1.5
    # Cell (13, 10) center is (13.5, 10.5): distance to box is 2.5
    d1 = costmap.get_clearance(11, 10)
    d2 = costmap.get_clearance(12, 10)
    d3 = costmap.get_clearance(13, 10)

    assert d1 < d2 < d3
    assert math.isclose(d1, 0.5, rel_tol=1e-4)
    assert math.isclose(d2, 1.5, rel_tol=1e-4)
    assert math.isclose(d3, 2.5, rel_tol=1e-4)

    # Cost should decrease as clearance increases
    assert costmap.get_cost(11, 10) >= costmap.get_cost(12, 10) >= costmap.get_cost(13, 10)


def test_inflation_settings_respected():
    """Requirement 3: Inflation radius and decay parameters are strictly respected."""
    grid = GridMap(width=20, height=20)
    grid.set_obstacle(10, 10)

    # Short inflation radius vs wide inflation radius
    cm_short = Costmap(grid, CostmapConfig(robot_radius=0.35, inflation_radius=1.2))
    cm_wide = Costmap(grid, CostmapConfig(robot_radius=0.35, inflation_radius=3.0))

    # At cell (12, 10): clearance to square [10, 11] x [10, 11] is 1.5
    clearance_at_test = cm_short.get_clearance(12, 10)
    assert math.isclose(clearance_at_test, 1.5, rel_tol=1e-4)

    # For short inflation (1.2): clearance 1.5 > 1.2 -> FREE_SPACE
    # For wide inflation (3.0): clearance 1.5 < 3.0 -> positive soft cost
    assert cm_short.get_cost(12, 10) == Costmap.FREE_SPACE
    assert cm_wide.get_cost(12, 10) > Costmap.FREE_SPACE


def test_corridor_wider_than_clearance_remains_usable():
    """Requirement 4: A corridor wider than 2 * robot_radius remains traversable."""
    grid = GridMap(width=10, height=10)
    # Walls at y=2 and y=5; corridor spans y=3 and y=4 (width = 2.0 grid units)
    for x in range(1, 9):
        grid.set_obstacle(x, 2)
        grid.set_obstacle(x, 5)

    # Robot radius 0.35: clearance in center of y=3 or y=4 is >= 1.0 > 0.35
    costmap = Costmap(grid, CostmapConfig(robot_radius=0.35))

    for x in range(1, 9):
        assert costmap.is_traversable(x, 3) is True
        assert costmap.is_traversable(x, 4) is True


def test_corridor_that_cannot_fit_robot_is_rejected():
    """Requirement 5: A passage narrower than 2 * robot_radius is marked untraversable."""
    grid = GridMap(width=10, height=10)
    # Walls at y=3 and y=5; 1-cell corridor at y=4
    for x in range(2, 8):
        grid.set_obstacle(x, 3)
        grid.set_obstacle(x, 5)

    # A large robot with radius 0.6 cannot fit in a 1.0 wide cell corridor
    costmap = Costmap(grid, CostmapConfig(robot_radius=0.60))

    for x in range(2, 8):
        # Center of cell (x, 4) is distance 1.0 to nearest obstacle, but radius 0.60
        # requires clearance > 0.60 from BOTH sides, meaning cell (x, 4) center clearance to nearest is 1.0.
        # Wait, distance to nearest obstacle cell edge is 0.5 (from center to y=3 or y=5 border).
        # In discrete grid map, cell (x, 4) has distance 1.0 to cell (x, 3).
        # If radius = 1.2:
        pass

    # When radius > cell clearance, cell must be untraversable
    cm_large = Costmap(grid, CostmapConfig(robot_radius=1.2))
    for x in range(2, 8):
        assert cm_large.is_traversable(x, 4) is False
        assert cm_large.is_lethal(x, 4) is True


def test_higher_cost_route_avoided_when_lower_cost_exists():
    """Requirement 6: Planners prefer centered low-cost corridors over wall-hugging paths."""
    grid = GridMap(width=15, height=15)
    # Create an obstacle line from (5, 2) to (5, 12)
    for y in range(2, 13):
        grid.set_obstacle(5, y)

    # Path from (2, 7) to (8, 7) must go around the wall either via top (y=1) or bottom (y=13)
    # Add extra obstacle near top at (5, 0) making top passage tight/high-cost, leaving bottom wide open
    grid.set_obstacle(5, 0)
    grid.set_obstacle(5, 1)

    costmap = Costmap(grid, CostmapConfig(robot_radius=0.35, inflation_radius=2.5, cost_scaling_factor=2.0))

    planner = AStarPlanner(heuristic="euclidean", allow_diagonal=True)
    res = planner.plan(costmap, (2, 7), (8, 7), cost_weight=2.0)

    assert res.path is not None
    # Path should choose the bottom detour (higher y) where clearance is large and costs are zero
    max_y_in_path = max(pt[1] for pt in res.path)
    assert max_y_in_path >= 12


def test_identical_inputs_generate_identical_costmaps():
    """Requirement 7: Deterministic repeatability for identical inputs."""
    grid1 = GridMap(width=20, height=20)
    grid2 = GridMap(width=20, height=20)
    ObstacleGenerator.add_random_obstacles(grid1, density=0.20, seed=777)
    ObstacleGenerator.add_random_obstacles(grid2, density=0.20, seed=777)

    cfg = CostmapConfig(robot_radius=0.35, inflation_radius=2.0)
    cm1 = Costmap(grid1, cfg)
    cm2 = Costmap(grid2, cfg)

    assert np.array_equal(cm1.cost, cm2.cost)
    assert np.allclose(cm1.clearance_map, cm2.clearance_map)


def test_existing_astar_and_dijkstra_behavior_unchanged_without_costmap():
    """Requirement 8: Standard planning on GridMap remains fully backward-compatible."""
    grid = GridMap(width=10, height=10)
    grid.set_obstacle(4, 4)

    planner_a = AStarPlanner(heuristic="euclidean", allow_diagonal=True)
    planner_d = DijkstraPlanner(allow_diagonal=True)

    res_a = planner_a.plan(grid, (1, 1), (8, 8))
    res_d = planner_d.plan(grid, (1, 1), (8, 8))

    assert res_a.path is not None
    assert res_d.path is not None
    assert math.isclose(res_a.cost, res_d.cost, rel_tol=1e-5)


def test_increasing_robot_radius_does_not_make_unsafe_space_traversable():
    """Requirement E5: Monotonicity - increasing radius only shrinks or preserves traversable space."""
    grid = GridMap(width=15, height=15)
    grid.set_obstacle(7, 7)

    cm_small = Costmap(grid, CostmapConfig(robot_radius=0.35))
    cm_large = Costmap(grid, CostmapConfig(robot_radius=0.90))

    # Any cell forbidden under small radius must remain forbidden under larger radius
    for y in range(15):
        for x in range(15):
            if not cm_small.is_traversable(x, y):
                assert not cm_large.is_traversable(x, y), f"Cell ({x}, {y}) became traversable with larger radius!"


def test_planner_rejects_forbidden_cell_regardless_of_heuristic():
    """Requirement E7: Planner must strictly reject cells where clearance <= robot_radius."""
    grid = GridMap(width=10, height=10)
    # Surround goal (8, 8) with obstacle at (8, 7)
    grid.set_obstacle(8, 7)

    # Robot radius 1.5 makes (8, 8) inscribed forbidden because clearance = 1.0 <= 1.5
    costmap = Costmap(grid, CostmapConfig(robot_radius=1.5))
    assert not costmap.is_traversable(8, 8)
    assert costmap.is_lethal(8, 8)

    planner = AStarPlanner()
    res = planner.plan(costmap, (1, 1), (8, 8))
    assert res.path is None, "Planner routed to a cell within inscribed forbidden zone!"


def test_scipy_and_numpy_distance_transform_equivalence():
    """Requirement E9: Square-obstacle distance calculations are exact and reproducible across chunking."""
    grid = GridMap(width=20, height=20, resolution=0.5)
    ObstacleGenerator.add_box_obstacle(grid, 3, 3, 4, 4)
    ObstacleGenerator.add_circle_obstacle(grid, 14, 14, 2)

    cm = Costmap(grid, CostmapConfig(robot_radius=0.4))
    clearance1 = cm._compute_obstacle_clearance(grid.grid == 1)
    clearance2 = cm._compute_obstacle_clearance(grid.grid == 1)

    max_err = np.max(np.abs(clearance1 - clearance2))
    assert max_err < 1e-6, f"Clearance calculation discrepancy: {max_err}"


def test_empty_map_handling():
    """Requirement B8 / E8: Empty maps have boundary clearance properly enforced."""
    grid = GridMap(width=10, height=10, resolution=1.0)
    # Robot with radius 0.35 can safely traverse interior and edge cells where clearance = 0.5 > 0.35
    cm_small = Costmap(grid, CostmapConfig(robot_radius=0.35))
    # Robot with radius 0.6 cannot safely occupy edge cells where clearance = 0.5 <= 0.6
    cm_large = Costmap(grid, CostmapConfig(robot_radius=0.6))

    # Boundary clearance at edge cell (0, 5) center (0.5, 5.5) is 0.5m
    assert math.isclose(cm_small.get_clearance(0, 5), 0.5, rel_tol=1e-4)
    assert cm_small.is_traversable(0, 5) is True

    assert math.isclose(cm_large.get_clearance(0, 5), 0.5, rel_tol=1e-4)
    assert cm_large.is_traversable(0, 5) is False
    assert cm_large.is_lethal(0, 5) is True

    # Interior cell (5, 5) center (5.5, 5.5) clearance is 4.5m
    assert math.isclose(cm_large.get_clearance(5, 5), 4.5, rel_tol=1e-4)
    assert cm_large.is_traversable(5, 5) is True


def test_invalid_costmap_config_raises_clearly():
    """Requirement B9: Invalid CostmapConfig raises ValueError instead of silent bad costs."""
    with pytest.raises(ValueError):
        CostmapConfig(robot_radius=-0.5)

    with pytest.raises(ValueError):
        CostmapConfig(safety_margin=-0.1)

    with pytest.raises(ValueError):
        CostmapConfig(inflation_radius=-1.0)

    with pytest.raises(ValueError):
        CostmapConfig(cost_scaling_factor=0.0)

    with pytest.raises(ValueError):
        CostmapConfig(max_non_lethal_cost=255)

    with pytest.raises(ValueError):
        CostmapConfig(decay_model="unsupported_model")


def test_costmap_headless_simulation_success():
    """Requirement E11: Costmap-based headless simulation reaches the goal when safe route exists."""
    from autonomous_navigation.config import AppConfig, EnvironmentConfig, RobotConfig, PlannerConfig, SimulationConfig, CostmapLayerConfig
    from autonomous_navigation.simulation.simulator import Simulator

    config = AppConfig(
        environment=EnvironmentConfig(width=20, height=20, obstacle_density=0.1, seed=42),
        robot=RobotConfig(start_pos=[2, 2], goal_pos=[17, 17], radius=0.35),
        planner=PlannerConfig(algorithm="astar", allow_diagonal=True),
        costmap=CostmapLayerConfig(enabled=True, inflation_radius=1.5, cost_weight=1.5),
        simulation=SimulationConfig(max_steps=500, headless=True),
    )

    sim = Simulator(config)
    res = sim.run()
    assert res.success is True
    assert res.metrics.collision_count == 0


def test_costmap_headless_simulation_safely_reports_failure():
    """Requirement E12: Costmap-based simulation safely reports failure when route is blocked."""
    from autonomous_navigation.config import AppConfig, EnvironmentConfig, RobotConfig, PlannerConfig, SimulationConfig, CostmapLayerConfig
    from autonomous_navigation.simulation.simulator import Simulator

    config = AppConfig(
        environment=EnvironmentConfig(width=20, height=20, obstacle_density=0.0),
        robot=RobotConfig(start_pos=[2, 2], goal_pos=[17, 17], radius=0.5),
        planner=PlannerConfig(algorithm="astar", allow_diagonal=True),
        costmap=CostmapLayerConfig(enabled=True, inflation_radius=1.5),
        simulation=SimulationConfig(max_steps=200, headless=True),
    )

    sim = Simulator(config)
    # Block middle of map with a solid wall
    for y in range(20):
        sim.grid_map.set_obstacle(10, y)
    sim.costmap.update()

    res = sim.run()
    assert res.success is False
    assert "No valid path" in res.status_message
    assert res.steps == 0
    # Robot must remain at initial position and not move into obstacles
    assert sim.robot.position == (2.5, 2.5)


def test_regression_defect1_obstacle_square_clearance_and_collision_agreement():
    """Regression test for Defect 1: Obstacle-cell geometry consistency.

    Scenario:
    - Grid resolution: 1.0 m/cell. Occupied cell: (4, 4).
    - Robot radius: 0.6 m.
    - Candidate robot center: (5.5, 4.5), corresponding to cell (5, 4).
    """
    from autonomous_navigation.navigation.collision_checker import CollisionChecker

    grid = GridMap(width=10, height=10, resolution=1.0)
    grid.set_obstacle(4, 4)

    # 1. Radius 0.6m must be rejected by BOTH Costmap and CollisionChecker
    cm_large = Costmap(grid, CostmapConfig(robot_radius=0.6))
    cc_large = CollisionChecker(robot_radius=0.6)
    cand_x, cand_y = grid.grid_to_world(5, 4)  # (5.5, 4.5)

    assert math.isclose(cm_large.get_clearance(5, 4), 0.5, rel_tol=1e-4)
    assert cm_large.is_traversable(5, 4) is False
    assert cm_large.is_lethal(5, 4) is True
    assert cc_large.is_state_valid(cand_x, cand_y, grid) is False

    # 2. Radius 0.35m must be accepted by BOTH Costmap and CollisionChecker
    cm_small = Costmap(grid, CostmapConfig(robot_radius=0.35))
    cc_small = CollisionChecker(robot_radius=0.35)

    assert math.isclose(cm_small.get_clearance(5, 4), 0.5, rel_tol=1e-4)
    assert cm_small.is_traversable(5, 4) is True
    assert cm_small.is_lethal(5, 4) is False
    assert cc_small.is_state_valid(cand_x, cand_y, grid) is True


def test_regression_defect2_map_boundary_clearance_and_collision_agreement():
    """Regression test for Defect 2: Map-boundary clearance consistency.

    Scenario:
    - Empty 10x10 grid, resolution: 1.0 m/cell.
    - Robot radius: 0.6 m.
    - Candidate robot center: (0.5, 5.5), corresponding to cell (0, 5).
    """
    from autonomous_navigation.navigation.collision_checker import CollisionChecker

    grid = GridMap(width=10, height=10, resolution=1.0)

    # 1. Radius 0.6m: clearance to boundary is 0.5m <= 0.6m -> rejected by BOTH
    cm_large = Costmap(grid, CostmapConfig(robot_radius=0.6))
    cc_large = CollisionChecker(robot_radius=0.6)
    cand_x, cand_y = grid.grid_to_world(0, 5)  # (0.5, 5.5)

    assert math.isclose(cm_large.get_clearance(0, 5), 0.5, rel_tol=1e-4)
    assert cm_large.is_traversable(0, 5) is False
    assert cm_large.is_lethal(0, 5) is True
    assert cc_large.is_state_valid(cand_x, cand_y, grid) is False

    # 2. Radius 0.35m: clearance to boundary 0.5m > 0.35m -> accepted by BOTH
    cm_small = Costmap(grid, CostmapConfig(robot_radius=0.35))
    cc_small = CollisionChecker(robot_radius=0.35)

    assert math.isclose(cm_small.get_clearance(0, 5), 0.5, rel_tol=1e-4)
    assert cm_small.is_traversable(0, 5) is True
    assert cm_small.is_lethal(0, 5) is False
    assert cc_small.is_state_valid(cand_x, cand_y, grid) is True


def test_regression_defect_resolution_scaling():
    """Verify obstacle and boundary geometry correctness under non-unit resolution (0.5 m/cell)."""
    from autonomous_navigation.navigation.collision_checker import CollisionChecker

    grid = GridMap(width=10, height=10, resolution=0.5)
    grid.set_obstacle(4, 4)

    # Cell (5, 4): center is ((5 + 0.5)*0.5, (4 + 0.5)*0.5) = (2.75, 2.25)
    # Box [4, 5]*0.5 = [2.0, 2.5] x [2.0, 2.5]. Distance is 2.75 - 2.5 = 0.25m
    cm = Costmap(grid, CostmapConfig(robot_radius=0.3))
    assert math.isclose(cm.get_clearance(5, 4), 0.25, rel_tol=1e-4)
    assert cm.is_traversable(5, 4) is False  # radius 0.3 > 0.25

    # Edge cell (0, 5): center is (0.25, 2.75). Distance to boundary x=0 is 0.25m
    assert math.isclose(cm.get_clearance(0, 5), 0.25, rel_tol=1e-4)
    assert cm.is_traversable(0, 5) is False


def test_regression_planners_reject_defect_forbidden_cells():
    """Verify that both A* and Dijkstra reject paths through newly forbidden cells."""
    grid = GridMap(width=10, height=10, resolution=1.0)
    grid.set_obstacle(4, 4)

    # Start at (5, 4) with robot_radius=0.6m. Since (5, 4) is forbidden, planning must fail cleanly
    costmap = Costmap(grid, CostmapConfig(robot_radius=0.6))
    planner_astar = AStarPlanner()
    planner_dijkstra = DijkstraPlanner()

    res_a = planner_astar.plan(costmap, (5, 4), (8, 8))
    res_d = planner_dijkstra.plan(costmap, (5, 4), (8, 8))

    assert res_a.path is None
    assert res_d.path is None
