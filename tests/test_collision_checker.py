import math

from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.navigation.collision_checker import CollisionChecker


def test_collision_checker_free_space():
    grid = GridMap(width=20, height=20)
    checker = CollisionChecker(robot_radius=0.5)
    # Middle of empty space
    assert checker.is_state_valid(10.0, 10.0, grid)


def test_collision_checker_boundaries():
    grid = GridMap(width=20, height=20)
    checker = CollisionChecker(robot_radius=0.5)
    # Outside boundary limits
    assert not checker.is_state_valid(0.2, 10.0, grid)
    assert not checker.is_state_valid(19.8, 10.0, grid)
    assert not checker.is_state_valid(10.0, -0.1, grid)


def test_collision_checker_obstacle_proximity():
    grid = GridMap(width=20, height=20)
    grid.set_obstacle(5, 5)  # Occupies [5, 6] x [5, 6]
    checker = CollisionChecker(robot_radius=0.5)

    # Directly inside obstacle
    assert not checker.is_state_valid(5.5, 5.5, grid)

    # Immediately adjacent to obstacle within radius 0.5
    assert not checker.is_state_valid(4.8, 5.5, grid)

    # Far away from obstacle
    assert checker.is_state_valid(10.0, 10.0, grid)


def test_collision_checker_path_segment():
    grid = GridMap(width=20, height=20)
    grid.set_obstacle(5, 5)
    checker = CollisionChecker(robot_radius=0.4)

    # Segment passing through obstacle
    assert not checker.is_path_segment_valid((3.0, 5.5), (7.0, 5.5), grid)

    # Segment clear of obstacle
    assert checker.is_path_segment_valid((3.0, 1.0), (7.0, 1.0), grid)


def test_collision_checker_tunneling_prevention():
    """Verify that jumping across a 1-cell obstacle is caught by continuous path checking."""
    grid = GridMap(width=10, height=10)
    grid.set_obstacle(5, 5)
    checker = CollisionChecker(robot_radius=0.3)

    # Moving from (4.0, 5.5) to (7.0, 5.5) jumps over cell (5, 5)
    assert not checker.is_path_segment_valid((4.0, 5.5), (7.0, 5.5), grid)


def test_collision_checker_radius_sensitivity():
    """Verify that larger radius detects collisions earlier near obstacles."""
    grid = GridMap(width=10, height=10)
    grid.set_obstacle(5, 5)  # Occupies [5, 6] x [5, 6]

    # Test point at (4.5, 5.5) - distance to obstacle edge (x=5.0) is 0.5
    small_checker = CollisionChecker(robot_radius=0.35)
    large_checker = CollisionChecker(robot_radius=0.60)

    # Small robot has 0.15 margin -> safe
    assert small_checker.is_state_valid(4.4, 5.5, grid)
    # Large robot overlaps -> collision
    assert not large_checker.is_state_valid(4.4, 5.5, grid)


def test_collision_checker_boundary_containment():
    """Verify robot footprint boundary containment on all 4 borders."""
    grid = GridMap(width=10, height=10)
    checker = CollisionChecker(robot_radius=0.4)

    # Perfectly valid margin
    assert checker.is_state_valid(0.5, 5.0, grid)
    assert checker.is_state_valid(9.5, 5.0, grid)
    assert checker.is_state_valid(5.0, 0.5, grid)
    assert checker.is_state_valid(5.0, 9.5, grid)

    # Touching or crossing boundary
    assert not checker.is_state_valid(0.3, 5.0, grid)
    assert not checker.is_state_valid(9.8, 5.0, grid)
    assert not checker.is_state_valid(5.0, 0.2, grid)
    assert not checker.is_state_valid(5.0, 9.7, grid)


def test_collision_checker_narrow_passage_rejection():
    """Verify that passages too narrow for robot footprint are rejected."""
    grid = GridMap(width=10, height=10)
    # Create horizontal channel at y=5 with walls at y=4 and y=6
    for x in range(2, 8):
        grid.set_obstacle(x, 4)
        grid.set_obstacle(x, 6)

    # Robot with radius 0.35 has diameter 0.70, fits in cell height 1.0
    fitting_checker = CollisionChecker(robot_radius=0.35)
    assert fitting_checker.is_state_valid(5.5, 5.5, grid)
    assert fitting_checker.is_path_segment_valid((3.5, 5.5), (6.5, 5.5), grid)

    # Robot with radius 0.55 has diameter 1.10, exceeds cell height 1.0
    blocked_checker = CollisionChecker(robot_radius=0.55)
    assert not blocked_checker.is_state_valid(5.5, 5.5, grid)
    assert not blocked_checker.is_path_segment_valid((3.5, 5.5), (6.5, 5.5), grid)


def test_distance_segment_to_point_exact():
    """Verify analytical point-to-segment Euclidean distance calculation."""
    from autonomous_navigation.navigation.collision_checker import distance_segment_to_point

    p1 = (0.0, 0.0)
    p2 = (10.0, 0.0)

    # Perpendicular projection on segment
    assert math.isclose(distance_segment_to_point(p1, p2, (5.0, 3.0)), 3.0)
    # Closest to endpoint p1 (beyond start)
    assert math.isclose(distance_segment_to_point(p1, p2, (-4.0, 3.0)), 5.0)
    # Closest to endpoint p2 (beyond end)
    assert math.isclose(distance_segment_to_point(p1, p2, (14.0, 3.0)), 5.0)


def test_is_path_valid_empty_path():
    """An empty path must return False."""
    grid = GridMap(width=10, height=10)
    checker = CollisionChecker(robot_radius=0.3)
    assert not checker.is_path_valid([], grid)


def test_is_path_valid_single_point_valid():
    """A valid single point within bounds and away from obstacles must return True."""
    grid = GridMap(width=10, height=10)
    checker = CollisionChecker(robot_radius=0.3)
    assert checker.is_path_valid([(5.0, 5.0)], grid)


def test_is_path_valid_single_point_out_of_bounds():
    """Single point outside bounds or clipping border envelope must return False."""
    grid = GridMap(width=10, height=10)
    checker = CollisionChecker(robot_radius=0.3)
    # Direct reproduction case from defect report
    assert not checker.is_path_valid([(-1.0, 1.0)], grid)
    # Right border clipping (x + radius > 10.0)
    assert not checker.is_path_valid([(9.8, 5.0)], grid)
    # Negative Y
    assert not checker.is_path_valid([(5.0, -0.5)], grid)


def test_is_path_valid_single_point_obstacle_collision():
    """Single point inside an obstacle or within robot radius must return False."""
    grid = GridMap(width=10, height=10)
    grid.set_obstacle(5, 5)  # Occupies [5, 6] x [5, 6]
    checker = CollisionChecker(robot_radius=0.3)
    # Point directly inside obstacle
    assert not checker.is_path_valid([(5.5, 5.5)], grid)
    # Point outside obstacle but within robot radius 0.3 (dist to x=5.0 is 0.2)
    assert not checker.is_path_valid([(4.8, 5.5)], grid)


def test_is_path_valid_multipoint_path_unchanged():
    """Multi-point paths continue to validate every segment properly."""
    grid = GridMap(width=10, height=10)
    checker = CollisionChecker(robot_radius=0.3)
    valid_path = [(1.5, 1.5), (5.5, 1.5), (5.5, 5.5)]
    assert checker.is_path_valid(valid_path, grid)

    # Place obstacle intersecting first segment
    grid.set_obstacle(3, 1)
    assert not checker.is_path_valid(valid_path, grid)
