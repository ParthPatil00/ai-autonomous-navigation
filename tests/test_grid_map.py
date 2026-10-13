"""Unit tests for GridMap."""

import numpy as np
import pytest

from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.environment.obstacle_generator import ObstacleGenerator


def test_grid_map_initialization():
    grid = GridMap(width=20, height=15, resolution=0.5)
    assert grid.width == 20
    assert grid.height == 15
    assert grid.resolution == 0.5
    assert grid.is_free(0, 0)
    assert not grid.is_obstacle(0, 0)


def test_grid_map_invalid_dimensions():
    with pytest.raises(ValueError):
        GridMap(width=-1, height=10)
    with pytest.raises(ValueError):
        GridMap(width=10, height=0)
    with pytest.raises(ValueError):
        GridMap(width=10, height=10, resolution=-0.1)


def test_grid_map_boundary_checking():
    grid = GridMap(width=10, height=10)
    assert grid.is_valid(0, 0)
    assert grid.is_valid(9, 9)
    assert not grid.is_valid(10, 5)
    assert not grid.is_valid(-1, 5)
    assert not grid.is_valid(5, -1)
    assert not grid.is_valid(5, 10)
    # Out of bounds is treated as obstacle
    assert grid.is_obstacle(-1, 0)
    assert grid.is_obstacle(15, 15)


def test_grid_map_obstacle_set_and_clear():
    grid = GridMap(width=10, height=10)
    assert grid.is_free(3, 4)
    grid.set_obstacle(3, 4)
    assert grid.is_obstacle(3, 4)
    assert not grid.is_free(3, 4)
    grid.set_free(3, 4)
    assert grid.is_free(3, 4)
    grid.set_obstacle(1, 1)
    grid.clear()
    assert grid.is_free(1, 1)


def test_grid_map_numpy_conversions():
    arr = np.zeros((8, 12), dtype=np.uint8)
    arr[2, 3] = 1
    grid = GridMap.from_numpy(arr, resolution=1.0)
    assert grid.width == 12
    assert grid.height == 8
    assert grid.is_obstacle(3, 2)
    assert grid.is_free(0, 0)

    exported = grid.to_numpy()
    assert np.array_equal(arr, exported)


def test_obstacle_generator_boundary():
    grid = GridMap(width=10, height=10)
    ObstacleGenerator.add_boundary_walls(grid)
    for x in range(10):
        assert grid.is_obstacle(x, 0)
        assert grid.is_obstacle(x, 9)
    for y in range(10):
        assert grid.is_obstacle(0, y)
        assert grid.is_obstacle(9, y)
    assert grid.is_free(5, 5)


def test_obstacle_generator_repeatable_seed():
    grid1 = GridMap(width=20, height=20)
    grid2 = GridMap(width=20, height=20)
    grid3 = GridMap(width=20, height=20)

    ObstacleGenerator.add_random_obstacles(grid1, density=0.25, seed=12345)
    ObstacleGenerator.add_random_obstacles(grid2, density=0.25, seed=12345)
    ObstacleGenerator.add_random_obstacles(grid3, density=0.25, seed=54321)

    assert np.array_equal(grid1.to_numpy(), grid2.to_numpy())
    assert not np.array_equal(grid1.to_numpy(), grid3.to_numpy())


def test_grid_map_non_integer_types():
    grid = GridMap(width=10, height=10)
    assert not grid.is_valid(1.5, 2.0)
    assert not grid.is_valid(True, False)
    assert not grid.is_valid("1", "2")
    assert grid.is_obstacle(1.5, 2.0)
    assert grid.is_obstacle(True, False)
