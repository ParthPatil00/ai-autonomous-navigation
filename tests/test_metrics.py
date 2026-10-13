"""Unit tests for Evaluation Metrics."""

import math
from autonomous_navigation.evaluation.metrics import (
    compute_path_smoothness,
    evaluate_run,
)
from autonomous_navigation.planning.path_utils import calculate_path_length


def test_path_length_calculation():
    path = [(0.0, 0.0), (3.0, 4.0)]
    assert calculate_path_length(path) == 5.0

    empty_path = []
    assert calculate_path_length(empty_path) == 0.0


def test_smoothness_straight_line():
    # Straight path has 0 angular change
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    assert compute_path_smoothness(path) == 0.0


def test_smoothness_right_turn():
    # 90 degree turn
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    smoothness = compute_path_smoothness(path)
    assert math.isclose(smoothness, math.pi / 2, rel_tol=1e-5)


def test_evaluate_run_metrics():
    trajectory = [(0.0, 0.0), (10.0, 0.0)]
    metrics = evaluate_run(
        trajectory=trajectory,
        start=(0.0, 0.0),
        goal=(10.0, 0.0),
        success=True,
        execution_time_ms=12.5,
        collision_count=0,
    )
    assert metrics.success is True
    assert metrics.path_length == 10.0
    assert metrics.straight_line_distance == 10.0
    assert metrics.path_efficiency == 1.0
    assert metrics.steps_taken == 2
    assert metrics.collision_count == 0
