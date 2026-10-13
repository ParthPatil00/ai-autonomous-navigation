# Testing Guide

This document describes the test suite structure, what each module covers, how to run tests, and how to interpret results.

---

## Overview

The project uses **pytest** for unit and regression testing. All test files live in the `tests/` directory at the repository root.

**Reported test count**: 167 tests, collected and run locally. The results below are reported locally; they have not been independently verified by a third party.

---

## Running Tests

### Full suite

```bash
pytest
```

Expected output (on a clean install with the default configuration):

```
167 passed, 1 warning in ~1.8s
```


The single warning comes from a third-party deprecation inside `pygame.pkgdata` (`pkg_resources` API). It is not a project defect.

### Verbose (shows each test name)

```bash
pytest -v
```

### Quiet summary only

```bash
pytest -q
```

### Run a single file

```bash
pytest tests/test_astar.py
pytest tests/test_path_smoother.py
```

### Run a single test function

```bash
pytest tests/test_astar.py::test_astar_diagonal_movement
```

### Stop on first failure

```bash
pytest -x
```

---

## Test Modules

### `test_grid_map.py` (8 tests)

Covers `GridMap`:
- Valid and invalid cell coordinates.
- Obstacle set, get, and clear.
- `is_free()`, `is_obstacle()`, `is_valid()`.
- Grid-to-world and world-to-grid coordinate conversions.
- NumPy array export.

---

### `test_costmap.py` (19 tests)

Covers `Costmap` and `CostmapConfig`:
- Obstacle clearance calculation at varying resolutions (1.0, 0.5, 0.25 m/cell).
- Boundary clearance (cells adjacent to map edges are treated as near an obstacle).
- Inscribed infeasible zone marking for cells within `robot_radius`.
- Soft inflation penalty decay (exponential and linear modes).
- `is_traversable()` and `is_lethal()` agreement with `CollisionChecker`.
- Headless simulation smoke-test (runs a short simulation, asserts goal is reached).
- Four multi-resolution regression tests ensuring physical scaling is consistent.

---

### `test_astar.py` (12 tests)

Covers `AStarPlanner`:
- Straight-line paths (4-connected and 8-connected).
- Diagonal movement with correct `sqrt(2)` cost.
- Obstacle avoidance around a partial wall.
- Blocked path (returns `path=None`, `cost=inf`).
- Start equals goal (returns single-cell path, cost 0).
- Start or goal inside obstacle.
- Invalid coordinate types (float, string, out-of-bounds, short tuples).
- Corner-cutting prevention through diagonal pinch points.
- Octile heuristic cost correctness.
- **Manhattan + diagonal inadmissibility warning** (`pytest.warns(UserWarning)`).

---

### `test_dijkstra.py` (8 tests)

Covers `DijkstraPlanner`:
- Straight-line shortest paths.
- Obstacle avoidance.
- Blocked paths.
- Start equals goal.
- Invalid inputs.
- Correct node-count reporting.

---

### `test_collision_checker.py` (14 tests)

Covers `CollisionChecker`:
- `is_state_valid()` for points inside/outside obstacles and at boundary distances.
- `is_path_valid()` for empty paths (returns `False`), valid single-point paths (returns `True`), out-of-bounds single points (returns `False`), and obstacle-intersecting single points (returns `False`).
- `is_path_valid()` for clear multi-point segments, obstacle-crossing segments, and paths that clip obstacle edges within robot radius.
- Behavior with different robot radii and boundary envelope containment.
- Out-of-bounds coordinate rejection.

---

### `test_path_smoother.py` (18 tests)

Covers `PathSmoother` and `SmoothingResult`:
- Step-size validation (rejects non-positive, NaN, infinite values).
- Empty path → `is_path_valid=False`.
- Valid 1-point path → `is_path_valid=True`.
- Out-of-bounds single point → `is_path_valid=False`.
- 2-point path crossing an obstacle → `is_path_valid=False`, original path preserved.
- Valid 2-point path → `is_path_valid=True`, endpoints and length preserved.
- Collinear path simplification (5 waypoints → 2).
- Staircase/detour shortcutting in open space.
- Obstacle between shortcut endpoints prevents the shortcut.
- Failed final re-validation falls back to original path with `is_path_valid=True`.
- Direct `SmoothingResult` dataclass construction preserves `is_path_valid=False`.
- Short-path metrics (sub-millimetre paths report correct non-zero lengths).

---

### `test_navigator.py` (9 tests)

Covers `Navigator`:
- `plan_to_goal()` with a successful plan.
- `step()` waypoint progression and goal detection.
- Blocked goal → plan returns `False`.
- `replan()` on an updated map.
- **Accepts valid smoothed path** and assigns it as active.
- **Accepts valid unsmoothed path** when smoother returns is_path_valid=True.
- **Rejects invalid smoothing result**: clears navigation state and returns `False` without assigning the invalid path.
- **Detects invalid remaining single-point path as blocked** via `is_path_blocked()`.


---

### `test_metrics.py` (4 tests)

Covers `evaluate_run()`:
- Path length calculation.
- Straight-line vs. actual path efficiency ratio.
- Execution time reporting.
- Angular smoothness on straight and curved paths.

---

### `test_unit_conventions.py` (75 tests)

A large parametric regression suite verifying physical unit conventions remain consistent across module boundaries:
- World-coordinate vs. grid-coordinate correspondences at multiple resolutions.
- Robot radius in metres correctly maps to inscribed inflation zone in cells.
- `CollisionChecker` and `Costmap` agree on traversability for the same point and radius.
- A\* and Dijkstra paths are collision-free according to `CollisionChecker`.

---

## Consistency Validator (Separate from pytest)

`scripts/validate_consistency.py` runs 48 targeted multi-case checks that span module boundaries:

```bash
python scripts/validate_consistency.py
```

Expected: `SUMMARY: 48 passed, 0 failed out of 48`

These checks verify:
- Clearance values at known obstacle-adjacent cells across resolutions (1.0, 0.5, 0.25 m/cell).
- `Costmap.is_traversable()` and `CollisionChecker.is_state_valid()` agree for the same (point, radius) pair.
- Boundary clearance is consistently half a cell width.
- A\* and Dijkstra find collision-free paths through 1-cell-wide corridors at appropriate robot radii, and correctly find no path when the robot is too large for the corridor.

---

## Known Limitations

- **No GUI rendering tests**: `PygameRenderer` is only tested via a headless simulation smoke-test in `test_costmap.py`. Visual output is not automatically verified.
- **Static maps only**: Tests do not cover dynamic obstacle generation mid-run.
- **No performance regression tests**: Test suite does not enforce upper bounds on planning time.
