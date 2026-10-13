# Defect Fix & Unit Convention Resolution Report

**Date**: 2026-10-11
**Milestone**: Physical Unit Unification & Multi-Resolution Consistency Verification
**Status**: Complete

---

## 1. Audit of Unit Conventions & Physical Semantics

### Documented Unit Conventions

Prior to this resolution, a latent inconsistency existed across the system when `resolution != 1.0`:
- `Costmap` computed clearances and applied inflation in **world meters** (`cx = (gx + 0.5) * resolution`, obstacle box = `[gx*res, (gx+1)*res]`).
- `CollisionChecker` checked bounding boxes and obstacle bounds directly in **grid-cell units** (`x - radius < 0` vs `grid_map.width`, obstacle box = `[gx, gx + 1.0]`).
- `Navigator` generated waypoints as `float(x) + 0.5` without resolution scaling.
- `RobotModel` and `Simulator` tracked positions without a formal definition of units.

### Canonical Physical Interpretation Selected

All continuous geometric quantities across the entire system are now explicitly standardized to **world meters**:
- **Grid Coordinates**: Discrete integer indices `(gx, gy)` designating cell row and column.
- **World Coordinates**: Continuous float coordinates `(wx, wy)` in meters. Cell center is `((gx + 0.5) * resolution, (gy + 0.5) * resolution)`.
- **Grid Resolution**: Real-world scale factor in meters per grid cell (`resolution > 0`).
- **Robot Radius**: Inscribed physical circular footprint radius in **world meters**.
- **Obstacle Clearance**: Shortest Euclidean distance from robot center to square obstacle boundary in **world meters**.
- **Inflation Radius & Safety Margin**: Hyperparameters in **world meters**.
- **Path Length & Path Cost**: Computed in **world meters** (`step_cost = 1.0` or `sqrt(2)` grid units for movement, scaled by resolution in physical execution).

---

## 2. Implementation Corrections

1. **`CollisionChecker` (`src/autonomous_navigation/navigation/collision_checker.py`)**:
   - Updated boundary check to `grid_map.width * res` and `grid_map.height * res`.
   - Updated obstacle bounding box representation to `[gx * res, (gx + 1) * res]`.
   - Converted robot footprint AABB bounds to grid coordinates via `res` with `_eps = 1e-9` buffer for exact boundary touching.
   - Updated docstrings to explicitly state world-meter conventions.

2. **`Navigator` (`src/autonomous_navigation/navigation/navigator.py`)**:
   - `plan_to_goal()` converts planner grid indices to world-meter waypoints: `((gx + 0.5) * res, (gy + 0.5) * res)`.
   - `is_goal_reached()` computes distance to goal center scaled by `res`.
   - `replan()` converts world coordinates back to discrete grid indices `math.floor(x / res)`.

3. **`Simulator` (`src/autonomous_navigation/simulation/simulator.py`)**:
   - Robot start position initialized in world meters: `(start_cell + 0.5) * res`.
   - Simulation metrics evaluated using world-meter start and goal positions.

4. **`Costmap` (`src/autonomous_navigation/environment/costmap.py`)**:
   - Removed dead/unused `scipy` imports (`distance_transform_edt` is not used).
   - Documented pure NumPy implementation with vectorized analytical point-to-box distance transform.
   - Updated `get_clearance()` docstring to state world meters.

5. **Configuration (`configs/default.yaml` & `src/autonomous_navigation/config.py`)**:
   - Updated comments and field documentation to reflect world meters for `radius`, `speed`, and `sensor_range`.

6. **Benchmark Runner (`scripts/run_benchmarks.py`)**:
   - Updated `simulate_plan_execution()` to scale cell centers and waypoints by `grid.resolution`.

---

## 3. Verification & Multi-Resolution Test Results

### 3.1 Unit Test Suite (60 unit tests + 65 multi-resolution tests = 125 total)

**Command:**
```bash
py -3.13 -m pytest tests/ -v --tb=short
```

**Results:**
- Total tests: **125 passed**, 0 failed, 1 warning (`pkg_resources` deprecation in pygame).
- Verified at resolutions **1.0**, **0.5**, and **0.25**:
  - `TestWorldGridConversions`: Grid-to-world, world-to-grid round-trips.
  - `TestObstacleAdjacent`: Clearance exactness (`0.5 * res`), safe acceptance for `r < 0.5*res`, rejection for `r >= 0.5*res`, and perfect Costmap/CC agreement.
  - `TestMapBoundaries`: Boundary clearance (`0.5 * res`) and all 4 border rejections when `r > 0.5*res`.
  - `TestEmptyMap`: Interior traversability and planner path generation.
  - `TestNarrowCorridors`: Passage clearance validation and planner rejection/routing.
  - `TestCostmapCCPhysicalAgreement`: Agreement across all map cells.
  - `TestPlannerPathCollisionFree`: A* and Dijkstra path validation.

### 3.2 Consistency Validator (48 test cases)

**Command:**
```bash
py -3.13 scripts/validate_consistency.py
```

**Results:**
- **48 passed**, 0 failed across all cases (Obstacle-adjacent, Boundary, Empty map, Resolution 0.5 & 0.25 physical scaling, and Radius-blocked corridors).

### 3.3 Headless Simulation Combinations

All four combinations executed to goal completion with 0 collisions:

| Planner | Map Layer | Status | Steps | Exec Time | Path Length |
|---|---|---|---|---|---|
| A* | GridMap | Success | 633 | ~568 ms | 63.20 m |
| Dijkstra | GridMap | Success | 634 | ~598 ms | 63.30 m |
| A* | Costmap | Success | 633 | ~572 ms | 63.20 m |
| Dijkstra | Costmap | Success | 634 | ~686 ms | 63.30 m |

### 3.4 Benchmark Baseline Regeneration

**Command:**
```bash
py -3.13 scripts/run_benchmarks.py --scenarios 100 --base-seed 2000 --density 0.20 --output outputs/metrics/benchmark_results_unified_units.json
```

**Results (100 scenarios, 50x50 map, resolution 1.0, seed 2000):**

| Planner | Plan OK | Goal OK | Avg Explored | Avg Plan (ms) | Avg Sim (ms) | Avg Cost | Collisions |
|---|---|---|---|---|---|---|---|
| A* (Euclidean) | 100/100 | 100/100 | 691.0 | 13.73 | 19.64 | 64.01 | 0 |
| A* (Octile) | 100/100 | 100/100 | 461.6 | 9.82 | 19.69 | 64.01 | 0 |
| Dijkstra | 100/100 | 100/100 | 1829.2 | 34.09 | 19.03 | 64.01 | 0 |

*(Archived historical baselines preserved at `outputs/metrics/benchmark_results.json` and `benchmark_results_postfix.json`).*

---

## 4. Remaining System Limitations

1. **Circular Footprint Approximation**: The robot footprint remains modeled as a circle of radius `r`; non-circular/polygonal footprints are not yet supported.
2. **Static Map Assumption**: Clearance and collision checking assume static obstacles throughout planning and execution.
3. **Discrete Path Step Interpolation**: Discrete continuous collision checks utilize dense sub-stepping (`min(step_size, radius/3)`), which guarantees safety down to the sub-step tolerance.

---

## 5. Summary of Modified Files

| File | Changes Made |
|---|---|
| `src/autonomous_navigation/navigation/collision_checker.py` | Converted all boundary, obstacle box, and swept volume checks to physical world meters with resolution scaling and epsilon safety buffers. |
| `src/autonomous_navigation/navigation/navigator.py` | Added resolution-aware waypoint generation, goal check, and replanning. |
| `src/autonomous_navigation/simulation/simulator.py` | Updated robot start pose and metric evaluations to world meters. |
| `src/autonomous_navigation/environment/costmap.py` | Removed dead SciPy imports; clarified NumPy point-to-box distance transform and world meter docstrings. |
| `src/autonomous_navigation/robot/robot_model.py` | Updated docstrings clarifying physical meter coordinates. |
| `src/autonomous_navigation/config.py` & `configs/default.yaml` | Updated unit descriptions in configuration files. |
| `scripts/run_benchmarks.py` | Updated headless benchmark simulation loop to scale positions by map resolution. |
| `scripts/validate_consistency.py` | Updated consistency validator covering unified world meter semantics. |
| `tests/test_unit_conventions.py` | **New**: Comprehensive 60-test multi-resolution regression suite (resolutions 1.0, 0.5, 0.25). |
| `outputs/metrics/benchmark_results_unified_units.json` | **New**: Newly regenerated benchmark results with unified coordinate semantics. |
