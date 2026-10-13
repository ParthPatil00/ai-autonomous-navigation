"""Task 3 -- Validate planner and collision-checker consistency.

Tests both A* and Dijkstra in GridMap and Costmap modes across:
  - An obstacle adjacent to the robot
  - A robot near each map boundary
  - An empty map
  - Multi-resolution scaling (1.0, 0.5, and 0.25)
  - A route blocked by robot radius even though center is in a nominally free cell

Reports PASS/FAIL for each case with details.

COORDINATE SYSTEM CONVENTION (UNIFIED WORLD METRES):
  All positions (x, y), robot radii, and clearances are expressed in physical
  WORLD METRES.
  CollisionChecker converts internal obstacle boxes to [gx*res, (gx+1)*res] in
  world metres, matching Costmap. Both modules now agree natively on whether a
  robot fits at a given position for any resolution.
"""

import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from autonomous_navigation.environment.costmap import Costmap, CostmapConfig
from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.navigation.collision_checker import CollisionChecker
from autonomous_navigation.planning.astar import AStarPlanner
from autonomous_navigation.planning.dijkstra import DijkstraPlanner

PASS_COUNT = 0
FAIL_COUNT = 0
RESULTS = []


def report(case_name: str, passed: bool, detail: str = ""):
    global PASS_COUNT, FAIL_COUNT
    status = "PASS" if passed else "FAIL"
    if passed:
        PASS_COUNT += 1
    else:
        FAIL_COUNT += 1
    line = f"  [{status}] {case_name}"
    if detail:
        line += f"  -- {detail}"
    print(line)
    RESULTS.append({"case": case_name, "status": status, "detail": detail})


def validate_path_collision_free(path, grid_map, collision_checker):
    """Check that every waypoint in a planner path is collision-free at cell-center coords."""
    if path is None:
        return True, "No path (correctly rejected)"
    res = grid_map.resolution
    for gx, gy in path:
        wx = (gx + 0.5) * res
        wy = (gy + 0.5) * res
        if not collision_checker.is_state_valid(wx, wy, grid_map):
            return False, f"Cell ({gx},{gy}) world ({wx:.2f},{wy:.2f}) FAILS collision check"
    return True, f"{len(path)} waypoints all collision-free"


def section(title):
    print(f"\n{'='*70}")
    print(f"  {title}")
    print(f"{'='*70}")


# ============================================================
# Case 1: Obstacle adjacent to robot (resolution 1.0)
# ============================================================
section("Case 1: Obstacle adjacent to robot (res=1.0)")

grid = GridMap(10, 10, resolution=1.0)
grid.set_obstacle(4, 4)

for robot_radius, expect_traversable_5_4 in [(0.35, True), (0.6, False)]:
    cm = Costmap(grid, CostmapConfig(robot_radius=robot_radius))
    cc = CollisionChecker(robot_radius=robot_radius)

    clearance = cm.get_clearance(5, 4)
    cm_trav = cm.is_traversable(5, 4)
    cc_valid = cc.is_state_valid(5.5, 4.5, grid)  # cell center in world meters

    report(
        f"r={robot_radius} clearance at (5,4)",
        math.isclose(clearance, 0.5, rel_tol=1e-4),
        f"clearance={clearance:.4f}, expected=0.5"
    )
    report(
        f"r={robot_radius} costmap/collision agreement at (5,4)",
        cm_trav == cc_valid == expect_traversable_5_4,
        f"costmap_trav={cm_trav}, cc_valid={cc_valid}, expected={expect_traversable_5_4}"
    )

    # Planner test: plan from (5,4) to (8,8)
    for PlannerCls, pname in [(AStarPlanner, "A*"), (DijkstraPlanner, "Dijkstra")]:
        planner = PlannerCls()
        res_gm = planner.plan(grid, (5, 4), (8, 8))
        res_cm = planner.plan(cm, (5, 4), (8, 8))

        if not expect_traversable_5_4:
            # Costmap should reject start
            report(
                f"r={robot_radius} {pname} Costmap rejects forbidden start",
                res_cm.path is None,
                f"path={'None' if res_cm.path is None else len(res_cm.path)}"
            )
        else:
            # Both should find a valid path
            ok_gm, detail_gm = validate_path_collision_free(res_gm.path, grid, cc)
            ok_cm, detail_cm = validate_path_collision_free(res_cm.path, grid, cc)
            report(f"r={robot_radius} {pname} GridMap path valid", res_gm.path is not None and ok_gm, detail_gm)
            report(f"r={robot_radius} {pname} Costmap path valid", res_cm.path is not None and ok_cm, detail_cm)


# ============================================================
# Case 2: Robot near each map boundary (resolution 1.0)
# ============================================================
section("Case 2: Robot near each map boundary (res=1.0)")

grid = GridMap(10, 10, resolution=1.0)
cc = CollisionChecker(robot_radius=0.6)
cm = Costmap(grid, CostmapConfig(robot_radius=0.6))

boundary_cells = [
    ("left", 0, 5),
    ("right", 9, 5),
    ("top", 5, 0),
    ("bottom", 5, 9),
]

for edge_name, gx, gy in boundary_cells:
    wx, wy = (gx + 0.5) * 1.0, (gy + 0.5) * 1.0
    clearance = cm.get_clearance(gx, gy)
    cm_trav = cm.is_traversable(gx, gy)
    cc_valid = cc.is_state_valid(wx, wy, grid)

    report(
        f"Boundary {edge_name} cell ({gx},{gy}) clearance",
        math.isclose(clearance, 0.5, rel_tol=1e-4),
        f"clearance={clearance:.4f}"
    )
    report(
        f"Boundary {edge_name} agreement (r=0.6, clearance=0.5 -> rejected)",
        cm_trav == cc_valid == False,
        f"costmap_trav={cm_trav}, cc_valid={cc_valid}"
    )


# ============================================================
# Case 3: Empty map (resolution 1.0)
# ============================================================
section("Case 3: Empty map (res=1.0)")

grid = GridMap(10, 10, resolution=1.0)

for robot_radius in [0.35, 0.6]:
    cm = Costmap(grid, CostmapConfig(robot_radius=robot_radius))
    cc = CollisionChecker(robot_radius=robot_radius)

    # Interior cell
    wx, wy = 5.5, 5.5
    report(
        f"Empty map r={robot_radius} interior (5,5)",
        cm.is_traversable(5, 5) == cc.is_state_valid(wx, wy, grid) == True,
        f"clearance={cm.get_clearance(5,5):.2f}"
    )

    # Edge cell
    wx, wy = 0.5, 5.5
    edge_trav = cm.is_traversable(0, 5)
    edge_valid = cc.is_state_valid(wx, wy, grid)
    expect = robot_radius < 0.5
    report(
        f"Empty map r={robot_radius} edge (0,5)",
        edge_trav == edge_valid == expect,
        f"costmap_trav={edge_trav}, cc_valid={edge_valid}, expected={expect}"
    )

    # Planner can plan across empty map interior
    for pname, planner in [("A*", AStarPlanner()), ("Dijkstra", DijkstraPlanner())]:
        start, goal = (2, 2), (7, 7)
        res_cm = planner.plan(cm, start, goal)
        report(
            f"Empty map r={robot_radius} {pname} finds path on Costmap",
            res_cm.path is not None,
            f"path_len={len(res_cm.path) if res_cm.path else 'None'}"
        )


# ============================================================
# Case 4: Multi-Resolution Physical Scaling (0.5 and 0.25)
# ============================================================
section("Case 4: Multi-Resolution Physical Scaling (res=0.5 and res=0.25)")

for res in [0.5, 0.25]:
    grid = GridMap(10, 10, resolution=res)
    grid.set_obstacle(4, 4)

    # Cell (5,4) centre in physical metres: ((5+0.5)*res, (4+0.5)*res)
    wx, wy = (5.5 * res, 4.5 * res)
    expected_clearance = 0.5 * res

    cm_under = Costmap(grid, CostmapConfig(robot_radius=0.35 * res))
    cc_under = CollisionChecker(robot_radius=0.35 * res)
    clr = cm_under.get_clearance(5, 4)

    report(
        f"res={res} Costmap clearance at (5,4) = {expected_clearance:.4f}m",
        math.isclose(clr, expected_clearance, rel_tol=1e-4),
        f"clearance={clr:.4f}, expected={expected_clearance:.4f}"
    )
    report(
        f"res={res} r={0.35*res:.4f}m accepted by both Costmap and CC",
        cm_under.is_traversable(5, 4) == cc_under.is_state_valid(wx, wy, grid) == True,
        f"cm={cm_under.is_traversable(5, 4)}, cc={cc_under.is_state_valid(wx, wy, grid)}"
    )

    cm_over = Costmap(grid, CostmapConfig(robot_radius=0.6 * res))
    cc_over = CollisionChecker(robot_radius=0.6 * res)
    report(
        f"res={res} r={0.6*res:.4f}m rejected by both Costmap and CC",
        cm_over.is_traversable(5, 4) == cc_over.is_state_valid(wx, wy, grid) == False,
        f"cm={cm_over.is_traversable(5, 4)}, cc={cc_over.is_state_valid(wx, wy, grid)}"
    )

    # Edge cell at (0, 5)
    edge_wx, edge_wy = (0.5 * res, 5.5 * res)
    report(
        f"res={res} boundary clearance at (0,5) = {expected_clearance:.4f}m",
        math.isclose(cm_under.get_clearance(0, 5), expected_clearance, rel_tol=1e-4),
        f"clearance={cm_under.get_clearance(0, 5):.4f}"
    )
    report(
        f"res={res} edge cell rejected by both Costmap and CC when r > clearance",
        cm_over.is_traversable(0, 5) == cc_over.is_state_valid(edge_wx, edge_wy, grid) == False,
        f"cm={cm_over.is_traversable(0, 5)}, cc={cc_over.is_state_valid(edge_wx, edge_wy, grid)}"
    )

    # Planner execution at resolution
    cm_plan = Costmap(grid, CostmapConfig(robot_radius=0.2 * res))
    cc_plan = CollisionChecker(robot_radius=0.2 * res)
    for pname, planner in [("A*", AStarPlanner()), ("Dijkstra", DijkstraPlanner())]:
        res_plan = planner.plan(cm_plan, (1, 1), (8, 8))
        ok, detail = validate_path_collision_free(res_plan.path, grid, cc_plan)
        report(f"res={res} {pname} path collision-free", res_plan.path is not None and ok, detail)


# ============================================================
# Case 5: Radius blocks route through nominally free cell
# ============================================================
section("Case 5: Radius blocks route through nominally free cell")

# 1-cell wide corridor between two obstacle walls with a gap at x=0 to allow passage
grid = GridMap(12, 12, resolution=1.0)
for x in range(1, 12):  # leave x=0 open as gap
    grid.set_obstacle(x, 4)
    grid.set_obstacle(x, 6)

cm_blocked = Costmap(grid, CostmapConfig(robot_radius=0.6))
cc_blocked = CollisionChecker(robot_radius=0.6)

# Verify corridor cells (x >= 1) are forbidden
corridor_forbidden = all(not cm_blocked.is_traversable(x, 5) for x in range(1, 11))
report(
    "Corridor cells [1-10] at y=5 forbidden for r=0.6 (clearance=0.5)",
    corridor_forbidden,
    f"All corridor cells non-traversable"
)

# Cell (5,5) is free in GridMap but forbidden in Costmap
report(
    "Cell (5,5) free in GridMap but forbidden in Costmap",
    grid.is_free(5, 5) and not cm_blocked.is_traversable(5, 5),
    f"GridMap.is_free={grid.is_free(5,5)}, Costmap.is_traversable={cm_blocked.is_traversable(5,5)}"
)

# Verify CC also rejects the cell center
cc_valid = cc_blocked.is_state_valid(5.5, 5.5, grid)
report(
    "CC also rejects (5.5,5.5) with r=0.6",
    cc_valid == False,
    f"cc_valid={cc_valid}"
)

# Planner cannot route from (2,2) to (2,9) through the corridor
for pname, planner in [("A*", AStarPlanner()), ("Dijkstra", DijkstraPlanner())]:
    res = planner.plan(cm_blocked, (2, 2), (2, 9))
    report(
        f"r=0.6 {pname} cannot route through 1-cell corridor",
        res.path is None,
        f"path={'None' if res.path is None else len(res.path)}"
    )

# With smaller radius, corridor IS passable (route goes through gap at x=0)
cm_ok = Costmap(grid, CostmapConfig(robot_radius=0.35))
cc_ok = CollisionChecker(robot_radius=0.35)

corridor_ok = all(cm_ok.is_traversable(x, 5) for x in range(1, 11))
report(
    "Corridor cells traversable for r=0.35",
    corridor_ok,
    f"All corridor cells [1-10] at y=5 traversable"
)

for pname, planner in [("A*", AStarPlanner()), ("Dijkstra", DijkstraPlanner())]:
    res = planner.plan(cm_ok, (2, 2), (2, 9))
    if res.path:
        ok, detail = validate_path_collision_free(res.path, grid, cc_ok)
        report(f"r=0.35 {pname} routes through corridor, path valid", ok, detail)
    else:
        report(f"r=0.35 {pname} routes through corridor", False, "No path found unexpectedly")


# ============================================================
# Summary
# ============================================================
print(f"\n{'='*70}")
print(f"  SUMMARY: {PASS_COUNT} passed, {FAIL_COUNT} failed out of {PASS_COUNT + FAIL_COUNT}")
print(f"{'='*70}")

sys.exit(1 if FAIL_COUNT > 0 else 0)
