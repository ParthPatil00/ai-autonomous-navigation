"""Benchmarking script comparing path planning and navigation algorithms across reproducible scenarios."""

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Dict, List

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.environment.obstacle_generator import ObstacleGenerator
from autonomous_navigation.navigation.collision_checker import CollisionChecker
from autonomous_navigation.navigation.navigator import Navigator
from autonomous_navigation.planning.astar import AStarPlanner
from autonomous_navigation.planning.dijkstra import DijkstraPlanner
from autonomous_navigation.planning.path_utils import calculate_path_length
from autonomous_navigation.robot.robot_model import RobotModel
from autonomous_navigation.utils.logging_utils import setup_logger


def simulate_plan_execution(
    grid: GridMap,
    path: List[tuple],
    start_pos: tuple,
    goal_pos: tuple,
    robot_radius: float = 0.35,
    speed: float = 1.0,
    dt: float = 0.1,
    max_steps: int = 1500,
) -> Dict[str, Any]:
    """Execute planned path in a headless simulation loop with continuous collision checking."""
    res = grid.resolution
    start_x = (float(start_pos[0]) + 0.5) * res
    start_y = (float(start_pos[1]) + 0.5) * res
    robot = RobotModel(start_x=start_x, start_y=start_y, speed=speed, radius=robot_radius)
    collision_checker = CollisionChecker(robot_radius=robot_radius)

    # Waypoints in cell center continuous world coordinates (meters)
    waypoints = [((float(x) + 0.5) * res, (float(y) + 0.5) * res) for x, y in path]
    waypoint_idx = 0
    goal_center = ((float(goal_pos[0]) + 0.5) * res, (float(goal_pos[1]) + 0.5) * res)
    waypoint_tol = 0.15
    goal_tol = 0.4

    sim_t0 = time.perf_counter()
    step = 0
    goal_reached = False
    collision_count = 0

    while step < max_steps:
        step += 1

        # Check goal reached
        dist_to_goal = math.hypot(robot.position[0] - goal_center[0], robot.position[1] - goal_center[1])
        if dist_to_goal <= goal_tol:
            goal_reached = True
            break

        # Advance waypoints safely
        while waypoint_idx < len(waypoints):
            target = waypoints[waypoint_idx]
            if math.hypot(robot.position[0] - target[0], robot.position[1] - target[1]) <= waypoint_tol:
                if waypoint_idx + 1 < len(waypoints):
                    next_target = waypoints[waypoint_idx + 1]
                    if not collision_checker.is_path_segment_valid(robot.position, next_target, grid):
                        break
                waypoint_idx += 1
            else:
                break

        if waypoint_idx >= len(waypoints):
            target = goal_center
        else:
            target = waypoints[waypoint_idx]

        # Compute next step and check collision
        cand_x, cand_y, cand_theta = robot.compute_next_position(target[0], target[1], dt)
        if collision_checker.is_path_segment_valid(robot.position, (cand_x, cand_y), grid):
            robot.apply_motion(cand_x, cand_y, cand_theta)
        else:
            collision_count += 1
            break

    execution_time_ms = (time.perf_counter() - sim_t0) * 1000.0

    return {
        "goal_reached": goal_reached,
        "collision_count": collision_count,
        "execution_steps": step,
        "execution_time_ms": round(execution_time_ms, 3),
        "path_length": round(calculate_path_length(robot.trajectory), 3),
    }


def run_benchmark(
    num_scenarios: int = 100,
    base_seed: int = 1000,
    width: int = 50,
    height: int = 50,
    density: float = 0.20,
    start_pos: tuple = (5, 5),
    goal_pos: tuple = (45, 45),
    output_path: str = "outputs/metrics/benchmark_results.json",
) -> List[Dict[str, Any]]:
    """Run comparative benchmark between A* and Dijkstra across reproducible scenarios."""
    logger = setup_logger("Benchmark")
    logger.info(
        "Initiating reproducible benchmark: %d scenarios on %dx%d grid (density=%.2f, base_seed=%d)",
        num_scenarios,
        width,
        height,
        density,
        base_seed,
    )

    planners = {
        "A* (Euclidean)": AStarPlanner(heuristic="euclidean", allow_diagonal=True),
        "A* (Octile)": AStarPlanner(heuristic="octile", allow_diagonal=True),
        "Dijkstra": DijkstraPlanner(allow_diagonal=True),
    }

    results: List[Dict[str, Any]] = []

    for scenario_idx in range(num_scenarios):
        seed = base_seed + scenario_idx
        grid = GridMap(width=width, height=height)
        ObstacleGenerator.add_boundary_walls(grid)
        ObstacleGenerator.add_random_obstacles(
            grid,
            density=density,
            seed=seed,
            safe_positions=[start_pos, goal_pos],
            safe_radius=3,
        )

        for name, planner in planners.items():
            t_plan_start = time.perf_counter()
            plan_res = planner.plan(grid, start_pos, goal_pos)
            planning_time_ms = (time.perf_counter() - t_plan_start) * 1000.0
            planning_succeeded = bool(plan_res.path is not None)

            if planning_succeeded:
                sim_res = simulate_plan_execution(
                    grid=grid,
                    path=plan_res.path,
                    start_pos=start_pos,
                    goal_pos=goal_pos,
                )
                goal_reached = sim_res["goal_reached"]
                collision_count = sim_res["collision_count"]
                execution_steps = sim_res["execution_steps"]
                execution_time_ms = sim_res["execution_time_ms"]
                path_length = sim_res["path_length"]
                path_cost = round(plan_res.cost, 3)
            else:
                goal_reached = False
                collision_count = 0
                execution_steps = 0
                execution_time_ms = 0.0
                path_length = None
                path_cost = None

            record = {
                "scenario_id": scenario_idx + 1,
                "seed": seed,
                "planner_name": name,
                "grid_width": width,
                "grid_height": height,
                "obstacle_density": density,
                "robot_radius": 0.35,
                "start_pos": list(start_pos),
                "goal_pos": list(goal_pos),
                "movement_mode": "8-connected (diagonal allowed)",
                "planning_succeeded": planning_succeeded,
                "goal_reached": goal_reached,
                "path_cost": path_cost,
                "path_length": path_length,
                "planning_time_ms": round(planning_time_ms, 3),
                "execution_time_ms": execution_time_ms,
                "nodes_explored": plan_res.nodes_explored,
                "collision_count": collision_count,
                "execution_steps": execution_steps,
            }
            results.append(record)

    # Print summary table
    print("\n" + "=" * 105)
    print(
        f"{'Planner':<18} | {'Plan OK':<8} | {'Goal OK':<8} | {'Avg Explored':<14} | "
        f"{'Avg Plan (ms)':<14} | {'Avg Sim (ms)':<13} | {'Avg Cost':<10}"
    )
    print("-" * 105)

    for name in planners.keys():
        alg_records = [r for r in results if r["planner_name"] == name]
        plan_ok_count = sum(1 for r in alg_records if r["planning_succeeded"])
        goal_ok_count = sum(1 for r in alg_records if r["goal_reached"])
        successful_plans = [r for r in alg_records if r["planning_succeeded"]]

        avg_explored = (
            sum(r["nodes_explored"] for r in alg_records) / len(alg_records)
            if alg_records
            else 0.0
        )
        avg_plan_time = (
            sum(r["planning_time_ms"] for r in alg_records) / len(alg_records)
            if alg_records
            else 0.0
        )
        avg_sim_time = (
            sum(r["execution_time_ms"] for r in successful_plans) / len(successful_plans)
            if successful_plans
            else 0.0
        )
        avg_cost = (
            sum(r["path_cost"] for r in successful_plans) / len(successful_plans)
            if successful_plans
            else 0.0
        )

        print(
            f"{name:<18} | {f'{plan_ok_count}/{num_scenarios}':<8} | {f'{goal_ok_count}/{num_scenarios}':<8} | "
            f"{avg_explored:<14.1f} | {avg_plan_time:<14.2f} | {avg_sim_time:<13.2f} | {avg_cost:<10.2f}"
        )
    print("=" * 105 + "\n")

    # Save to JSON with structured metadata and results
    benchmark_payload = {
        "metadata": {
            "num_scenarios": num_scenarios,
            "base_seed": base_seed,
            "grid_width": width,
            "grid_height": height,
            "density": density,
            "start_pos": list(start_pos),
            "goal_pos": list(goal_pos),
            "robot_radius": 0.35,
            "movement_mode": "8-connected (diagonal allowed)",
            "benchmark_version": "milestone3_verified",
        },
        "records": results,
    }

    out_file = Path(output_path)
    out_file.parent.mkdir(parents=True, exist_ok=True)
    with open(out_file, "w", encoding="utf-8") as f:
        json.dump(benchmark_payload, f, indent=2)
    logger.info("Saved %d benchmark records to %s", len(results), out_file)

    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Reproducible Path Planning & Navigation Benchmarks")
    parser.add_argument("--scenarios", "--runs", dest="scenarios", type=int, default=100, help="Number of benchmark scenarios (min 100)")
    parser.add_argument("--base-seed", type=int, default=1000, help="Starting random seed")
    parser.add_argument("--density", type=float, default=0.20, help="Obstacle density ratio")
    parser.add_argument("--output", type=str, default="outputs/metrics/benchmark_results.json", help="Path for JSON output")
    args = parser.parse_args()

    run_benchmark(
        num_scenarios=args.scenarios,
        base_seed=args.base_seed,
        density=args.density,
        output_path=args.output,
    )
