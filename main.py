"""Main entry point for AI-Based Autonomous Navigation System."""

import argparse
import sys
from pathlib import Path

# Ensure src is on sys.path when executed directly
SRC_DIR = Path(__file__).resolve().parent / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from autonomous_navigation.config import load_config
from autonomous_navigation.simulation.simulator import Simulator
from autonomous_navigation.utils.logging_utils import setup_logger


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="AI-Based Autonomous Navigation System Simulation",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--config",
        type=str,
        default="configs/default.yaml",
        help="Path to YAML configuration file",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run simulation in headless mode without graphical window",
    )
    parser.add_argument(
        "--planner",
        type=str,
        choices=["astar", "dijkstra"],
        default=None,
        help="Path planning algorithm override",
    )
    parser.add_argument(
        "--density",
        type=float,
        default=None,
        help="Obstacle density ratio override (e.g. 0.15)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed override for map generation",
    )
    parser.add_argument(
        "--costmap",
        action="store_true",
        help="Enable clearance-aware 2D costmap layer for planning and visualization",
    )
    parser.add_argument(
        "--cost-weight",
        type=float,
        default=None,
        help="Weight multiplier for soft costmap clearance penalties (default: 1.0)",
    )
    parser.add_argument(
        "--smooth-path",
        action="store_true",
        help="Enable collision-safe path smoothing on planned waypoints",
    )
    return parser.parse_args()


def main() -> int:
    """Application main entry point."""
    args = parse_args()
    logger = setup_logger("Main")

    logger.info("Loading configuration from: %s", args.config)
    config = load_config(args.config)

    # Apply command line overrides
    if args.headless:
        config.simulation.headless = True
    if args.planner:
        config.planner.algorithm = args.planner
    if args.density is not None:
        config.environment.obstacle_density = args.density
    if args.seed is not None:
        config.environment.seed = args.seed
    if args.costmap:
        config.costmap.enabled = True
    if args.cost_weight is not None:
        config.costmap.cost_weight = args.cost_weight
    if args.smooth_path:
        config.planner.smooth_path = True

    logger.info(
        "Starting simulation: Planner=%s, Costmap=%s, Map=%dx%d, Headless=%s",
        config.planner.algorithm,
        config.costmap.enabled,
        config.environment.width,
        config.environment.height,
        config.simulation.headless,
    )

    sim = Simulator(config)
    result = sim.run()

    logger.info("Simulation completed: %s", result.status_message)
    logger.info(
        "Metrics: Success=%s, Steps=%d, Path Length=%.2f, Explored Nodes=%s, Time=%.1fms",
        result.metrics.success,
        result.metrics.steps_taken,
        result.metrics.path_length,
        sim.navigator.last_plan_result.nodes_explored if sim.navigator.last_plan_result else "N/A",
        result.metrics.execution_time_ms,
    )

    return 0 if result.success else 1


if __name__ == "__main__":
    sys.exit(main())
