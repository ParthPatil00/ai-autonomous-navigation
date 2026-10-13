"""Simulation orchestrator coordinating physics, perception, planning, and control."""

from dataclasses import dataclass
import time
from typing import List, Optional, Tuple

from autonomous_navigation.config import AppConfig
from autonomous_navigation.environment.costmap import Costmap, CostmapConfig
from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.environment.obstacle_generator import ObstacleGenerator
from autonomous_navigation.evaluation.metrics import NavigationMetrics, evaluate_run
from autonomous_navigation.navigation.collision_checker import CollisionChecker
from autonomous_navigation.navigation.navigator import Navigator
from autonomous_navigation.perception.obstacle_detection import ObstacleDetector, ScanReading
from autonomous_navigation.planning.astar import AStarPlanner
from autonomous_navigation.planning.dijkstra import DijkstraPlanner
from autonomous_navigation.robot.robot_model import RobotModel
from autonomous_navigation.simulation.renderer import PygameRenderer
from autonomous_navigation.utils.logging_utils import setup_logger


@dataclass
class SimulationResult:
    """Outcome and evaluation summary of a simulation experiment."""
    success: bool
    status_message: str
    steps: int
    metrics: NavigationMetrics
    trajectory: List[Tuple[float, float]]
    planning_time_ms: float = 0.0
    simulation_time_ms: float = 0.0


class Simulator:
    """Coordinates environment, perception, planning, robot control, and rendering."""

    def __init__(self, config: Optional[AppConfig] = None) -> None:
        """Initialize simulation environment.

        Args:
            config: Optional AppConfig instance.
        """
        self.config = config or AppConfig()
        self.logger = setup_logger("Simulator", level=self.config.logging.level)

        # 1. Environment & Obstacles
        self.grid_map = GridMap(
            width=self.config.environment.width,
            height=self.config.environment.height,
            resolution=self.config.environment.resolution,
        )

        ObstacleGenerator.add_boundary_walls(self.grid_map)
        start_cell = (self.config.robot.start_pos[0], self.config.robot.start_pos[1])
        goal_cell = (self.config.robot.goal_pos[0], self.config.robot.goal_pos[1])

        ObstacleGenerator.add_random_obstacles(
            self.grid_map,
            density=self.config.environment.obstacle_density,
            seed=self.config.environment.seed,
            safe_positions=[start_cell, goal_cell],
            safe_radius=3,
        )

        # 2. Robot Model
        # Position is in world metres: cell centre = (cell_index + 0.5) * resolution
        res = self.grid_map.resolution
        start_x = (float(start_cell[0]) + 0.5) * res
        start_y = (float(start_cell[1]) + 0.5) * res
        self.robot = RobotModel(
            start_x=start_x,
            start_y=start_y,
            speed=self.config.robot.speed,
            radius=self.config.robot.radius,
        )

        # 3. Optional Costmap Layer
        self.costmap: Optional[Costmap] = None
        if self.config.costmap.enabled:
            cm_config = CostmapConfig(
                robot_radius=self.config.robot.radius,
                safety_margin=self.config.costmap.safety_margin,
                inflation_radius=self.config.costmap.inflation_radius,
                cost_scaling_factor=self.config.costmap.cost_scaling_factor,
                max_non_lethal_cost=self.config.costmap.max_non_lethal_cost,
                decay_model=self.config.costmap.decay_model,
            )
            self.costmap = Costmap(self.grid_map, cm_config)

        # 4. Perception
        self.detector = ObstacleDetector(
            max_range=self.config.robot.sensor_range,
            num_rays=self.config.robot.num_sensor_rays,
        )

        # 5. Planner selection
        if self.config.planner.algorithm.lower() == "dijkstra":
            self.planner = DijkstraPlanner(
                allow_diagonal=self.config.planner.allow_diagonal,
            )
        else:
            self.planner = AStarPlanner(
                heuristic=self.config.planner.heuristic,
                allow_diagonal=self.config.planner.allow_diagonal,
            )

        # 6. Collision & Navigator
        self.collision_checker = CollisionChecker(robot_radius=self.config.robot.radius)
        self.navigator = Navigator(
            planner=self.planner,
            collision_checker=self.collision_checker,
            waypoint_tolerance=0.15,
        )

        # 7. Renderer
        self.renderer = PygameRenderer(
            config=self.config.visualization,
            headless=self.config.simulation.headless,
        )
        if not self.config.simulation.headless:
            self.renderer.initialize_display(self.grid_map.width, self.grid_map.height)

    def run(self) -> SimulationResult:
        """Execute the autonomous navigation simulation loop.

        Returns:
            SimulationResult summary.
        """
        start_cell = (self.config.robot.start_pos[0], self.config.robot.start_pos[1])
        goal_cell = (self.config.robot.goal_pos[0], self.config.robot.goal_pos[1])

        self.logger.info("Initiating path planning from %s to %s", start_cell, goal_cell)
        plan_t0 = time.perf_counter()

        planning_map = self.costmap if self.costmap is not None else self.grid_map
        cost_weight = self.config.costmap.cost_weight if self.costmap is not None else 1.0

        smooth_path = getattr(self.config.planner, "smooth_path", False)
        planned = self.navigator.plan_to_goal(
            planning_map, start_cell, goal_cell, cost_weight=cost_weight, smooth_path=smooth_path
        )
        planning_time_ms = (time.perf_counter() - plan_t0) * 1000.0

        if not planned:
            self.logger.error("No valid path exists between start and goal.")
            metrics = evaluate_run(
                trajectory=self.robot.trajectory,
                start=self.robot.position,
                goal=(
                    (float(goal_cell[0]) + 0.5) * self.grid_map.resolution,
                    (float(goal_cell[1]) + 0.5) * self.grid_map.resolution,
                ),
                success=False,
                execution_time_ms=planning_time_ms,
                collision_count=0,
            )
            self.renderer.close()
            return SimulationResult(
                success=False,
                status_message="Path planning failed: No valid path found",
                steps=0,
                metrics=metrics,
                trajectory=self.robot.trajectory,
                planning_time_ms=planning_time_ms,
                simulation_time_ms=0.0,
            )

        self.logger.info(
            "Path planned successfully in %.2fms. Length: %d waypoints, nodes explored: %d",
            planning_time_ms,
            len(self.navigator.current_path),
            self.navigator.last_plan_result.nodes_explored if self.navigator.last_plan_result else 0,
        )

        sim_t0 = time.perf_counter()
        step = 0
        success = False
        collision_detected = False
        status_message = "Running"
        dt = self.config.simulation.dt

        while step < self.config.simulation.max_steps:
            step += 1

            # Handle user UI events
            if not self.renderer.handle_events():
                status_message = "Simulation terminated by user."
                break

            # 1. Perception scan
            scan_readings: List[ScanReading] = self.detector.scan(
                self.grid_map,
                self.robot.position[0],
                self.robot.position[1],
                self.robot.state.theta,
            )

            # 2. Check current state validity
            if not self.collision_checker.is_state_valid(
                self.robot.position[0],
                self.robot.position[1],
                self.grid_map,
            ):
                collision_detected = True
                status_message = f"Collision detected at step {step} at {self.robot.position}."
                self.logger.warning(status_message)
                break

            # 3. Check goal reached
            if self.navigator.is_goal_reached(self.robot.position):
                success = True
                status_message = f"Goal reached successfully in {step} steps!"
                self.logger.info(status_message)
                break

            # 4. Advance waypoint progress with obstacle corner verification
            self.navigator.update_waypoint_progress(self.robot.position, self.grid_map)

            # 5. Move towards current target waypoint with pre-motion continuous collision check
            target_waypoint = self.navigator.get_current_waypoint()
            if target_waypoint:
                cand_x, cand_y, cand_theta = self.robot.compute_next_position(
                    target_waypoint[0], target_waypoint[1], dt
                )
                if self.collision_checker.is_path_segment_valid(
                    self.robot.position, (cand_x, cand_y), self.grid_map
                ):
                    self.robot.apply_motion(cand_x, cand_y, cand_theta)
                else:
                    collision_detected = True
                    status_message = (
                        f"Collision prevented: motion from {self.robot.position} to "
                        f"({cand_x:.2f}, {cand_y:.2f}) blocked by obstacle."
                    )
                    self.logger.warning(status_message)
                    break

            # 6. Render frame
            explored = (
                self.navigator.last_plan_result.explored_nodes
                if self.navigator.last_plan_result
                else None
            )
            self.renderer.render(
                grid_map=self.grid_map,
                robot=self.robot,
                start_pos=start_cell,
                goal_pos=goal_cell,
                planned_path=self.navigator.current_path,
                explored_nodes=explored,
                scan_readings=scan_readings,
                costmap=self.costmap,
            )

        if step >= self.config.simulation.max_steps and not success and not collision_detected:
            status_message = f"Max steps limit reached ({self.config.simulation.max_steps})."

        sim_time_ms = (time.perf_counter() - sim_t0) * 1000.0
        metrics = evaluate_run(
            trajectory=self.robot.trajectory,
            start=(
                (float(start_cell[0]) + 0.5) * self.grid_map.resolution,
                (float(start_cell[1]) + 0.5) * self.grid_map.resolution,
            ),
            goal=(
                (float(goal_cell[0]) + 0.5) * self.grid_map.resolution,
                (float(goal_cell[1]) + 0.5) * self.grid_map.resolution,
            ),
            success=success,
            execution_time_ms=planning_time_ms + sim_time_ms,
            collision_count=1 if collision_detected else 0,
        )

        self.renderer.close()
        return SimulationResult(
            success=success,
            status_message=status_message,
            steps=step,
            metrics=metrics,
            trajectory=self.robot.trajectory,
            planning_time_ms=planning_time_ms,
            simulation_time_ms=sim_time_ms,
        )
