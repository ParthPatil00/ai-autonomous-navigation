"""Configuration management for autonomous navigation system."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional
import yaml


@dataclass
class EnvironmentConfig:
    width: int = 50
    height: int = 50
    resolution: float = 1.0
    obstacle_density: float = 0.20
    seed: Optional[int] = 42


@dataclass
class RobotConfig:
    start_pos: List[int] = field(default_factory=lambda: [5, 5])
    goal_pos: List[int] = field(default_factory=lambda: [45, 45])
    radius: float = 0.35
    speed: float = 1.0
    sensor_range: float = 6.0
    num_sensor_rays: int = 36


@dataclass
class PlannerConfig:
    algorithm: str = "astar"
    heuristic: str = "euclidean"
    allow_diagonal: bool = True
    smooth_path: bool = False


@dataclass
class SimulationConfig:
    max_steps: int = 1000
    dt: float = 0.1
    headless: bool = False


@dataclass
class VisualizationConfig:
    cell_size: int = 14
    fps: int = 30
    window_title: str = "AI Autonomous Navigation Simulation"
    show_sensors: bool = True
    show_path: bool = True
    show_explored: bool = True


@dataclass
class LoggingConfig:
    level: str = "INFO"


@dataclass
class CostmapLayerConfig:
    enabled: bool = False
    inflation_radius: float = 1.5
    safety_margin: float = 0.0
    cost_scaling_factor: float = 3.0
    max_non_lethal_cost: int = 199
    decay_model: str = "exponential"
    cost_weight: float = 1.0


@dataclass
class AppConfig:
    environment: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    costmap: CostmapLayerConfig = field(default_factory=CostmapLayerConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    visualization: VisualizationConfig = field(default_factory=VisualizationConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)


def load_config(config_path: Optional[str | Path] = None) -> AppConfig:
    """Load configuration from a YAML file or use default settings if None or not found.

    Args:
        config_path: Path to the YAML configuration file.

    Returns:
        AppConfig populated with settings.
    """
    if config_path is None:
        default_path = Path("configs/default.yaml")
        if default_path.is_file():
            config_path = default_path
        else:
            return AppConfig()

    path = Path(config_path)
    if not path.is_file():
        return AppConfig()

    with open(path, "r", encoding="utf-8") as f:
        data: Dict[str, Any] = yaml.safe_load(f) or {}

    env_data = data.get("environment", {})
    robot_data = data.get("robot", {})
    planner_data = data.get("planner", {})
    costmap_data = data.get("costmap", {})
    sim_data = data.get("simulation", {})
    vis_data = data.get("visualization", {})
    log_data = data.get("logging", {})

    return AppConfig(
        environment=EnvironmentConfig(**env_data),
        robot=RobotConfig(**robot_data),
        planner=PlannerConfig(**planner_data),
        costmap=CostmapLayerConfig(**costmap_data),
        simulation=SimulationConfig(**sim_data),
        visualization=VisualizationConfig(**vis_data),
        logging=LoggingConfig(**log_data),
    )
