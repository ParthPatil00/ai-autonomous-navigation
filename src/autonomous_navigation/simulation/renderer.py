"""Pygame 2D top-down renderer for autonomous navigation simulation."""

import math
import os
from typing import Any, List, Optional, Set, Tuple

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False

from autonomous_navigation.config import VisualizationConfig
from autonomous_navigation.environment.grid_map import GridMap
from autonomous_navigation.perception.obstacle_detection import ScanReading
from autonomous_navigation.robot.robot_model import RobotModel


class PygameRenderer:
    """Renders 2D simulation state with support for headless execution."""

    def __init__(self, config: Optional[VisualizationConfig] = None, headless: bool = False) -> None:
        """Initialize renderer.

        Args:
            config: VisualizationConfig instance.
            headless: If True, disables window creation and graphics output.
        """
        self.config = config or VisualizationConfig()
        self.headless = headless or not PYGAME_AVAILABLE
        self.screen = None
        self.clock = None
        self.cell_size = self.config.cell_size

        if not self.headless:
            try:
                pygame.init()
                pygame.display.set_caption(self.config.window_title)
                self.clock = pygame.time.Clock()
            except pygame.error:
                self.headless = True

    def initialize_display(self, grid_width: int, grid_height: int) -> bool:
        """Create window surface sized to map dimensions."""
        if self.headless:
            return False

        width_px = grid_width * self.cell_size
        height_px = grid_height * self.cell_size
        try:
            self.screen = pygame.display.set_mode((width_px, height_px))
            return True
        except pygame.error:
            self.headless = True
            return False

    def handle_events(self) -> bool:
        """Process Pygame event queue. Returns False if user requests quit."""
        if self.headless or not self.screen:
            return True

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                return False
        return True

    def render(
        self,
        grid_map: GridMap,
        robot: RobotModel,
        start_pos: Tuple[int, int],
        goal_pos: Tuple[int, int],
        planned_path: Optional[List[Tuple[float, float]]] = None,
        explored_nodes: Optional[Set[Tuple[int, int]]] = None,
        scan_readings: Optional[List[ScanReading]] = None,
        costmap: Optional[Any] = None,
    ) -> None:
        """Render a single simulation frame.

        Args:
            grid_map: Environment occupancy grid.
            robot: Robot model.
            start_pos: Start grid coordinates.
            goal_pos: Goal grid coordinates.
            planned_path: List of (x, y) coordinates along active path.
            explored_nodes: Set of (x, y) grid cells explored by planner.
            scan_readings: List of sensor ray readings.
            costmap: Optional Costmap instance to render cost layers.
        """
        if self.headless or not self.screen:
            return

        cs = self.cell_size
        self.screen.fill((240, 240, 245))  # Light background

        # 1. Render explored nodes (if configured)
        if self.config.show_explored and explored_nodes:
            for ex, ey in explored_nodes:
                rect = pygame.Rect(ex * cs, ey * cs, cs, cs)
                pygame.draw.rect(self.screen, (220, 235, 252), rect)

        # 2. Render costmap or grid_map obstacles
        if costmap is not None:
            # Multi-layer costmap display
            for y in range(grid_map.height):
                for x in range(grid_map.width):
                    c = costmap.get_cost(x, y)
                    rect = pygame.Rect(x * cs, y * cs, cs, cs)
                    if c == costmap.LETHAL_OBSTACLE:
                        # Physical obstacle (Dark slate gray)
                        pygame.draw.rect(self.screen, (40, 40, 45), rect)
                    elif c == costmap.INSCRIBED_INFLATED:
                        # Inscribed forbidden inflation zone (Crimson red)
                        pygame.draw.rect(self.screen, (235, 87, 87), rect)
                    elif c > 0:
                        # Soft inflation gradient: yellow-orange (high cost) to pale lemon (low cost)
                        ratio = c / float(costmap.config.max_non_lethal_cost)
                        r = int(255)
                        g = int(240 - 70 * ratio)
                        b = int(200 - 150 * ratio)
                        pygame.draw.rect(self.screen, (r, g, b), rect)
        else:
            # Standard binary GridMap obstacles
            for y in range(grid_map.height):
                for x in range(grid_map.width):
                    if grid_map.is_obstacle(x, y):
                        rect = pygame.Rect(x * cs, y * cs, cs, cs)
                        pygame.draw.rect(self.screen, (50, 50, 60), rect)

        # 3. Subtle grid lines
        for x in range(grid_map.width + 1):
            pygame.draw.line(self.screen, (225, 225, 230), (x * cs, 0), (x * cs, grid_map.height * cs))
        for y in range(grid_map.height + 1):
            pygame.draw.line(self.screen, (225, 225, 230), (0, y * cs), (grid_map.width * cs, y * cs))

        # 4. Render planned path
        if self.config.show_path and planned_path and len(planned_path) > 1:
            pixel_pts = [(int(px * cs), int(py * cs)) for px, py in planned_path]
            pygame.draw.lines(self.screen, (30, 144, 255), False, pixel_pts, 3)

        # 5. Render start and goal markers
        start_px = int((start_pos[0] + 0.5) * cs)
        start_py = int((start_pos[1] + 0.5) * cs)
        goal_px = int((goal_pos[0] + 0.5) * cs)
        goal_py = int((goal_pos[1] + 0.5) * cs)

        pygame.draw.circle(self.screen, (46, 204, 113), (start_px, start_py), max(4, cs // 2))
        pygame.draw.circle(self.screen, (231, 76, 60), (goal_px, goal_py), max(4, cs // 2))

        # 6. Render sensor rays
        if self.config.show_sensors and scan_readings:
            rx_px = int(robot.position[0] * cs)
            ry_px = int(robot.position[1] * cs)
            for reading in scan_readings:
                if reading.hit_point:
                    hit_px = int(reading.hit_point[0] * cs)
                    hit_py = int(reading.hit_point[1] * cs)
                    ray_color = (255, 165, 0) if reading.hit_obstacle else (255, 235, 150)
                    pygame.draw.line(self.screen, ray_color, (rx_px, ry_px), (hit_px, hit_py), 1)

        # 7. Render robot footprint & heading
        rx = int(robot.position[0] * cs)
        ry = int(robot.position[1] * cs)
        radius_px = max(4, int(robot.radius * cs))

        pygame.draw.circle(self.screen, (52, 152, 219), (rx, ry), radius_px)
        pygame.draw.circle(self.screen, (24, 88, 140), (rx, ry), radius_px, 2)

        # Direction indicator
        nose_x = rx + int(radius_px * math.cos(robot.state.theta))
        nose_y = ry + int(radius_px * math.sin(robot.state.theta))
        pygame.draw.line(self.screen, (255, 255, 255), (rx, ry), (nose_x, nose_y), 2)

        pygame.display.flip()
        if self.clock:
            self.clock.tick(self.config.fps)

    def close(self) -> None:
        """Clean up Pygame resources."""
        if not self.headless and PYGAME_AVAILABLE:
            try:
                pygame.quit()
            except Exception:
                pass
