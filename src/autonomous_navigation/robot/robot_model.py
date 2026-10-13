"""Robot state and planar kinematic motion model."""

from dataclasses import dataclass, field
import math
from typing import List, Tuple


@dataclass
class RobotState:
    """Represents the instantaneous planar pose and velocity of the robot."""
    x: float
    y: float
    theta: float = 0.0  # Heading in radians
    speed: float = 1.0


class RobotModel:
    """Planar kinematic robot model simulating differential motion and history tracking."""

    def __init__(
        self,
        start_x: float = 0.0,
        start_y: float = 0.0,
        theta: float = 0.0,
        speed: float = 1.0,
        radius: float = 0.35,
    ) -> None:
        """Initialize RobotModel.

        Args:
            start_x: Initial X coordinate.
            start_y: Initial Y coordinate.
            theta: Initial orientation in radians.
            speed: Motion velocity (grid units per second).
            radius: Circular bounding footprint radius.
        """
        self.state = RobotState(x=float(start_x), y=float(start_y), theta=float(theta), speed=float(speed))
        self.radius = float(radius)
        self.trajectory: List[Tuple[float, float]] = [(self.state.x, self.state.y)]

    @property
    def position(self) -> Tuple[float, float]:
        """Current (x, y) coordinates."""
        return (self.state.x, self.state.y)

    def compute_next_position(self, target_x: float, target_y: float, dt: float) -> Tuple[float, float, float]:
        """Compute candidate next (x, y, theta) without mutating state.

        Args:
            target_x: Destination X.
            target_y: Destination Y.
            dt: Time step duration in seconds.

        Returns:
            Tuple of (candidate_x, candidate_y, candidate_theta).
        """
        dx = target_x - self.state.x
        dy = target_y - self.state.y
        dist = math.hypot(dx, dy)

        if dist < 1e-6:
            return self.state.x, self.state.y, self.state.theta

        theta = math.atan2(dy, dx)
        step_distance = min(dist, self.state.speed * dt)

        cand_x = self.state.x + (dx / dist) * step_distance
        cand_y = self.state.y + (dy / dist) * step_distance
        return cand_x, cand_y, theta

    def apply_motion(self, x: float, y: float, theta: float) -> Tuple[float, float]:
        """Apply a validated motion update and record trajectory."""
        self.state.x = float(x)
        self.state.y = float(y)
        self.state.theta = float(theta)
        self.trajectory.append((self.state.x, self.state.y))
        return self.position

    def move_towards(self, target_x: float, target_y: float, dt: float) -> Tuple[float, float]:
        """Advance robot towards target coordinates using planar velocity kinematics.

        Args:
            target_x: Destination X.
            target_y: Destination Y.
            dt: Time step duration in seconds.

        Returns:
            Updated (x, y) robot position.
        """
        cand_x, cand_y, cand_theta = self.compute_next_position(target_x, target_y, dt)
        return self.apply_motion(cand_x, cand_y, cand_theta)

    def teleport(self, x: float, y: float, theta: float = 0.0) -> None:
        """Instantly update robot pose without recording incremental path."""
        self.state.x = float(x)
        self.state.y = float(y)
        self.state.theta = float(theta)
        self.trajectory = [(self.state.x, self.state.y)]
