"""
Abstract Base Class for Path Planning Algorithms
ROBO 201/202 Joint Assignment - Fall 2025
"""

from abc import ABC, abstractmethod


class Configuration:
    """
    Represents a 2D configuration (position) in space.
    For the 3D bonus extension, create a subclass that extends this.
    """

    def __init__(self, x: float, y: float):
        """
        Initialize a 2D configuration.

        Args:
            x: X coordinate
            y: Y coordinate
        """
        self.x = x
        self.y = y

    def to_tuple(self) -> tuple[float, float]:
        """Convert configuration to tuple."""
        return (self.x, self.y)

    def __str__(self) -> str:
        return f"Configuration({self.x}, {self.y})"

    def distance_to(self, other: "Configuration") -> float:
        """
        Calculate Euclidean distance to another configuration.

        Args:
            other: Another configuration

        Returns:
            Euclidean distance between configurations
        """
        dx = self.x - other.x
        dy = self.y - other.y
        return (dx * dx + dy * dy) ** 0.5


class PathPlanner(ABC):
    """Abstract base class for 2D path planning algorithms."""

    def __init__(
        self,
        start: Configuration,
        goal: Configuration,
        bounds: tuple[tuple[float, float], ...],
    ):
        """
        Initialize the planner.

        Args:
            start: Starting configuration
            goal: Goal configuration
            bounds: Workspace bounds as tuple of (min, max) pairs for each dimension.
                    For 2D: ((x_min, x_max), (y_min, y_max))
                    For 3D: ((x_min, x_max), (y_min, y_max), (z_min, z_max))
        """
        self.start = start
        self.goal = goal
        self.bounds = bounds
        self.obstacles = []
        self.path = None

    def set_obstacles(self, obstacles: list):
        """Set the list of obstacles in the environment."""
        self.obstacles = obstacles

    @abstractmethod
    def plan(self) -> bool:
        """
        Execute the planning algorithm.

        Returns:
            True if a path is found, False otherwise
        """
        pass

    def get_path(self) -> list[Configuration]:
        """Return the computed path as a list of Configuration objects."""
        return self.path if self.path else []

    def is_collision_free(self, config1: Configuration, config2: Configuration) -> bool:
        """
        Check if the line segment between two configurations is collision-free.

        Here we discretize the segment and ensure no sampled point is inside
        any obstacle. Each obstacle must implement a contains(x, y) method.
        """
        if not self.obstacles:
            return True

        x1, y1 = config1.to_tuple()
        x2, y2 = config2.to_tuple()

        num_samples = 20  # number of points along the segment
        for i in range(num_samples + 1):
            t = i / num_samples
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            for obs in self.obstacles:
                if obs.contains(x, y):
                    return False
        return True

    @abstractmethod
    def get_planning_time(self) -> float:
        """Return the time taken to plan (in seconds)."""
        pass

    @abstractmethod
    def get_num_nodes(self) -> int:
        """Return the number of nodes in the tree/graph."""
        pass
