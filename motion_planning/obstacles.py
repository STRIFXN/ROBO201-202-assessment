import random
from abc import ABC, abstractmethod
from typing import Tuple, List


class Obstacle(ABC):
    @abstractmethod
    def contains(self, x: float, y: float) -> bool:
        """Return True if point (x, y) is inside the obstacle."""
        pass


class CircleObstacle(Obstacle):
    def __init__(self, center: Tuple[float, float], radius: float):
        self.cx, self.cy = center
        self.r = radius

    def contains(self, x: float, y: float) -> bool:
        dx = x - self.cx
        dy = y - self.cy
        return dx * dx + dy * dy <= self.r * self.r

    def __repr__(self):
        return f"CircleObstacle(center=({self.cx:.2f}, {self.cy:.2f}), r={self.r:.2f})"


class RectangleObstacle(Obstacle):
    """Axis-aligned rectangle obstacle."""

    def __init__(self, xmin: float, xmax: float, ymin: float, ymax: float):
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax

    def contains(self, x: float, y: float) -> bool:
        return (self.xmin <= x <= self.xmax) and (self.ymin <= y <= self.ymax)

    def __repr__(self):
        return (
            f"RectangleObstacle(xmin={self.xmin:.2f}, xmax={self.xmax:.2f}, "
            f"ymin={self.ymin:.2f}, ymax={self.ymax:.2f})"
        )


def generate_random_obstacles(num_obstacles: int, bounds):
    """
    Generate random circle/rectangle obstacles inside bounds.
    bounds = ((xmin, xmax), (ymin, ymax))
    """
    (xmin, xmax), (ymin, ymax) = bounds
    obstacles: List[Obstacle] = []

    for _ in range(num_obstacles):
        shape_type = random.choice(["circle", "rect"])

        if shape_type == "circle":
            cx = random.uniform(xmin, xmax)
            cy = random.uniform(ymin, ymax)
            r = random.uniform(0.4, 1.0)  # adjust if needed
            obstacles.append(CircleObstacle((cx, cy), r))
        else:
            x1 = random.uniform(xmin, xmax)
            x2 = random.uniform(xmin, xmax)
            y1 = random.uniform(ymin, ymax)
            y2 = random.uniform(ymin, ymax)
            xmin_o, xmax_o = min(x1, x2), max(x1, x2)
            ymin_o, ymax_o = min(y1, y2), max(y1, y2)
            obstacles.append(RectangleObstacle(xmin_o, xmax_o, ymin_o, ymax_o))

    return obstacles
