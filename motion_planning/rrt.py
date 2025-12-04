import random
import time
from typing import List, Dict, Optional
from .path_planner_interface import PathPlanner, Configuration


class RRTPlanner(PathPlanner):
    """
    Basic RRT implementation in 2D.
    Hyperparameters:
      - step_size
      - max_iterations
    """

    def __init__(
        self,
        start: Configuration,
        goal: Configuration,
        bounds: tuple[tuple[float, float], ...],
        step_size: float = 1.0,
        max_iterations: int = 1000,
    ):
        super().__init__(start, goal, bounds)
        self.step_size = step_size
        self.max_iterations = max_iterations
        self._nodes: List[Configuration] = []
        self._parents: Dict[Configuration, Optional[Configuration]] = {}
        self._planning_time: float = 0.0
        self._num_nodes: int = 0

    def _sample_random_config(self) -> Configuration:
        (xmin, xmax), (ymin, ymax) = self.bounds
        x = random.uniform(xmin, xmax)
        y = random.uniform(ymin, ymax)
        return Configuration(x, y)

    def _nearest_neighbor(self, q_rand: Configuration) -> Configuration:
        """Return node in self._nodes closest to q_rand."""
        assert self._nodes, "No nodes in RRT yet."
        best_node = self._nodes[0]
        best_dist = best_node.distance_to(q_rand)
        for node in self._nodes[1:]:
            d = node.distance_to(q_rand)
            if d < best_dist:
                best_dist = d
                best_node = node
        return best_node

    def _steer(self, q_near: Configuration, q_rand: Configuration) -> Configuration:
        """Return new config at most step_size from q_near toward q_rand."""
        dx = q_rand.x - q_near.x
        dy = q_rand.y - q_near.y
        dist = (dx * dx + dy * dy) ** 0.5
        if dist <= self.step_size:
            return Configuration(q_rand.x, q_rand.y)
        ratio = self.step_size / dist
        x_new = q_near.x + ratio * dx
        y_new = q_near.y + ratio * dy
        return Configuration(x_new, y_new)

    def _reconstruct_path(self, last: Configuration):
        path: List[Configuration] = []
        current: Optional[Configuration] = last
        while current is not None:
            path.append(current)
            current = self._parents.get(current, None)
        path.reverse()
        self.path = path

    def plan(self) -> bool:
        start_time = time.time()
        self._nodes = [self.start]
        self._parents = {self.start: None}
        success = False

        for _ in range(self.max_iterations):
            q_rand = self._sample_random_config()
            q_near = self._nearest_neighbor(q_rand)
            q_new = self._steer(q_near, q_rand)

            if not self.is_collision_free(q_near, q_new):
                continue

            self._nodes.append(q_new)
            self._parents[q_new] = q_near

            # If close enough to goal, attempt direct connection
            if q_new.distance_to(self.goal) <= self.step_size and \
               self.is_collision_free(q_new, self.goal):
                self._parents[self.goal] = q_new
                self._reconstruct_path(self.goal)
                success = True
                break

        self._planning_time = time.time() - start_time
        self._num_nodes = len(self._nodes)
        return success

    def get_planning_time(self) -> float:
        return self._planning_time

    def get_num_nodes(self) -> int:
        return self._num_nodes
