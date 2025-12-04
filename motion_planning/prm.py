import random
import time
import heapq
from typing import List, Dict, Optional
from .path_planner_interface import PathPlanner, Configuration


class PRMPlanner(PathPlanner):
    """
    Probabilistic Roadmap (PRM) in 2D.
    Hyperparameters:
      - num_samples
      - k_neighbors
    """

    def __init__(
        self,
        start: Configuration,
        goal: Configuration,
        bounds: tuple[tuple[float, float], ...],
        num_samples: int = 1000,
        k_neighbors: int = 10,
    ):
        super().__init__(start, goal, bounds)
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors
        self._samples: List[Configuration] = []
        self._graph: Dict[Configuration, List[Configuration]] = {}
        self._planning_time: float = 0.0
        self._num_nodes: int = 0

    def _sample_free_configuration(self) -> Configuration:
        """Sample random configuration not in collision."""
        (xmin, xmax), (ymin, ymax) = self.bounds
        while True:
            x = random.uniform(xmin, xmax)
            y = random.uniform(ymin, ymax)
            free = True
            for obs in self.obstacles:
                if obs.contains(x, y):
                    free = False
                    break
            if free:
                return Configuration(x, y)

    def _k_nearest(self, q: Configuration, nodes: List[Configuration]) -> List[Configuration]:
        """Return k nearest nodes to q (excluding q)."""
        dists = []
        for node in nodes:
            if node is q:
                continue
            d = q.distance_to(node)
            dists.append((d, node))
        dists.sort(key=lambda t: t[0])
        return [node for _, node in dists[: self.k_neighbors]]

    def _build_roadmap(self):
        # 1. sample free configurations
        self._samples = []
        for _ in range(self.num_samples):
            q = self._sample_free_configuration()
            self._samples.append(q)

        # 2. include start and goal
        self._samples.append(self.start)
        self._samples.append(self.goal)

        # 3. initialize adjacency list
        self._graph = {q: [] for q in self._samples}

        # 4. connect neighbors if collision-free
        for q in self._samples:
            neighbors = self._k_nearest(q, self._samples)
            for q_n in neighbors:
                if self.is_collision_free(q, q_n):
                    # undirected edge
                    self._graph[q].append(q_n)
                    self._graph[q_n].append(q)

    def _shortest_path(self) -> bool:
        """Dijkstra from start to goal on self._graph."""
        start = self.start
        goal = self.goal

        dist: Dict[Configuration, float] = {q: float("inf") for q in self._samples}
        prev: Dict[Configuration, Optional[Configuration]] = {q: None for q in self._samples}
        dist[start] = 0.0

        heap = [(0.0, start)]
        visited = set()

        while heap:
            d, u = heapq.heappop(heap)
            if u in visited:
                continue
            visited.add(u)

            if u == goal:
                break

            for v in self._graph[u]:
                alt = d + u.distance_to(v)
                if alt < dist[v]:
                    dist[v] = alt
                    prev[v] = u
                    heapq.heappush(heap, (alt, v))

        if dist[goal] == float("inf"):
            # no path
            return False

        # reconstruct path
        path: List[Configuration] = []
        current: Optional[Configuration] = goal
        while current is not None:
            path.append(current)
            current = prev[current]
        path.reverse()
        self.path = path
        return True

    def plan(self) -> bool:
        start_time = time.time()
        self._build_roadmap()
        success = self._shortest_path()
        self._planning_time = time.time() - start_time
        self._num_nodes = len(self._samples)
        return success

    def get_planning_time(self) -> float:
        return self._planning_time

    def get_num_nodes(self) -> int:
        return self._num_nodes
