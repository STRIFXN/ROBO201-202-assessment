import math


class Configuration:
    """2D configuration: (x, y)."""

    def __init__(self, x: float, y: float):
        self.x = float(x)
        self.y = float(y)

    def distance_to(self, other: "Configuration") -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        return math.hypot(dx, dy)

    def to_tuple(self):
        return (self.x, self.y)

    def __repr__(self):
        return f"Configuration({self.x:.3f}, {self.y:.3f})"

    # Let Configuration be used as dict keys (for parents/graph)
    def __eq__(self, other):
        if not isinstance(other, Configuration):
            return False
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))


# BONUS 3D (optional for extra marks)
class Configuration3D(Configuration):
    def __init__(self, x: float, y: float, z: float):
        super().__init__(x, y)
        self.z = float(z)

    def distance_to(self, other: "Configuration3D") -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - other.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def to_tuple(self):
        return (self.x, self.y, self.z)

    def __repr__(self):
        return f"Configuration3D({self.x:.3f}, {self.y:.3f}, {self.z:.3f})"
