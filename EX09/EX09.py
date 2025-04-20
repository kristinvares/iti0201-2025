"""EX09."""
import math
from collections import deque


class Robot:
    """Turtlebot robot."""

    def __init__(self, robot: object) -> None:
        """Initialize the robot with default values and state."""
        self.robot = robot
        self.accessible = [(0, 0)]
        self.unknown = []
        self.layout = {}
        self.lidar_scan = None
        self.heading = None
        self.coords = None
        self.target = None

    def get_traversable_cells(self) -> list:
        """Return a list of known traversable cells."""
        return self.accessible

    def get_unmapped_cells(self) -> list:
        """Return a list of known but unmapped cells."""
        return self.unknown

    def get_map(self) -> dict:
        """Return the current map layout as a graph."""
        return self.layout

    def add_segments(self, count, direction):
        """Add visible segments in a specific direction from current position."""
        x, y = self.coords
        shifts = {
            "up": (0, 1),
            "down": (0, -1),
            "back": (0, -1),
            "left": (-1, 0),
            "right": (1, 0)
        }

        if direction not in shifts:
            return

        dx, dy = shifts[direction]

        for step in range(1, count + 1):
            pos = (x + dx * step, y + dy * step)

            if step == 1:
                self.layout.setdefault(self.coords, []).append(pos)
                self.layout.setdefault(pos, []).append(self.coords)

            if pos not in self.accessible:
                self.accessible.append(pos)
                self.unknown.append(pos)

    def _forward(self):
        """Add cells when facing 'up' (north-like direction)."""
        if self.ahead > 0.45:
            dist = self.ahead // 0.625
            self.add_segments(int(dist), "up")
        if self.behind > 0.45:
            dist = self.behind // 0.625
            self.add_segments(int(dist), "back")
        if self.rightside > 0.45:
            dist = self.rightside // 0.625
            self.add_segments(int(dist), "right")
        if self.leftside > 0.45:
            dist = self.leftside // 0.625
            self.add_segments(int(dist), "left")

    def _left_turn(self):
        """Add cells when facing left."""
        if self.ahead > 0.45:
            dist = self.ahead // 0.625
            self.add_segments(int(dist), "left")
        if self.behind > 0.45:
            dist = self.behind // 0.625
            self.add_segments(int(dist), "right")
        if self.rightside > 0.45:
            dist = self.rightside // 0.625
            self.add_segments(int(dist), "up")
        if self.leftside > 0.45:
            dist = self.leftside // 0.625
            self.add_segments(int(dist), "back")

    def _right_turn(self):
        """Add cells when facing right."""
        if self.ahead > 0.45:
            dist = self.ahead // 0.625
            self.add_segments(int(dist), "right")
        if self.behind > 0.45:
            dist = self.behind // 0.625
            self.add_segments(int(dist), "left")
        if self.rightside > 0.45:
            dist = self.rightside // 0.625
            self.add_segments(int(dist), "back")
        if self.leftside > 0.45:
            dist = self.leftside // 0.625
            self.add_segments(int(dist), "up")

    def _map_area(self):
        """Map visible environment based on current heading."""
        if -0.1 < self.heading < 0.1:
            self._forward()
        if 1.47 < self.heading < 1.67:
            self._left_turn()
        if -1.67 < self.heading < -1.47:
            self._right_turn()
        if self.heading > (math.pi - 0.1) or self.heading < (-math.pi + 0.1):
            if self.ahead > 0.45:
                dist = self.ahead // 0.625
                self.add_segments(int(dist), "back")
            if self.behind > 0.45:
                dist = self.behind // 0.625
                self.add_segments(int(dist), "up")
            if self.rightside > 0.45:
                dist = self.rightside // 0.625
                self.add_segments(int(dist), "left")
            if self.leftside > 0.45:
                dist = self.leftside // 0.625
                self.add_segments(int(dist), "right")

    def _bfs(self, start, goal):
        """Perform Breadth-First Search from start to goal in the map."""
        queue = deque()
        queue.append((start, [start]))
        visited = {start}
        while queue:
            current, path = queue.popleft()
            if current == goal:
                return path
            for neighbor in self.layout.get(current, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return []

    def _choose_frontier(self):
        """Select closest unmapped cell and compute path to it."""
        if not self.unknown:
            return
        min_dist = float('inf')
        nearest = None
        for cell in self.unknown:
            dist = abs(cell[0] - self.coords[0]) + abs(cell[1] - self.coords[1])
            if dist < min_dist:
                min_dist = dist
                nearest = cell
        route = self._bfs(self.coords, nearest)
        self.target = (nearest, route)
        if nearest in self.unknown:
            self.unknown.remove(nearest)

    def get_frontier_and_path(self) -> list:
        """Return the current target frontier and path to it."""
        return self.target

    def sense(self) -> None:
        """Update internal sensor data from robot."""
        self.heading = self.robot.get_orientation()
        self.lidar_readings = self.robot.get_lidar_range_list()
        self.coords = self.robot.get_current_position()
        if self.lidar_readings:
            self.ahead = self.lidar_readings[480]
            self.behind = self.lidar_readings[150]
            self.rightside = self.lidar_readings[1]
            self.leftside = self.lidar_readings[320]

    def plan(self) -> None:
        """Perform planning logic to decide next frontier."""
        self._map_area()
        self._choose_frontier()

    def act(self) -> None:
        """Execute the planned action (not implemented)."""
        pass

    def spin(self) -> None:
        """Run one full sense-plan-act cycle."""
        self.sense()
        self.plan()
        self.act()
