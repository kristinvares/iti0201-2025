import math
from collections import deque


class TurtleBot:
    def __init__(self, api):
        self.api = api
        self.traversed = [(0, 0)]
        self.unexplored = []
        self.map_data = {}
        self.scan = None
        self.angle = None
        self.coord = None
        self.path_to_frontier = None

    def collect_data(self):
        self.angle = self.api.get_orientation()
        self.scan = self.api.get_lidar_range_list()
        self.coord = self.api.get_current_position()
        if self.scan:
            self.forward = self.scan[480]
            self.backward = self.scan[160]
            self.left = self.scan[320]
            self.right = self.scan[1]

    def explore_direction(self, distance, side):
        x, y = self.coord
        directions = {
            "north": (0, 1),
            "south": (0, -1),
            "east": (1, 0),
            "west": (-1, 0)
        }

        if side not in directions:
            return

        dx, dy = directions[side]
        for i in range(1, int(distance) + 1):
            new = (x + dx * i, y + dy * i)

            if i == 1:
                self.map_data.setdefault(self.coord, []).append(new)
                self.map_data.setdefault(new, []).append(self.coord)

            if new not in self.traversed:
                self.traversed.append(new)
                if new not in self.unexplored:
                    self.unexplored.append(new)

    def build_map(self):
        if -0.1 < self.angle < 0.1:
            if self.forward > 0.45:
                self.explore_direction(self.forward // 0.625, "north")
            if self.backward > 0.45:
                self.explore_direction(self.backward // 0.625, "south")
        elif 1.47 < self.angle < 1.67:
            if self.forward > 0.45:
                self.explore_direction(self.forward // 0.625, "west")
        elif -1.67 < self.angle < -1.47:
            if self.forward > 0.45:
                self.explore_direction(self.forward // 0.625, "east")

    def bfs_path(self, start, goal):
        queue = deque([(start, [start])])
        visited = set([start])

        while queue:
            current, path = queue.popleft()
            if current == goal:
                return path

            for neighbor in self.map_data.get(current, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))

        return []

    def locate_frontier(self):
        if not self.unexplored:
            return

        closest = None
        shortest = float("inf")
        for cell in self.unexplored:
            dist = abs(cell[0] - self.coord[0]) + abs(cell[1] - self.coord[1])
            if dist < shortest:
                shortest = dist
                closest = cell

        route = self.bfs_path(self.coord, closest)
        self.path_to_frontier = (closest, route)

    def get_traversable_cells(self):
        return self.traversed

    def get_unmapped_cells(self):
        return self.unexplored

    def get_map(self):
        return self.map_data

    def get_frontier_and_path(self):
        return self.path_to_frontier

    def sense(self):
        self.collect_data()

    def plan(self):
        self.build_map()
        self.locate_frontier()

    def act(self):
        pass

    def spin(self):
        self.sense()
        self.plan()
        self.act()
