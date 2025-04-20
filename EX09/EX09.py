import math
from collections import deque


class TurtleBot:
    def __init__(self, api):
        self.api = api
        self.seen_cells = [(0, 0)]
        self.pending_cells = []
        self.world_map = {}
        self.lidar_scan = None
        self.direction = None
        self.location = None
        self.target_info = None

    def gather_data(self):
        self.direction = self.api.get_orientation()
        self.lidar_scan = self.api.get_lidar_range_list()
        self.location = self.api.get_current_position()
        if self.lidar_scan:
            self.dist_front = self.lidar_scan[480]
            self.dist_back = self.lidar_scan[150]
            self.dist_right = self.lidar_scan[1]
            self.dist_left = self.lidar_scan[320]

    def get_visible_area(self, dist, heading):
        px, py = self.location
        offsets = {
            "north": (0, 1),
            "south": (0, -1),
            "east": (1, 0),
            "west": (-1, 0)
        }

        dx, dy = offsets[heading]
        steps = []

        for step in range(1, int(dist) + 1):
            coord = (px + dx * step, py + dy * step)
            if step == 1:
                self.world_map.setdefault(self.location, []).append(coord)
                self.world_map.setdefault(coord, []).append(self.location)

            if coord not in self.seen_cells:
                self.seen_cells.append(coord)
                self.pending_cells.append(coord)

    def orientation_case(self):
        if -0.1 < self.direction < 0.1:
            if self.dist_front > 0.45:
                self.get_visible_area(self.dist_front // 0.625, "north")
            if self.dist_back > 0.45:
                self.get_visible_area(self.dist_back // 0.625, "south")
            if self.dist_right > 0.45:
                self.get_visible_area(self.dist_right // 0.625, "east")
            if self.dist_left > 0.45:
                self.get_visible_area(self.dist_left // 0.625, "west")

        elif 1.47 < self.direction < 1.67:
            if self.dist_front > 0.45:
                self.get_visible_area(self.dist_front // 0.625, "west")
            if self.dist_back > 0.45:
                self.get_visible_area(self.dist_back // 0.625, "east")
            if self.dist_right > 0.45:
                self.get_visible_area(self.dist_right // 0.625, "north")
            if self.dist_left > 0.45:
                self.get_visible_area(self.dist_left // 0.625, "south")

        elif -1.67 < self.direction < -1.47:
            if self.dist_front > 0.45:
                self.get_visible_area(self.dist_front // 0.625, "east")
            if self.dist_back > 0.45:
                self.get_visible_area(self.dist_back // 0.625, "west")
            if self.dist_right > 0.45:
                self.get_visible_area(self.dist_right // 0.625, "south")
            if self.dist_left > 0.45:
                self.get_visible_area(self.dist_left // 0.625, "north")

        elif self.direction > math.pi - 0.1 or self.direction < -math.pi + 0.1:
            if self.dist_front > 0.45:
                self.get_visible_area(self.dist_front // 0.625, "south")
            if self.dist_back > 0.45:
                self.get_visible_area(self.dist_back // 0.625, "north")
            if self.dist_right > 0.45:
                self.get_visible_area(self.dist_right // 0.625, "west")
            if self.dist_left > 0.45:
                self.get_visible_area(self.dist_left // 0.625, "east")

    def explore(self):
        if not self.pending_cells:
            return

        px, py = self.location
        closest = None
        shortest = float('inf')

        for cell in self.pending_cells:
            dist = abs(cell[0] - px) + abs(cell[1] - py)
            if dist < shortest:
                shortest = dist
                closest = cell

        route = self._bfs(self.location, closest)
        self.target_info = (closest, route)

        if closest in self.pending_cells:
            self.pending_cells.remove(closest)

    def _bfs(self, source, target):
        queue = deque([(source, [source])])
        visited = set([source])

        while queue:
            current, path = queue.popleft()
            if current == target:
                return path

            for neighbor in self.world_map.get(current, []):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))

        return []

    def get_traversable_cells(self):
        return self.seen_cells

    def get_unmapped_cells(self):
        return self.pending_cells

    def get_map(self):
        return self.world_map

    def get_frontier_and_path(self):
        return self.target_info

    def sense(self):
        self.gather_data()

    def plan(self):
        self.orientation_case()
        self.explore()

    def act(self):
        pass

    def spin(self):
        self.sense()
        self.plan()
        self.act()
