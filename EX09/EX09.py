"""EX09."""

import math

class Robot:
    """Turtlebot robot."""

    CELL_SIZE = 0.615

    def __init__(self, robot: object) -> None:
        """Class initializer.

        Args:
            robot (object): An instance of a Turtlebot-like robot interface.
        """
        self.robot = robot
        self.time = 0.0
        self.orientation = 0.0
        self.current_frontier_and_path = None
        self.lidar = None

        self.known_cells = []
        self.env_graph = {}
        self.mapped_cells = {}
        self.x = 0
        self.y = 0
        self.i = 0


    def get_frontier_and_path(self) -> list:
        """Identify next frontier for exploration and calculate the path to reach it.

        The frontier is the boundary between the known (mapped) and unknown (unmapped)
        regions of the map. This method determines the most suitable frontier to
        explore next and computes the path from the robot's current position to that
        frontier. Formula for choosing the next frontier to explore: Manhattan distance

        Returns:
            [(int, int), [(int, int), ...]]:
            - The first element is a tuple (x, y) representing the coordinates of the
              selected frontier.
            - The second element is a list of tuples [(x1, y1), (x2, y2), ...]
              representing the sequence of grid cells (coordinates) the robot should
              traverse in order to reach the frontier.

        Example:
            Suppose the robot's current position is (0, 0), and it detects a frontier
            at (3, 0). The function might return:
                [(3, 0), [(0, 0), (1, 0), (2, 0), (3, 0)]]
            This means the robot should travel through the listed cells to reach the
            frontier at (3, 0).
        """
        return self.current_frontier_and_path

    def _orientation_to_direction(self) -> str:
        """Convert the robot's current orientation angle to a cardinal direction."""
        angle = self.facing_angle
        margin = 0.05
        if abs(angle) < margin:
            return "up"
        elif abs(angle - math.pi / 2) < margin:
            return "left"
        elif abs(angle - math.pi) < margin or abs(angle + math.pi) < margin:
            return "down"
        elif abs(angle + math.pi / 2) < margin:
            return "right"
        return "unknown"

    def get_traversable_cells(self) -> list:
        """Return a list of all known traversable cells in the map."""
        return list(set(self.known_cells))

    def get_map(self) -> dict:
        """Get the map representation as a dictionary of adjacency."""
        return self.env_graph

    def _resolve_lidar_index(self, current: str, target: str, base_indices: dict, dir_labels: list) -> int:
        """Compute correct LIDAR index for a given relative direction."""
        current_idx = dir_labels.index(current)
        relative_idx = (dir_labels.index(target) - current_idx) % 4
        return list(base_indices.values())[relative_idx]

    def _generate_visible_coordinates(self, x: int, y: int, visibility: dict) -> list:
        """Return a list of visible grid cell coordinates from current location."""
        result = []
        for i in range(visibility["up"]):
            result.append((x, y + i + 1))
        for i in range(visibility["left"]):
            result.append((x - i - 1, y))
        for i in range(visibility["down"]):
            result.append((x, y - i - 1))
        for i in range(visibility["right"]):
            result.append((x + i + 1, y))
        return result

    def _add_to_graph(self, x: int, y: int, visibility: dict) -> None:
        """Add the current cell and its adjacent cells to the map graph."""
        neighbors = []
        if visibility["up"]:
            neighbors.append((x, y + 1))
        if visibility["left"]:
            neighbors.append((x - 1, y))
        if visibility["down"]:
            neighbors.append((x, y - 1))
        if visibility["right"]:
            neighbors.append((x + 1, y))

        self.env_graph[(x, y)] = neighbors
        self.mapped_cells[(x, y)] = neighbors

        for neighbor in neighbors:
            self.env_graph.setdefault(neighbor, [])
            if (x, y) not in self.env_graph[neighbor]:
                self.env_graph[neighbor].append((x, y))

    def _compute_cell_visibility(self) -> dict:
        """Return the number of visible cells in each direction based on LIDAR."""
        cardinal_directions = ["up", "left", "down", "right"]
        lidar_reference_indices = {"up": 480, "left": 320, "down": 160, "right": 639}
        current_facing = self._orientation_to_direction()
        visibility = {}

        for direction in cardinal_directions:
            idx = self._resolve_lidar_index(current_facing, direction, lidar_reference_indices, cardinal_directions)
            segment = self.lidar[idx - 10: idx + 10]
            valid_readings = [d for d in segment if not math.isinf(d)]
            max_distance = max(valid_readings, default=0.0)
            visibility[direction] = int(max_distance / self.CELL_SIZE)
        return visibility

    def get_unmapped_cells(self) -> list:
        """Return a list of all unmapped cells discovered so far."""
        self.lidar = self.robot.get_lidar_range_list()
        if not self.lidar:
            return []

        self.facing_angle = self.robot.get_orientation()

        directions = self._compute_cell_visibility()
        visible_cells = self._generate_visible_coordinates(self.x, self.y, directions)

        self.known_cells.extend(visible_cells)
        self.known_cells.append((self.x, self.y))

        self._add_to_graph(self.x, self.y, directions)

        frontiers = [cell for cell in self.known_cells if self._is_frontier_cell(cell)]
        return frontiers

    def _manhattan_distance(self, start, end):
        """Calculates the Manhattan distance between two grid cells."""
        return abs(start[0] - end[0]) + abs(start[1] - end[1])

    def find_path(self, start, end):
        """Finds the shortest path between two cells using A* algorithm."""
        if start not in self.env_graph or end not in self.env_graph:
            return None

        open_set = {start}
        closed_set = set()
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._manhattan_distance(start, end)}

        while open_set:
            if self.i == 20:  # testing
                return
            current = min(open_set, key=lambda node: f_score.get(node, float('inf')))
            print(f"Finding path from {current} to {end}")
            print(f"Open set: {open_set}")
            print(f"Closed set: {closed_set}")
            print(f"Came from: {came_from}")
            print(current, end)
            if current == end:
                print("Finished with finding path")
                print("")
                return self._reconstruct_path(came_from, current)

            open_set.remove(current)
            closed_set.add(current)
            self.i += 1  # testing

            for neighbor in self.env_graph.get(current, []):
                if neighbor in closed_set:
                    continue

                tentative_g_score = g_score[current] + 1

                if neighbor not in open_set or tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self._manhattan_distance(neighbor, end)
                    open_set.add(neighbor)

        return None

    def _reconstruct_path(self, came_from, current):
        """Reconstructs the path from the came_from dictionary."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def find_frontier_and_path(self) -> None:
        frontiers = self.get_unmapped_cells()
        current_grid_x = int(round(self.x / self.CELL_SIZE))
        current_grid_y = int(round(self.y / self.CELL_SIZE))
        current_position = (current_grid_x, current_grid_y)

        if not frontiers:
            self.current_frontier_and_path = [None, None]
            return

        best_frontier = None
        shortest_path = None
        min_distance = float('inf')
        for frontier in frontiers:
            path = self.find_path(current_position, frontier)
            if path:
                distance = len(path) - 1
                if distance < min_distance:
                    min_distance = distance
                    best_frontier = frontier
                    shortest_path = path
            elif best_frontier is None:
                best_frontier = frontier
                shortest_path = []

        self.current_frontier_and_path = [best_frontier, shortest_path]

    def _is_frontier_cell(self, cell):
        """Check if the given cell is a frontier cell (i.e., it has at least one unknown neighbor)."""
        x, y = cell
        neighbors = [(x, y + 1), (x, y - 1), (x + 1, y), (x - 1, y)]
        for n in neighbors:
            if n not in self.known_cells:
                return True
        return False

    def sense(self) -> None:
        """Gather sensor data.

        Use the robot's sensors to collect data about its environment.
        This method updates internal state variables based on sensor readings.
        """
        self.time = self.robot.get_time()
        self.orientation = self.robot.get_orientation()
        self.find_frontier_and_path()


    def plan(self) -> None:
        """Plan the robot's actions.

        Process the data collected during sensing and decide the next course
        of action for the robot.
        """
        pass

    def act(self) -> None:
        """Execute planned actions.

        Perform the actions decided in the planning step, such as moving or
        interacting with the environment.
        """
        pass

    def spin(self) -> None:
        """Spin the robot.

        This is the main loop where the robot performs its sense-plan-act cycle.
        """
        self.sense()
        self.plan()
        self.act()