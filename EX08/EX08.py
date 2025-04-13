"""EX08: Mapping the Environment."""

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
        self.lidar_snapshot = None
        self.facing_angle = 0.0

        self.known_cells = []
        self.env_graph = {}
        self.mapped_cells = {}

    def get_unmapped_cells(self) -> list:
        """Return a list of all unmapped cells discovered so far."""
        return [cell for cell in self.known_cells if cell not in self.mapped_cells]

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
        """Returns list of visible grid cell coordinates from current location."""
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
        """Adds the current cell and its adjacent cells to the map graph."""
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
        """Returns the number of visible cells in each direction based on LIDAR."""
        cardinal_directions = ["up", "left", "down", "right"]
        lidar_reference_indices = {"up": 480, "left": 320, "down": 160, "right": 639}
        current_facing = self._orientation_to_direction()
        visibility = {}

        for direction in cardinal_directions:
            idx = self._resolve_lidar_index(current_facing, direction, lidar_reference_indices, cardinal_directions)
            segment = self.lidar_snapshot[idx - 10: idx + 10]
            valid_readings = [d for d in segment if not math.isinf(d)]
            max_distance = max(valid_readings, default=0.0)
            visibility[direction] = int(max_distance / self.CELL_SIZE)
        return visibility

    def sense(self) -> None:
        """Run one sense-plan-act cycle for the robot."""
        self.lidar_snapshot = self.robot.get_lidar_range_list()
        if not self.lidar_snapshot:
            return

        x, y = self.robot.get_current_position()
        self.facing_angle = self.robot.get_orientation()

        directions = self._compute_cell_visibility()
        visible_cells = self._generate_visible_coordinates(x, y, directions)

        self.known_cells.extend(visible_cells)
        self.known_cells.append((x, y))

        self._add_to_graph(x, y, directions)


    def plan(self) -> None:
        """Plan the robot's actions."""
        pass

    def act(self) -> None:
        """Execute planned actions."""
        pass

    def spin(self) -> None:
        """Spin the robot."""
        self.sense()
        self.plan()
        self.act()
