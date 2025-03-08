"""EX05: Triangle Forming."""
from __future__ import annotations
import math
import numpy as np


class Robot:
    """Turtlebot robot."""

    def __init__(self, robot: object) -> None:
        """Class initializer.

        Args:
            robot (object): An instance of a Turtlebot-like robot interface.
        """
        self.robot = robot
        self.detected_objects = []

        self.WHEEL_BASE = 0.233
        self.TRACK_WIDTH = self.WHEEL_BASE
        self.TICKS_PER_RADIANS = 508.8 / (2 * math.pi)
        self.WHEEL_RADIUS = 0.03575

        self.start_orientation = None
        self.theta = 0.0

        self.left_ticks = 0
        self.right_ticks = 0

        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.prev_time = 0
        self.curr_time = 0

        self.lidar_data = None
        self.robot_x = 0.0
        self.robot_y = 0.0

        # minu lisatud
        self.distance_to_target = None
        self.right_velocity = 0.5
        self.left_velocity = -0.5
        self.triangle_vertex = None
        self.target_x = None
        self.target_y = None
        self.target_angle = None
        self.turning_left = False
        self.turning_right = False
        self.moving_forward = False
        self.targeting_closest = True
        self.target_object = None


    def get_triangle_vertex_coordinates(self) -> tuple | None:
            """Look for sudden changes in LIDAR data that indicate cylinders."""
            self.detected_objects = []
            start_index = None

            threshold = 0.1
            object_size_min = 1

            if self.lidar_data is not None:
                for i in range(1, len(self.lidar_data)):
                    if self.lidar_data[i] == float('inf') or self.lidar_data[i - 1] == float('inf'):
                        start_index = None
                        continue

                    delta = self.lidar_data[i] - self.lidar_data[i - 1]

                    if start_index is None and abs(delta) > threshold and delta < 0:
                        start_index = i

                    elif start_index is not None and abs(delta) > threshold and delta > 0:
                        end_index = i - 1

                        if abs(end_index - start_index) >= object_size_min:
                            object_values = self.lidar_data[start_index:end_index]
                            distance = np.min(object_values)
                            index = np.argmin(object_values)
                            center_index = start_index + index

                            angle = (center_index / len(self.lidar_data)) * (2 * np.pi)
                            self.detected_objects.append((distance, angle))

                        start_index = None

            obj_coordinates_robot = []
            obj_coordinates_world = []
            for object in self.detected_objects:
                x__robot_position = -(object[0] * math.sin(object[1]))
                y_robot_position = -(object[0] * math.cos(object[1]))
                obj_coordinates_robot.append((x__robot_position, y_robot_position))

            for object in obj_coordinates_robot:
                x_world = (self.robot_x + object[0] * math.cos(self.theta) - object[1] * math.sin(self.theta))
                y_world = (self.robot_y + object[0] * math.sin(self.theta) + object[1] * math.cos(self.theta))
                obj_coordinates_world.append((x_world, y_world))

            if len(obj_coordinates_world) < 2:
                return None

            x1, y1 = obj_coordinates_world[0]
            x2, y2 = obj_coordinates_world[1]

            dx = (math.sqrt(3) / 2) * (y2 - y1)
            dy = (math.sqrt(3) / 2) * (x2 - x1)

            x_middle = (x1 + x2) / 2
            y_middle = (y1 + y2) / 2

            g_1 = (x_middle + dx, y_middle - dy)
            g_2 = (x_middle - dx, y_middle + dy)
            return g_1, g_2

    def get_robot_pose(self) -> tuple:
        """Return the current robot pose."""
        delta_time = self.curr_time - self.prev_time
        print("Delta Time:", delta_time)

        if delta_time <= 0:
            return self.robot_x, self.robot_y, self.theta

        left_ticks = self.robot.get_left_motor_encoder_ticks()
        right_ticks = self.robot.get_right_motor_encoder_ticks()

        delta_left_ticks = left_ticks - self.prev_left_ticks
        delta_right_ticks = right_ticks - self.prev_right_ticks

        left_velocity = (delta_left_ticks / self.TICKS_PER_RADIANS) / delta_time
        right_velocity = (delta_right_ticks / self.TICKS_PER_RADIANS) / delta_time

        linear_velocity = (self.WHEEL_RADIUS / 2) * (left_velocity + right_velocity)
        angular_velocity = (self.WHEEL_RADIUS / self.WHEEL_BASE) * (right_velocity - left_velocity)

        self.theta += angular_velocity * delta_time
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        self.robot_x += linear_velocity * math.cos(self.theta) * delta_time
        self.robot_y += linear_velocity * math.sin(self.theta) * delta_time

        self.prev_time = self.curr_time
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        return self.robot_x, self.robot_y, self.theta

    def sense(self) -> None:
        """Gather sensor data.

        Use the robot's sensors to collect data about its environment.
        This method updates internal state variables based on sensor readings.
        """
        self.lidar_data = self.robot.get_lidar_range_list()
        self.curr_time = self.robot.get_time()
        self.left_ticks = self.robot.get_left_motor_encoder_ticks()
        self.right_ticks = self.robot.get_right_motor_encoder_ticks()

        if self.start_orientation is None:
            self.start_orientation = self.robot.get_orientation()
        self.theta = self.robot.get_orientation() - self.start_orientation

    # minu lisatud
    def detect_triangle(self) -> None:
        # if self.lidar_data is None:
        #   return
        self.get_robot_pose()
        self.detect_triangle()

        points = []
        for i, distance in enumerate(self.lidar_data):
            angle = i * (2 * np.pi / len(self.lidar_data))
            x = self.robot_x + distance * np.cos(angle)
            y = self.robot_y + distance * np.sin(angle)
            points.append((x, y))

        points = sorted(points, key=lambda p: np.linalg.norm([p[0] - self.robot_x, p[1] - self.robot_y]))

        if len(points) >= 3:
            A, B, C = points[:3]

            d1 = np.linalg.norm(np.array(A) - np.array(B))
            d2 = np.linalg.norm(np.array(B) - np.array(C))
            d3 = np.linalg.norm(np.array(C) - np.array(A))

            if abs(d1 - d2) < 0.05 and abs(d2 - d3) < 0.05:
                self.triangle_vertex = C
                self.target_x, self.target_y = C
                print(f"Kolmnurga tipp leitud: {self.target_x}, {self.target_y}")

    # minu lisatud
    def plan(self) -> None:
        """Plan the robot's actions.

        Process the data collected during sensing and decide the next course
        of action for the robot.
        """
        self.get_robot_pose()
        self.detect_triangle()

        if self.target_x is None:
            self.detect_triangle()
            self.target_y = -self.target_y

        if self.target_x is None or -self.target_y is None:
            print("No target found. Waiting for detection...")
            self.moving_forward = False
            self.turning_left = True
            self.turning_right = False
            return self.detect_triangle()

        delta_x = self.target_x - self.robot_x
        delta_y = self.target_y - self.robot_y
        self.target_angle = math.atan2(delta_y, delta_x)

        if self.target_angle is None:
            self.target_angle = math.atan2(delta_y, delta_x)

        print(f"Target: ({self.target_x}, {self.target_y}), Angle: {self.target_angle}, Robot Angle: {self.theta}")

        angle_diff = (self.target_angle - self.theta + np.pi) % (2 * np.pi) - np.pi

        if abs(angle_diff) > 0.05 and self.distance_to_target is None:
            self.moving_forward = False
            self.turning_left = angle_diff > 0
            self.turning_right = not self.turning_left
            print(f"Rotating {'left' if self.turning_left else 'right'} to align with target.")
            return

        self.moving_forward = True
        self.turning_left = False
        self.turning_right = False
        print("Aligned with target. Moving forward!")

        print(
            f"Moving: {self.moving_forward} | Turning Left: {self.turning_left} | Turning Right: {self.turning_right}")

        self.distance(delta_x, delta_y)

        if self.turning_left:
            print("Turning left to align with target...")
            self.right_velocity = -0.5
            self.left_velocity = 0.5

        elif self.turning_right:
            print("Turning right to align with target...")
            # actis kutsutakse välja (self.left_velocity) ja (self.right_velocity) sisu välja, mis on real 167 ja 168
            self.left_velocity = 0.5
            self.right_velocity = -0.5

        elif self.moving_forward:
            if self.distance_to_target > 0.1:
                print("Moving forward!")
                self.left_velocity = 1.5
                self.right_velocity = 1.5
            else:
                self.left_velocity = 0
                self.right_velocity = 0


    # minu lisatud
    def calculate_target_angle(self):
        pass
    # minu lisatud
    def distance(self, delta_x=None, delta_y=None):
        # Roboti ja punkti kaugus
        self.distance_to_target = math.sqrt(delta_x ** 2 + delta_y ** 2)

        if abs(self.target_angle) < 0.01:
            self.moving_forward = True
        #  print(f"Moving straight ({self.target_angle})")
        if self.distance_to_target < 0.1:  # 0.1m = 10cm
            self.moving_forward = False
            self.turning_left = False
            self.turning_right = False
            print("Reached the target. Stopping.", flush=True)
        return math.sqrt((delta_x[0] - delta_y[0] ** 2 + (delta_x[1] - delta_y[1] ** 2)))

    def act(self) -> None:
        """Execute planned actions.

        Perform the actions decided in the planning step, such as moving or
        interacting with the environment.
        """
        # minu lisatud
        self.robot.set_left_motor_velocity(self.left_velocity)
        self.robot.set_right_motor_velocity(self.right_velocity)

    def spin(self) -> None:
        """Spin the robot.

        This is the main loop, where the robot performs its sense-plan-act cycle.
        """
        self.sense()
        self.plan()
        self.act()