"""S3"""
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
        self.rotation_flips = 0

        self.left_ticks = 0
        self.right_ticks = 0

        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.prev_time = 0
        self.curr_time = 0

        self.lidar_data = None
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.object_1 = None
        self.object_2 = None
        self.in_search_mode = True
        self.reposition_cycles = 0
        self.left_speed = 0
        self.right_speed = 0

    def get_triangle_vertex_coordinates(self) -> tuple | None:
        """Look for sudden changes in LIDAR data that indicate cylinders."""
        self.detected_objects = []
        start_index = None

        threshold = 0.1
        object_size_min = 1

        if self.lidar_data is not None and len(self.lidar_data) > 1:
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

        if delta_time <= 0:
            return self.robot_x, self.robot_y, self.theta

        left_ticks = self.robot.get_left_motor_encoder_ticks()
        right_ticks = self.robot.get_right_motor_encoder_ticks()

        not_anymore_theta = self.theta

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

        if (not_anymore_theta > 0 and self.theta < 0) or (not_anymore_theta < 0 and self.theta > 0):
            self.rotation_flips = self.rotation_flips + 1
            print(f"The rotation has been flipped {self.rotation_flips} times.")

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
        self.get_robot_pose()
        if self.in_search_mode is True:
            self.lidar_data = self.robot.get_lidar_range_list()
            collect_result = self.get_triangle_vertex_coordinates()
            if collect_result:
                self.object_1, self.object_2 = collect_result
                self.in_search_mode = False
            elif self.rotation_flips >= 2:
                self.reposition_cycles = 50
                self.rotation_flips = 0
            else:
                print("sense: no objects found, continuing search...")


    def plan(self) -> None:
        """Plan the robot's actions.

        Process the data collected during sensing and decide the next course
        of action for the robot.
        """
        if self.reposition_cycles > 0:
            self.reposition_cycles -= 1
            self.left_speed = -0.5
            self.right_speed = -0.5
        elif self.in_search_mode:
            self.left_speed = -1
            self.right_speed = 1
            return

        valid_vertices = []

        if not self.object_1 or not self.object_2:
            return
        for valid_point in [self.object_1, self.object_2]:
            distance_x = valid_point[0] - self.robot_x
            distance_y = valid_point[1] - self.robot_y
            final_distance = math.sqrt(distance_x ** 2 + distance_y ** 2)
            if final_distance < 4.0:
                valid_vertices.append((valid_point, final_distance))
        if not valid_vertices:
            self.left_speed = 0
            self.right_speed = 0
            return
        nearest_vertex = min(valid_vertices, key=lambda x: x[1])  # closest vertex
        print(f"nearest vertex = {nearest_vertex}")
        final_distance = nearest_vertex[1]

        if final_distance > 0.1:
            distance_y = nearest_vertex[0][1]
            distance_x = nearest_vertex[0][0]
            goal = math.atan2(distance_x, distance_y)
            print("The goal angle is = ", goal)
            orientation_offset = goal - self.theta
            orientation_offset = (orientation_offset + math.pi) % (2 * math.pi) - math.pi

            if abs(orientation_offset) > 0.5:
                turn_sensitivity = 2.0
                turn_speed = max(0.3, min(1.0, abs(orientation_offset) * turn_sensitivity))
                self.left_speed = -turn_speed if orientation_offset > 0 else turn_speed
                self.right_speed = turn_speed if orientation_offset > 0 else -turn_speed
            else:
                movement_speed: float = 1.0 if final_distance > 0.5 else 0.5
                rotation_gain = 1.0
                rotation_adjustment = rotation_gain * orientation_offset
                self.left_speed = max(0, min(3, movement_speed - rotation_adjustment))
                self.right_speed = max(0, min(3, movement_speed + rotation_adjustment))
        else:
            self.left_speed = 0
            self.right_speed = 0


    def act(self) -> None:
        """Execute planned actions.

        Perform the actions decided in the planning step, such as moving or
        interacting with the environment.
        """
        self.robot.set_left_motor_velocity(self.left_speed)
        self.robot.set_right_motor_velocity(self.right_speed)

    def spin(self) -> None:
        """Spin the robot.

        This is the main loop, where the robot performs its sense-plan-act cycle.
        """
        self.sense()
        self.plan()
        self.act()