from __future__ import annotations

import math
import numpy as np


class Robot:
    """Turtlebot robot."""

    def __init__(self, robot: object) -> None:
        """Class initializer."""
        self.robot = robot
        self.WHEEL_BASE = 0.233
        self.WHEEL_RADIUS = 0.03575
        self.TICKS_PER_RADIANS = 508.8 / (2 * math.pi)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.theta = 0.0

        self.previous_time = 0
        self.current_time = 0

        self.left_ticks = 0
        self.right_ticks = 0
        self.previous_left_ticks = 0
        self.previous_right_ticks = 0

        self.turning_left = False
        self.turning_right = False
        self.moving_forward = False
        self.targeting_closest = True
        self.detected_objects = []
        self.target_object = None

        self.lidar_data = None
        self.triangle_vertex = None
        self.target_x = None
        self.target_y = None
        self.target_angle = None

        self.right_velocity = 0.5
        self.left_velocity = -0.5

        self.distance_to_target = None

    def get_robot_pose(self) -> tuple:
        """Return the current robot pose."""
        delta_time = self.current_time - self.previous_time
        print("Delta Time:", delta_time)

        if delta_time <= 0:
            return self.robot_x, self.robot_y, self.theta

        left_ticks = self.robot.get_left_motor_encoder_ticks()
        right_ticks = self.robot.get_right_motor_encoder_ticks()

        delta_left_ticks = left_ticks - self.previous_left_ticks
        delta_right_ticks = right_ticks - self.previous_right_ticks

        left_velocity = (delta_left_ticks / self.TICKS_PER_RADIANS) / delta_time
        right_velocity = (delta_right_ticks / self.TICKS_PER_RADIANS) / delta_time

        linear_velocity = (self.WHEEL_RADIUS / 2) * (left_velocity + right_velocity)
        angular_velocity = (self.WHEEL_RADIUS / self.WHEEL_BASE) * (right_velocity - left_velocity)

        self.theta += angular_velocity * delta_time
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi

        self.robot_x += linear_velocity * math.cos(self.theta) * delta_time
        self.robot_y += linear_velocity * math.sin(self.theta) * delta_time

        self.previous_time = self.current_time
        self.previous_left_ticks = left_ticks
        self.previous_right_ticks = right_ticks

        return self.robot_x, self.robot_y, self.theta

    def sense(self) -> None:
        """Gather sensor data."""
        self.lidar_data = self.robot.get_lidar_range_list()
        self.current_time = self.robot.get_time()
        self.left_ticks = self.robot.get_left_motor_encoder_ticks()
        self.right_ticks = self.robot.get_right_motor_encoder_ticks()

    def detect_triangle(self) -> None:
        if self.lidar_data is None:
            return

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

    def plan(self) -> None:
        """Plan the robot's actions."""
        self.get_robot_pose()
        self.detect_triangle()

        if self.target_x is None or self.target_y is None:
            print("No target found. Waiting for detection...")
            self.moving_forward = False
            self.turning_left = True
            self.turning_right = False
            return

        delta_x = self.target_x - self.robot_x
        delta_y = self.target_y - self.robot_y
        self.target_angle = math.atan2(delta_y, delta_x)

        if self.target_angle is None:
            self.target_angle = math.atan2(delta_y, delta_x)

        print(f"Target: ({self.target_x}, {self.target_y}), Angle: {self.target_angle}, Robot Angle: {self.theta}")

        angle_diff = (self.target_angle - self.theta + np.pi) % (2 * np.pi) - np.pi

        if abs(angle_diff) > 0.05:
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

    # angle oli nullilähedane. Robot kaugus ja punkti kaugus väiksem kui 10cm
    # vaata videos kujutisi
    def distance(self, delta_x=None, delta_y=None):
        # Roboti ja punkti kaugus
        distance_to_target = math.sqrt(delta_x ** 2 + delta_y ** 2)

        if abs(self.target_angle) < 0.01:
            self.moving_forward = True
        # print(f"Moving straight ({self.target_angle}))
        if distance_to_target < 0.1:  # 0.1m = 10cm
            self.moving_forward = False
            self.turning_left = False
            self.turning_right = False
            print("Reached the target. Stopping.", flush=True)
            return


    def act(self) -> None:
        """Execute planned actions."""
        self.robot.set_left_motor_velocity(self.left_velocity)
        self.robot.set_right_motor_velocity(self.right_velocity)


    # print(
    #    f"Moving: {self.moving_forward} | Turning Left: {self.turning_left} | Turning Right: {self.turning_right}")

    # if self.turning_left:
    #   print("Turning left to align with target...")
    #  self.robot.set_left_motor_velocity(-0.5)
    # self.robot.set_right_motor_velocity(0.5)

    # elif self.turning_right:
    #   print("Turning right to align with target...")
    #  self.robot.set_left_motor_velocity(0.5)
    # self.robot.set_right_motor_velocity(-0.5)

    # elif self.moving_forward:
    #   print("Moving forward!")
    #  self.robot.set_right_motor_velocity(1.5)
    # self.robot.set_left_motor_velocity(1.5)


    # else:
    #   print("Stopping...")
    #  self.robot.set_right_motor_velocity(0)
    # self.robot.set_left_motor_velocity(0)


    def spin(self) -> None:
        """Spin the robot."""
        self.sense()
        self.plan()
        self.act()
