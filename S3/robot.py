"""S3."""
from __future__ import annotations
import math
import numpy as np



#from S1 import turtlebot


class Robot:
    """Turtlebot robot."""

    def __init__(self, robot: object) -> None:
        """Class initializer.

        Args:
            robot (object): An instance of a Turtlebot-like robot interface.
        """
        self.robot = robot
        self.WHEEL_BASE = 0.233
        self.TRACK_WIDTH = self.WHEEL_BASE
        self.TICKS_PER_RADIANS = 508.8 / (2 * math.pi)
        self.WHEEL_RADIUS = 0.03575

        self.previous_x = 0.0
        self.previous_y = 0.0

        self.start_orientation = None
        self.theta = 0.0

        self.left_ticks = 0
        self.right_ticks = 0

        self.previous_left_ticks = 0
        self.previous_right_ticks = 0
        self.previous_time = 0
        self.current_time = 0

        self.lidar_data = None
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.turning_left = False
        self.turning_right = False
        self.moving_forward = False
        self.targeting_closest = True
        self.detected_objects = []
        self.target_object = None


    def get_robot_pose(self) -> tuple:
        """Return the current robot pose."""
        delta_time = self.current_time - self.previous_time
        print(delta_time)

        if delta_time <= 0:
             return self.robot_x, self.robot_y, self.theta

        left_ticks = self.robot.get_left_motor_encoder_ticks()
        right_ticks = self.robot.get_right_motor_encoder_ticks()
        print(right_ticks)

        delta_left_ticks = left_ticks - self.previous_left_ticks
        delta_right_ticks = right_ticks - self.previous_right_ticks
        print(delta_right_ticks)
    #
    #     left_velocity = (delta_left_ticks / self.TICKS_PER_RADIANS) / delta_time
    #     right_velocity = (delta_right_ticks / self.TICKS_PER_RADIANS) / delta_time
    #
    #     linear_velocity = (self.WHEEL_RADIUS / 2) * (left_velocity + right_velocity)
    #     angular_velocity = (self.WHEEL_RADIUS / self.WHEEL_BASE) * (right_velocity - left_velocity)
    #
    #     self.theta += angular_velocity * delta_time
    #     self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi
    #
    #     self.robot_x += linear_velocity * math.cos(self.theta) * delta_time
    #     self.robot_y += linear_velocity * math.sin(self.theta) * delta_time
    #
    #     self.previous_time = self.current_time
    #     self.previous_left_ticks = left_ticks
    #     self.previous_right_ticks = right_ticks
    #
    #     return self.robot_x, self.robot_y, self.theta
    #
    def sense(self) -> None:
         """Gather sensor data.

         Use the robot's sensors to collect data about its environment.
         This method updates internal state variables based on sensor readings.
         """
         self.lidar_data = self.robot.get_lidar_range_list()
         self.current_time = self.robot.get_time()
         self.left_ticks = self.robot.get_left_motor_encoder_ticks()
         self.right_ticks = self.robot.get_right_motor_encoder_ticks()
    #
        # if self.start_orientation is None:
         #    self.start_orientation = self.robot.get_orientation()
         #self.theta = self.robot.get_orientation() - self.start_orientation

    def plan(self) -> None:
        """Plan the robot's actions.

        Process the data collected during sensing and decide the next course
        of action for the robot.
        """
    #     self.turning_left = False
    #     self.turning_right = False
    #     self.moving_forward = False
    #
    #     target_x = 1.0
    #     target_y = 1.0
    #
    #     delta_x = target_x - self.robot_x
    #     delta_y = target_y - self.robot_y
    #     distance = math.sqrt(delta_x ** 2 + delta_y ** 2)
    #     print(distance)
    #     target_angle = math.atan2(delta_y, delta_x)
    #     print(target_angle)
        self.get_robot_pose()

    def act(self) -> None:
        """Execute planned actions.

        Perform the actions decided in the planning step, such as moving or
        interacting with the environment.
        """
    #
    #     print(
    #         f" Moving: {self.moving_forward} | Turning Left: {self.turning_left} |  Turning Right: {self.turning_right}")
    #
    #     if self.moving_forward and not self.turning_right and not self.turning_left:
    #         print("Moving forward!")
    #         self.robot.set_right_motor_velocity(2.0)
    #         self.robot.set_left_motor_velocity(2.0)
    #     else:
    #         print(" Still adjusting (turning or correcting angle)...")

    def spin(self) -> None:
        """Spin the robot.

        This is the main loop where the robot performs its sense-plan-act cycle.
        """

        self.sense()
        self.plan()
        self.act()

