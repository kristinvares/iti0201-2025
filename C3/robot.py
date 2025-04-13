from __future__ import annotations
import numpy as np
import math


class Robot:
    def __init__(self, robot: object) -> None:
        self.robot = robot
        self.camera_image = None
        self.fov = None
        self.angle = None
        self.distance = None
        self.left_velocity = 0.0
        self.right_velocity = 0.0

    def spin(self) -> None:
        self.sense()
        self.plan()
        self.act()

    def sense(self) -> None:
        self.camera_image = self.robot.get_camera_rgb_image()
        self.fov = self.robot.get_camera_field_of_view()

        if self.camera_image is not None:
            cube = self.detect_blue_cube()
            if cube:
                self.angle, self.distance = cube
            else:
                self.angle = None
                self.distance = None

    def plan(self) -> None:
        if self.angle is None or self.distance is None:
            self.left_velocity = -0.5
            self.right_velocity = 0.5
            return

        if abs(self.angle) > 0.1:
            self.left_velocity = 0.3 if self.angle > 0 else -0.3
            self.right_velocity = -self.left_velocity
        elif self.distance > 0.2:
            self.left_velocity = 1.0
            self.right_velocity = 1.0
        else:
            self.left_velocity = 0.0
            self.right_velocity = 0.0

    def act(self) -> None:
        self.robot.set_left_motor_velocity(self.left_velocity)
        self.robot.set_right_motor_velocity(self.right_velocity)

    def detect_blue_cube(self) -> tuple | None:
        image = self.camera_image
        if image is None:
            return None


        blue = image[:, :, 0].astype(float)
        green = image[:, :, 1].astype(float)
        red = image[:, :, 2].astype(float)

        mask = (blue > 100) & ((green + red) < 150)
        if not np.any(mask):
            return None

        coords = np.column_stack(np.where(mask))
        y_center, x_center = np.mean(coords, axis=0)

        width = image.shape[1]
        angle = ((x_center - width / 2) / (width / 2)) * (self.fov / 2)

        height = image.shape[0]
        y_from_bottom = height - y_center
        estimated_distance = y_from_bottom * 0.006

        return angle, estimated_distance
