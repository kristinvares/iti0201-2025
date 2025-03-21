"""EX07: Object Detection."""
from __future__ import annotations
import numpy as np


class Robot:
    """Turtlebot robot."""

    def __init__(self, robot: object) -> None:
        """Class initializer.

        Args:
            robot (object): An instance of a Turtlebot-like robot interface.
        """
        self.robot = robot
        self.blue_cubes = None

    def get_object_bounding_box_list(self):
        x_min = 0
        x_max = 1
        y_min = 0
        y_max = 1
        # [(x_min, x_max, y_min, y_max), ...].
        return [(x_min, x_max, y_min, y_max)]

    def update_cube_objects(self):
        boxes = self.get_object_bounding_box_list()
        for box in boxes:
            x_min = box[0]
            x_max = box[1]
            y_min = box[2]
            y_max = box[3]
            width = x_max - x_min + 1
            height = y_max - y_min + 1
            aspect_ratio = width / height
            blue_channel_roi = self.image[x_min:x_max + 1, y_min:y_max + 1, 0]
            green_channel_roi = self.image[x_min:x_max + 1, y_min:y_max + 1, 1]
            red_channel_roi = self.image[x_min:x_max + 1, y_min:y_max + 1, 2]
            threshold = 20
            roi_mask = (blue_channel_roi > green_channel_roi + threshold) & (blue_channel_roi > red_channel_roi + threshold)
            blue_pixels_count = np.sum(roi_mask)
            total_pixel_count = width * height
            color_ratio = blue_pixels_count / total_pixel_count
            # seda peab täiendama, ise veel juurde kirjutama
            self.blue_cubes.append(box)



    def get_cube_objects(self) -> list | None:
        """Return the bounding boxes for detected objects.

        Image Data:
        - The image is of the form BGRA (height x width x 4).
        - The blue, green, and red channels correspond to the first, second, and third
          dimensions, respectively.
        - The alpha channel is included but not used for this detection.

        Bounding Box Data:
        - Each detected bounding box is returned as a tuple in the form
          `(x_min, x_max, y_min, y_max)`, where:
            - `x_min`: The minimum x-coordinate (left edge).
            - `x_max`: The maximum x-coordinate (right edge).
            - `y_min`: The minimum y-coordinate (top edge).
            - `y_max`: The maximum y-coordinate (bottom edge).

        Criteria for Detection:
        - Shape detection of correctly identifying the cube from the rest of objects.
        - Color detection of correctly identifying blue cubes from the rest of objects.

        Example:
          If an image contains a blue cube of size 100x100 pixels, the method will
          return a bounding box like:
          [(50, 150, 200, 300)]
          where:
            - The object spans from x=50 to x=150 horizontally.
            - The object spans from y=200 to y=300 vertically.

        Returns:
            [(x_min, x_max, y_min, y_max), ...]: A list of bounding boxes for
            detected cubes.
            Returns `None` if no cubes are detected.
        """
        return self.blue_cubes


    def sense(self) -> None:
        """Gather sensor data.

        Use the robot's sensors to collect data about its environment.
        This method updates internal state variables based on sensor readings.
        """
        self.image = self.robot.get_camera_rgb_image()
        self.update_cube_objects()

    def plan(self) -> None:
        """Plan the robot's actions.

        Process the data collected during sensing and decide the next course
        of action for the robot.
        """

    def act(self) -> None:
        """Execute planned actions.

        Perform the actions decided in the planning step, such as moving or
        interacting with the environment.
        """

    def spin(self) -> None:
        """Spin the robot.

        This is the main loop where the robot performs its sense-plan-act cycle.
        """
        self.sense()
        self.plan()
        self.act()

