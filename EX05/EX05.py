"""EX05: Triangle Forming."""
from __future__ import annotations
import math



def get_orientation(self) ->int:
        return 0


class Robot:
    """Turtlebot robot."""

    def __init__(self, robot: object) -> None:
        """Class initializer.

        Args:
            robot (object): An instance of a Turtlebot-like robot interface.
        """
        self.robot = robot
        self.lidar_data = []
        self.angle = 0
        self.detected_objects = []

        self.left_ticks = 0
        self.previous_left_ticks = 0
        self.delta_left_ticks = 0
        self.right_ticks = 0
        self.previous_right_ticks = 0
        self.delta_right_ticks = 0

        self.delta_time = 0
        self.previous_time = 0

        self.left_angular_velocity = 0
        self.right_angular_velocity = 0

        self.theta = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0

    def get_triangle_vertex_coordinates(self) -> tuple | None:
        """Return the triangle corner coordinates.

        Based on lidar range list and current robot position, calculate the world
        position of the equilateral triangle corner, and return coordinates of x, y.

        Logic:
        - This method uses lidar data to find the two objects that form the base of
          triangle (vertex)
        - Based on the found objects transform them to world frame coordinates and
          calculate triangle corner coordinates.
        - The robot's orientation and position are used to compute the actual world
          coordinates of the corner.

        Returns:
            A tuple representing the (x, y) world coordinates of the triangle's corner.
            Returns `None` if no valid triangle corner can be detected.
        """
        if len(self.detected_objects) < 2:
            return None

        object_positions = []
        for distance, angle in self.detected_objects[:2]:
            x_r = distance * math.cos(angle)
            y_r = distance * math.sin(angle)
            x_w = self.robot_x + x_r * math.cos(self.theta) - y_r * math.sin(self.theta)
            y_w = self.robot_y + x_r * math.sin(self.theta) + y_r * math.cos(self.theta)
            object_positions.append((x_w, y_w))

        (x1, y1), (x2, y2) = object_positions

        # Compute equilateral triangle's third vertex
        Mx = (x1 + x2) / 2
        My = (y1 + y2) / 2
        a = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        h = (math.sqrt(3) / 2) * a

        dx = x2 - x1
        dy = y2 - y1

        # One possible vertex
        x3a = Mx - h * (dy / a)
        y3a = My + h * (dx / a)
        # Other possible vertex
        x3b = Mx + h * (dy / a)
        y3b = My - h * (dx / a)

        # Choose the vertex in front of the robot
        # vertex = (x3a, y3a) if x3a > self.robot_x else (x3b, y3b)
        return ((x3a, y3a), (x3b, y3b))


    def get_robot_pose(self) -> tuple:
        """Return the current robot pose.

        Return the robot's pose as a tuple, based on wheel encoders and odometry.

        Returns:
            A tuple representing the (x, y, theta) robot's pose. Theta is the angle
            between robot's starting direction and its current direction.
        """
        WHEEL_RADIUS = self.robot.WHEEL_DIAMETER / 2  # meters
        TICKS_PER_RADIANS = 508.8 / (2 * math.pi)  # ticks/rad
        TRACK_WIDTH = self.robot.TRACK_WIDTH

        # x_velocity = (WHEEL_RADIUS / 2) * (self.left_angular_velocity + self.right_angular_velocity) * math.cos(self.theta)
        # y_velocity = (WHEEL_RADIUS / 2) * (self.left_angular_velocity + self.right_angular_velocity) * math.sin(self.theta)
        # theta_velocity = (WHEEL_RADIUS / TRACK_WIDTH) * (self.right_angular_velocity - self.left_angular_velocity) * self.delta_time

        self.delta_time = self.robot.get_time() - self.previous_time  # seconds
        self.previous_time = self.robot.get_time()

        self.previous_left_ticks = self.left_ticks
        self.left_ticks = self.robot.get_left_motor_encoder_ticks()

        self.previous_right_ticks = self.right_ticks
        self.right_ticks = self.robot.get_right_motor_encoder_ticks()

        if self.delta_time <= 0:
            self.left_angular_velocity = 0.0  # rad/s
            self.right_angular_velocity = 0.0  # rad/s
            self.theta = 0.0
        else:
            self.delta_left_ticks = self.left_ticks - self.previous_left_ticks
            self.left_angular_velocity = (self.delta_left_ticks / TICKS_PER_RADIANS) / self.delta_time  # rad/s
            self.delta_right_ticks = self.right_ticks - self.previous_right_ticks
            self.right_angular_velocity = (self.delta_right_ticks / TICKS_PER_RADIANS) / self.delta_time  # rad/s
            # self.theta = (WHEEL_RADIUS / TRACK_WIDTH) * (self.right_angular_velocity - self.left_angular_velocity) * self.delta_time

            # Compute linear and angular velocity
            # tema kasutas orientation funktsiooni -> velocity = (WHEEL_RADIUS / 2) * (self.left_angular_velocity + self.right_angular_velocity)
            omega = (WHEEL_RADIUS / TRACK_WIDTH) * (self.right_angular_velocity - self.left_angular_velocity)

            # Update pose
            delta_theta = omega * self.delta_time
            self.theta += delta_theta
            #self.robot_x += velocity * self.delta_time * math.cos(self.theta)
            #self.robot_y += velocity * self.delta_time * math.sin(self.theta)
            theta = get_orientation()

        # self.robot_x = self.robot_x + x_velocity * self.delta_time
        # self.robot_y = self.robot_y + y_velocity * self.delta_time

        return (self.robot_x, self.robot_y, self.theta)

    def convert_to_world_frame(self) -> tuple:
        """Convert robot frame coordinates to world frame coordinates."""

    def sense(self) -> None:
        """Gather sensor data.

        Use the robot's sensors to collect data about its environment.
        This method updates internal state variables based on sensor readings.
        """
        self.lidar_data = self.robot.get_lidar_range_list()

        if not self.lidar_data:
            self.detected_objects = []
            return None

        all_elements = len(self.lidar_data)
        self.angle = (2 * math.pi) / all_elements
        self.detected_objects = []

        index = 1
        while index < all_elements - 1:
            prev_dist = self.lidar_data[index - 1]
            curr_dist = self.lidar_data[index]
            next_dist = self.lidar_data[index + 1]

            if math.isinf(prev_dist) or math.isinf(curr_dist) or math.isinf(next_dist):
                index += 1
                continue

            if curr_dist < prev_dist * 0.9:
                object_start = index

                object_valid = True
                object_points = [curr_dist]

                while index < all_elements - 1:
                    curr_dist = self.lidar_data[index]
                    next_dist = self.lidar_data[index + 1]

                    if math.isinf(curr_dist) or math.isinf(next_dist):
                        object_valid = False
                        break

                    object_points.append(curr_dist)

                    if next_dist > curr_dist * 1.1:
                        object_end = index

                        if object_valid and len(object_points) > 2:
                            center_index = (object_start + object_end) // 2
                            center_distance = self.lidar_data[center_index]
                            center_angle = center_index * self.angle
                            self.detected_objects.append((center_distance, center_angle))
                            # print(self.detected_objects)
                        break

                    index += 1
            index += 1

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