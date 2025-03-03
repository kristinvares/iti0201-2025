"""S1."""
import math


class Robot:
    """Turtlebot robot."""

    def __init__(self, robot: object) -> None:
        """Class initializer.

        Args:
            robot (object): An instance of a Turtlebot-like robot interface.
        """
        self.robot = robot
        self.lidar_data = []
        self.detected_objects = []
        self.angle = 0

        self.turning_left = False
        self.turning_right = False
        self.moving_forward = False

    def sense(self) -> None:
        """Gather sensor data.

        Use the robot's sensors to collect data about its environment.
        This method updates internal state variables based on sensor readings.
        """
        self.lidar_data = self.robot.get_lidar_range_list()

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
        self.turning_left = False
        self.turning_right = False
        self.moving_forward = False

        target_zone = 0.1

        for info in self.detected_objects:
            distance = info[0]
            # print(distance)
            angle = info[1]
            # print(angle)

            # If not facing the cylinder, rotate
            if angle > (math.pi / 2) and angle < ((3 * math.pi / 2) - target_zone):
                self.robot.set_right_motor_velocity(1.0)
                self.robot.set_left_motor_velocity(-1.0)
                self.turning_left = False
                self.turning_right = True
            else:
                self.robot.set_left_motor_velocity(1.0)
                self.robot.set_right_motor_velocity(-1.0)
                self.turning_right = False
                self.turning_left = True

            if abs(angle - (3 * math.pi / 2)) < target_zone:
                self.turning_right = False
                self.turning_left = False
                self.moving_forward = True

            if distance < 0.3:  # If too close, stop
                self.robot.set_right_motor_velocity(0)
                self.robot.set_left_motor_velocity(0)
        self.set_right_motor_torque()
        self.set_left_motor_torque()
        self.get_left_motor_encoder_ticks()

            # REALISTIC MODE
    def set_right_motor_torque(self, torque: float) -> None:
        torque_right = max(min(torque, 1.0), -1.0)
        print(torque_right)
        self.robot.right_motor.set_torque(torque)

    def set_left_motor_torque(self, torque: float) -> None:
        torque_left = max(min(torque, 1.0), -1.0)
        print(torque_left)
        self.robot.left_motor.set_torque(torque)

    def get_left_motor_encoder_ticks(self) -> int:
        return self.robot.left_motor.get_encoder_ticks()

    def get_right_motor_encoder_ticks(self) -> int:
        return self.robot.right_motor.get_encoder_ticks()

    def act(self) -> None:
        """Execute planned actions.

        Perform the actions decided in the planning step, such as moving or
        interacting with the environment.
        """
        if not self.turning_right and not self.turning_left and self.moving_forward:
            self.robot.set_right_motor_velocity(0.5)
            self.robot.set_left_motor_velocity(0.5)

        # REALISTIC MODE

    def spin(self) -> None:
        """Spin the robot.

        This is the main loop where the robot performs its sense-plan-act cycle.
        """
        self.sense()
        self.plan()
        self.act()