"""S1."""


class Robot:
    """Turtlebot robot."""

    def __init__(self, robot: object) -> None:
        """Class initializer.

        Args:
            robot (object): An instance of a Turtlebot-like robot interface.
        """
        self.robot = robot
        self.lidar_data = None
        self.cylinder_angle = None
        self.front_distance = None

    def sense(self) -> None:
        """Gather sensor data.

        Use the robot's sensors to collect data about its environment.
        This method updates internal state variables based on sensor readings.
        """
        self.lidar_data = self.robot.get_lidar_range_list()

    def plan(self) -> None:
        """Plan the robot's actions.

        Process the data collected during sensing and decide the next course
        of action for the robot.
        """
        if self.lidar_data is not None:
            min_distance = min(self.lidar_data)
            min_index = self.lidar_data.index(min_distance)

            angle_per_step = 360 / len(self.lidar_data)
            self.cylinder_angle = (min_index * angle_per_step) - 90

            index = len(self.lidar_data) // 4
            self.front_distance = self.lidar_data[-index]

    def act(self) -> None:
        """Execute planned actions.

        Perform the actions decided in the planning step, such as moving or
        interacting with the environment.
        """
        if self.front_distance < 0.3:
            self.robot.set_left_motor_velocity(0)
            self.robot.set_right_motor_velocity(0)
            print("Reached the cylinder!")

        if -10 <= self.cylinder_angle <= 10:
            self.robot.set_left_motor_velocity(2.0)
            self.robot.set_right_motor_velocity(2.0)
        elif self.cylinder_angle > 10:
            self.robot.set_left_motor_velocity(1.5)
            self.robot.set_right_motor_velocity(2.5)
        else:
            self.robot.set_left_motor_velocity(2.5)
            self.robot.set_right_motor_velocity(1.5)

    def spin(self) -> None:
        """Spin the robot.

        This is the main loop where the robot performs its sense-plan-act cycle.
        """
        print("lala")
        self.sense()
        #self.plan()
        #self.act()