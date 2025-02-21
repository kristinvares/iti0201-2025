"""EX04: PID Control."""
import time

class Robot:
    """Turtlebot robot."""

    def __init__(self, robot: object) -> None:
        """Class initializer.

        Args:
            robot (object): An instance of a Turtlebot-like robot interface.
        """
        self.robot = robot

        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.05

        self.previous_time = time.time()
        self.previous_left_error = 0
        self.previous_right_error = 0
        self.left_error_sum = 0
        self.right_error_sum = 0
        self.left_wheel_speed = 0
        self.right_wheel_speed = 0


    def set_pid(self, kp: float = 1.0, ki: float = 0.1, kd: float = 0.05) -> None:
        """Set the PID controller gains for the robot's wheel speed control.

        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_target_speeds(self, left_target: float, right_target: float) -> None:
        """Set the target speeds for the robot's wheels.

        Args:
            left_target (float): Target speed for the left wheel.
            right_target (float): Target speed for the right wheel.
        """
        self.left_target = left_target
        self.right_target = right_target

    def update_left_wheel_speed(self) -> None:
        """Update left wheel speed using PID control."""
        pass

    def update_right_wheel_speed(self) -> None:
        """Update right wheel speed using PID control."""
        pass

    def get_pid_corrected_left_wheel_speed(self) -> float:
        """Return the corrected left wheel speed."""
        return self.left_wheel_speed

    def get_pid_corrected_right_wheel_speed(self) -> float:
        """Return the corrected right wheel speed."""
        return self.right_wheel_speed

    def sense(self) -> None:
        """Gather sensor data."""
        current_time = time.time()
        delta_time = max(current_time - self.previous_time, 0.001)

        current_left_speed = self.left_wheel_speed
        current_right_speed = self.right_wheel_speed

        # Vasaku ratta PID arvutused
        left_error = self.left_target - current_left_speed
        self.left_error_sum += left_error * delta_time
        self.left_error_sum = max(min(self.left_error_sum, 100), -100)  # Piira vigade summeerimist

        left_derivative = (left_error - self.previous_left_error) / delta_time
        left_correction = (self.kp * left_error +
                           self.ki * self.left_error_sum +
                           self.kd * left_derivative)

        # Parema ratta PID arvutused
        right_error = self.right_target - current_right_speed
        self.right_error_sum += right_error * delta_time
        self.right_error_sum = max(min(self.right_error_sum, 100), -100)  # Piira vigade summeerimist

        right_derivative = (right_error - self.previous_right_error) / delta_time
        right_correction = (self.kp * right_error +
                            self.ki * self.right_error_sum +
                            self.kd * right_derivative)

        # Uuenda rataste kiiruseid
        self.left_wheel_speed += left_correction
        self.right_wheel_speed += right_correction

        # Salvesta praegused vead ja aeg järgmise iteratsiooni jaoks
        self.previous_left_error = left_error
        self.previous_right_error = right_error
        self.previous_time = current_time
        print(
            f"delta_time: {delta_time}, left_error: {self.previous_left_error}, left_wheel_speed: {self.left_wheel_speed}")

    def plan(self) -> None:
        """Plan robot actions."""
        pass

    def act(self) -> None:
        """Execute planned actions."""
        pass
    def spin(self) -> None:
        """Spin the robot."""
        self.sense()
        self.plan()
        self.act()

if __name__ == "__main__":
    pass