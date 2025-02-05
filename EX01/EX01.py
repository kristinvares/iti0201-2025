"""EX01: SPA."""

class Robot:
    """Turtlebot robot."""

    def __init__(self, robot: object) -> None:
        """Class initializer.

        Args:
            robot (object): An instance of a Turtlebot-like robot interface.
        """
        self.robot = robot
    
    def sense(self):
        """Gathers data."""
        pass
    
    def plan(self):
        """Process the data."""
        pass

    def act(self):
        """Excecutes planned actions."""
        pass

    def spin(self):
        """Spin the robot."""
        self.sense()
        self.plan()
        self.act()
