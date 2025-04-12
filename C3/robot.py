"""C3."""

class Robot:
    """Turtlebot robot."""

    def __init__(self, robot: object) -> None:
        """Class initializer.

        Args:
            robot (object): An instance of a Turtlebot-like robot interface.
        """
        self.robot = robot
        self.image = None
        self.fov = None
        self.blue_cubes = None

    def find_blobs(self, mask):
        """
        Flood fill algorithm to find the blue object.

        :param mask:
        :return:
        """
        height, width = mask.shape
        labled_mask = np.zeros_like(mask, dtype=np.uint32)

        lable_id = 1
        to_visit = []
        neighbours = ((-1, 0), (1, 0), (0, -1), (0, 1))

        for y, x in np.argwhere(mask):
            if labled_mask[y, x] == 0:
                labled_mask[y, x] = lable_id
                to_visit.append((y, x))
                while to_visit:
                    current_y, current_x = to_visit.pop()
                    for dy, dx in neighbours:
                        new_y, new_x = current_y + dy, current_x + dx
                        if 0 <= new_y < height and 0 <= new_x < width:
                            if mask[new_y, new_x] and labled_mask[new_y, new_x] == 0:
                                labled_mask[new_y, new_x] = lable_id
                                to_visit.append((new_y, new_x))
                lable_id += 1

        return labled_mask, lable_id - 1

    def get_object_bounding_box_list(self) -> list | None:
        if self.image is None:
            return None

        blue_channel = self.image[:, :, 0]
        green_channel = self.image[:, :, 1]
        red_channel = self.image[:, :, 2]
        threshold = 50

        mask = (blue_channel > green_channel + threshold) & (blue_channel > red_channel + threshold)
        labled_mask, lable_count = self.find_blobs(mask)

        if lable_count == 0:
            return None

        blobs = []
        for i in range(1, lable_count + 1):
            blobs_pixels = np.column_stack(np.where(labled_mask == i))
            if blobs_pixels.size == 0:
                return None

            y_min, x_min = blobs_pixels.min(axis=0)
            y_max, x_max = blobs_pixels.max(axis=0)
            blobs.append((x_min, x_max, y_min, y_max))

        return blobs if blobs else None

    def update_cube_objects(self):
        boxes = self.get_object_bounding_box_list()
        if boxes is None:
            return None
        something = []
        for box in boxes:
            x_min = box[0]
            x_max = box[1]
            y_min = box[2]
            y_max = box[3]


            x_side = x_max - x_min
            y_side = y_max - y_min

            threshold = 20
            diffrence = 0
            if x_side > y_side:
                diffrence = x_side - y_side
            elif y_side > x_side:
                diffrence = y_side - x_side

            if diffrence <= threshold:
                something.append(box)
        return something

    def get_cube_objects(self) -> list | None:
        self.blue_cubes = self.update_cube_objects()
        if self.blue_cubes:
            return self.blue_cubes
        else:
            return None

    def sense(self) -> None:
        """Gather sensor data.

        Use the robot's sensors to collect data about its environment.
        This method updates internal state variables based on sensor readings.
        """
        self.image = self.robot.get_camera_rgb_image()
        self.update_cube_objects()
        self.fov = self.robot.get_camera_field_of_view()

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