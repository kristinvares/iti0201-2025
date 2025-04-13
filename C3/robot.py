from __future__ import annotations
import numpy as np


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
        labeled, count = self.find_blobs(mask)
        if count == 0:
            return None
        best_box = None
        best_y_center = 0
        for i in range(1, count + 1):
            pixels = np.column_stack(np.where(labeled == i))
            if pixels.size == 0:
                continue
            y_min, x_min = pixels.min(axis=0)
            y_max, x_max = pixels.max(axis=0)
            width = x_max - x_min
            height = y_max - y_min
            aspect_ratio = width / height if height != 0 else 0
            if 0.6 < aspect_ratio < 1.5:
                y_center = (y_min + y_max) / 2
                if y_center > best_y_center:
                    best_y_center = y_center
                    best_box = (x_min, x_max, y_min, y_max)
        if best_box is None:
            return None
        x_min, x_max, y_min, y_max = best_box
        x_center = (x_min + x_max) / 2
        width = image.shape[1]
        angle = ((x_center - width / 2) / (width / 2)) * (self.fov / 2)
        height = image.shape[0]
        y_from_bottom = height - ((y_min + y_max) / 2)
        estimated_distance = y_from_bottom * 0.006
        return angle, estimated_distance

    def find_blobs(self, mask):
        height, width = mask.shape
        labeled = np.zeros_like(mask, dtype=np.uint32)
        label_id = 1
        to_visit = []
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for y, x in np.argwhere(mask):
            if labeled[y, x] == 0:
                labeled[y, x] = label_id
                to_visit.append((y, x))
                while to_visit:
                    cy, cx = to_visit.pop()
                    for dy, dx in neighbors:
                        ny, nx = cy + dy, cx + dx
                        if 0 <= ny < height and 0 <= nx < width:
                            if mask[ny, nx] and labeled[ny, nx] == 0:
                                labeled[ny, nx] = label_id
                                to_visit.append((ny, nx))
                label_id += 1
        return labeled, label_id - 1
