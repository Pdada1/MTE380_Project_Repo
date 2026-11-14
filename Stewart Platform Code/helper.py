"""Camera, platform coordinates, and ball detection shared by both programs."""

import math
import sys
from pathlib import Path

import cv2


CONFIG_PATH = Path(__file__).with_name("config.json")
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
SERVO_PINS = (8, 9, 10)


def open_camera():
    """
    Open the camera using the win32 backend
    """
    backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
    print(f"Opening camera {CAMERA_INDEX}...", flush=True)
    camera = cv2.VideoCapture(CAMERA_INDEX, backend)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"Unable to open camera {CAMERA_INDEX}")
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    print(f"Camera {CAMERA_INDEX} opened. Waiting for video...", flush=True)
    return camera


class Platform:
    def __init__(self, origin, radius, servo_points, x_index=0, pins=SERVO_PINS):
        self.origin = tuple(origin)
        self.radius = radius
        self.servo_points = tuple(servo_points)
        self.x_point = tuple(servo_points[x_index])

        ox, oy = self.origin
        angles = [math.atan2(oy - y, x - ox) for x, y in servo_points]
        x_angle = angles[x_index]
        self.cos_x = math.cos(x_angle)
        self.sin_x = math.sin(x_angle)

        # Preserve the pin assignment even if servos 1 and 2 were clicked clockwise.
        order = sorted(range(3), key=lambda i: (angles[i] - x_angle) % math.tau)
        self.pins = tuple(pins[i] for i in order)

    def normalize(self, point):
        """Convert image pixels to normalized coordinates relative to the selected +X servo."""
        dx = (point[0] - self.origin[0]) / self.radius
        dy = (self.origin[1] - point[1]) / self.radius
        return (
            dx * self.cos_x + dy * self.sin_x,
            dy * self.cos_x - dx * self.sin_x,
        )

    def draw(self, frame):
        center = tuple(map(round, self.origin))
        cv2.circle(frame, center, round(self.radius), (255, 255, 0), 1)
        cv2.line(frame, center, self.x_point, (255, 255, 0), 1)


class BallDetector:
    def __init__(self, lower_hsv, upper_hsv, min_radius_px):
        self.lower = tuple(lower_hsv)
        self.upper = tuple(upper_hsv)
        self.min_radius = min_radius_px
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    def detect(self, frame):
        """Return ((x, y), radius) in pixels, or None."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            center, radius = cv2.minEnclosingCircle(contour)
            if radius < self.min_radius:
                continue
            if cv2.contourArea(contour) >= 0.6 * math.pi * radius**2:
                return center, radius

        return None


def draw_ball(frame, detection):
    if detection is not None:
        center, radius = detection
        cv2.circle(frame, tuple(map(round, center)), round(radius), (0, 255, 255), 2)
