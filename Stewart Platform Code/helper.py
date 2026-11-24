"""Shared camera access, platform coordinates, and ball detection."""

import math
from pathlib import Path
import sys

import cv2


CONFIG_PATH = Path(__file__).with_name("config.json")
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
SERVO_PINS = (0, 2, 1) # corresponds to servo indices 0, 2, and 1


def open_camera():
    """Open the configured camera and request the capture dimensions."""
    backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
    camera = cv2.VideoCapture(CAMERA_INDEX, backend)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"Unable to open camera {CAMERA_INDEX}")
    # MJPG first: uncompressed 720p is limited to a low frame rate on many webcams.
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    camera.set(cv2.CAP_PROP_FPS, 30)
    return camera


class Platform:
    """Store the calibrated platform geometry and servo mapping."""
    def __init__(self, origin, radius, servo_points, x_index=0, pins=SERVO_PINS):
        """Set the origin, scale, +X direction, and servo pin order."""
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
        """Overlay the platform boundary and +X axis on the frame."""
        center = tuple(map(round, self.origin))
        cv2.circle(frame, center, round(self.radius), (255, 255, 0), 1)
        cv2.line(frame, center, self.x_point, (255, 255, 0), 1)


class BallDetector:
    """Locate the ball by color, size, and contour shape."""
    def __init__(self, lower_hsv, upper_hsv, min_radius_px):
        """Store the HSV bounds, minimum radius, and mask cleanup kernel."""
        self.lower = tuple(lower_hsv)
        self.upper = tuple(upper_hsv)
        self.min_radius = min_radius_px
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    def detect(self, frame):
        """Return ((x, y), radius) in pixels, or None."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        # Remove isolated pixels before checking contour size and shape.
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
    """Outline the detected ball on the frame when visible."""
    if detection is not None:
        center, radius = detection
        cv2.circle(frame, tuple(map(round, center)), round(radius), (0, 255, 255), 2)