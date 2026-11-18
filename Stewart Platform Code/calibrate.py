"""Sample ball color and click the three servo locations to save calibration."""

import json
import math
import numpy as np
import cv2

from helper import CAMERA_INDEX, CONFIG_PATH, SERVO_PINS, BallDetector, Platform, draw_ball, open_camera


WINDOW = "Platform Calibration"
MIN_BALL_RADIUS = 5


def fit_circle(points):
    """Return the circumcenter and radius """
    (x0, y0), (x1, y1), (x2, y2) = points
    ax, ay = x1 - x0, y1 - y0
    bx, by = x2 - x0, y2 - y0
    determinant = 2 * (ax * by - ay * bx)
    if determinant == 0:
        raise ValueError("Servo points must be distinct and non-collinear")

    # Solve for the center offset that is equidistant from all
    # three points, also known as the circumcenter
    a_squared = ax * ax + ay * ay
    b_squared = bx * bx + by * by
    dx = (a_squared * by - b_squared * ay) / determinant
    dy = (ax * b_squared - bx * a_squared) / determinant

    return (x0 + dx, y0 + dy), math.hypot(dx, dy)


def identify_servos(platform):
    """Return (positive X point index, pins in calibration click order).

    Step C placeholder. A future implementation can move each servo and
    use the camera to match its pin to one of platform.servo_points.
    """
    return 0, SERVO_PINS


class Calibration:
    def __init__(self):
        self.frame = None
        self.detector = None
        self.status = "Selecting Ball"
        self.points = []
        self.platform = None

    def on_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or self.frame is None:
            # Ignore unexpected click events
            return

        if self.status == "Selecting Ball":
            # Calibration stage A: sample the ball
            self.sample_ball_color(x, y)

    def sample_ball_color(self, x, y):
        """
        Samples the ball color and stores the associated parameters to
        track it during operation.
        """
        patch = self.frame[max(0, y - 2):y + 3, max(0, x - 2):x + 3]
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        old_lower = self.detector.lower if self.detector else (179, 255, 255)
        old_upper = self.detector.upper if self.detector else (0, 0, 0)
        lower = []
        upper = []
        for channel, old_low, old_high, margin, limit in zip(
            cv2.split(hsv), old_lower, old_upper, (5, 10, 10), (179, 255, 255)
        ):
            sample_low, sample_high, _, _ = cv2.minMaxLoc(channel)
            lower.append(min(old_low, max(0, int(sample_low) - margin)))
            upper.append(max(old_high, min(limit, int(sample_high) + margin)))
        self.detector = BallDetector(lower, upper, MIN_BALL_RADIUS)


    def select_servo_point(self):
        """Detect the three RGB squares and fit the platform circle."""
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        ranges = [
            [(0, 100, 70), (10, 255, 255)],    # Red
            [(35, 70, 50), (85, 255, 255)],    # Green
            [(95, 70, 50), (135, 255, 255)],   # Blue
        ]

        points = []

        for i, (lower, upper) in enumerate(ranges):
            mask = cv2.inRange(hsv, lower, upper)

            if i == 0:
                mask |= cv2.inRange(hsv, (170, 100, 70), (179, 255, 255))

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            candidates = []

            for contour in contours:
                area = cv2.contourArea(contour)
                (cx, cy), radius = cv2.minEnclosingCircle(contour)

                # Look for small colored squares, not large reflections
                if not 10 <= area <= 250 or not 2 <= radius <= 12:
                    continue

                x, y, w, h = cv2.boundingRect(contour)

                # Reject long, thin objects
                if not 0.5 <= w / h <= 2.0:
                    continue

                candidates.append((area, (int(cx), int(cy))))

            if not candidates:
                return

            points.append(max(candidates)[1])

        # Reject points that cannot define the expected platform geometry
        distances = [
            math.dist(points[i], points[(i + 1) % 3])
            for i in range(3)
        ]

        if min(distances) < 50 or max(distances) > 1.8 * min(distances):
            return

        try:
            origin, radius = fit_circle(points)
        except ValueError:
            return

        self.points = points
        self.platform = Platform(origin, radius, points)

    def draw(self):
        preview = self.frame.copy()
        if self.platform is not None:
            self.platform.draw(preview)
            message = "S: save | D: Re-detect | Q: quit"
        else:
            message = "Phase A: Click ball to sample | C: continue | R: reset | Q: quit"

        for point, pin in zip(self.points, SERVO_PINS):
            # Draw each of the servo points for awareness
            cv2.circle(preview, point, 4, (0, 255, 0), -1)
            cv2.putText(preview, str(pin), (point[0] + 6, point[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        if self.detector is not None:
            draw_ball(preview, self.detector.detect(self.frame))
        cv2.putText(preview, message, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1, cv2.LINE_AA)
        return preview

    def save(self):
        x_index, pins = identify_servos(self.platform)
        height, width = self.frame.shape[:2]
        config = {
            "camera": {"index": CAMERA_INDEX, "frame_width": width, "frame_height": height},
            "ball_detection": {
                "lower_hsv": self.detector.lower,
                "upper_hsv": self.detector.upper,
                "min_radius_px": MIN_BALL_RADIUS,
            },
            "calibration": {
                "origin_px": self.platform.origin,
                "platform_radius_px": self.platform.radius,
                "zero_degree_servo": x_index,
            },
            "servos": [
                {"index": index, "point_px": point, "pin": pin}
                for index, (point, pin) in enumerate(zip(self.points, pins))
            ],
        }
        CONFIG_PATH.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
        print(f"Saved calibration parameters to {CONFIG_PATH.name}")


def main():
    calibration = Calibration()
    camera = open_camera()
    try:
        cv2.namedWindow(WINDOW)

        # This runs every time the mouse is clicked. What it does depends on the 
        # current calibration phase (A or B).
        cv2.setMouseCallback(WINDOW, calibration.on_click)

        # Main calibration loop
        while True:
            ok, calibration.frame = camera.read()
            if not ok:
                raise RuntimeError("Camera stopped returning frames")

            if calibration.status == "Selecting Platform" and calibration.platform is None:
                calibration.select_servo_point()
            cv2.imshow(WINDOW, calibration.draw())

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                # Quit
                break
            if key == ord("c") and calibration.detector is not None:
                # Proceed to selecting servo points to define the circular platform
                calibration.status = "Selecting Platform"
            if key == ord("r") and calibration.status == "Selecting Ball":
                # Reset the ball sampler
                calibration.detector = None
            if key == ord("d") and calibration.points:
                # Re-detect the servo points
                calibration.points.pop()
                calibration.platform = None
            if key == ord("s") and calibration.platform is not None:
                # Save calibration to json file
                calibration.save()
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
