"""Ball and platform calibration for the graphical interface."""

import json
import math

import cv2

from helper import CAMERA_INDEX, CONFIG_PATH, SERVO_PINS, BallDetector, Platform


MIN_BALL_RADIUS = 5


def fit_circle(points):
    """Return the center and radius of the circle through three points."""
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


def sample_ball_color(frame, detector, x, y):
    """Expand the HSV bounds using a small patch around the click."""
    patch = frame[max(0, y - 2):y + 3, max(0, x - 2):x + 3]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    old_lower = detector.lower if detector else (179, 255, 255)
    old_upper = detector.upper if detector else (0, 0, 0)
    lower = []
    upper = []
    for channel, old_low, old_high, margin, limit in zip(
        cv2.split(hsv), old_lower, old_upper, (5, 10, 10), (179, 255, 255)
    ):
        sample_low, sample_high, _, _ = cv2.minMaxLoc(channel)
        lower.append(min(old_low, max(0, int(sample_low) - margin)))
        upper.append(max(old_high, min(limit, int(sample_high) + margin)))
    return BallDetector(lower, upper, MIN_BALL_RADIUS)


def find_platform(frame):
    """Detect the three RGB squares and fit the platform circle."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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

    return origin, radius, points


def identify_marker(baseline, tilted):
    """Identify which RGB marker the center moved away from."""
    (x, y), radius, points = baseline
    dx, dy = tilted[0][0] - x, tilted[0][1] - y
    scores = [((x - px) * dx + (y - py) * dy) / radius for px, py in points]
    best, second, _ = sorted(range(3), key=scores.__getitem__, reverse=True)
    if scores[best] >= 3 and scores[best] - scores[second] >= 2:
        return best
    return None


def read_config():
    """Load the saved calibration or return an empty configuration."""
    return json.loads(CONFIG_PATH.read_text(encoding="utf-8")) if CONFIG_PATH.exists() else {}


def calibration_objects(config, detector=None, platform=None):
    """Build the detector and platform from available calibration data."""
    if "ball_detection" in config:
        detector = BallDetector(**config["ball_detection"])
    if "calibration" in config and "servos" in config:
        geometry = config["calibration"]
        servos = sorted(config["servos"], key=lambda servo: servo["index"])
        platform = Platform(
            geometry["origin_px"], geometry["platform_radius_px"],
            [servo["point_px"] for servo in servos],
            geometry.get("zero_degree_servo", 0),
            [servo.get("pin", SERVO_PINS[servo["index"]]) for servo in servos],
        )
    return detector, platform


def save_config(config, frame, action, candidate):
    """Save the selected calibration and preserve the other settings."""
    config = dict(config)
    height, width = frame.shape[:2]
    config["camera"] = {"index": CAMERA_INDEX, "frame_width": width, "frame_height": height}
    if action == "ball":
        detector = candidate
        config["ball_detection"] = {
            "lower_hsv": detector.lower, "upper_hsv": detector.upper,
            "min_radius_px": detector.min_radius,
        }
    elif action == "platform":
        origin, radius, points, pins = candidate
        config["calibration"] = {
            "origin_px": origin, "platform_radius_px": radius,
            "zero_degree_servo": config.get("calibration", {}).get("zero_degree_servo", 0),
        }
        config["servos"] = [{"index": i, "point_px": point, "pin": pins[i]}
                            for i, point in enumerate(points)]

    temp = CONFIG_PATH.with_suffix(".tmp")
    temp.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    # Replace the saved file only after the temporary write succeeds.
    temp.replace(CONFIG_PATH)
    return config
