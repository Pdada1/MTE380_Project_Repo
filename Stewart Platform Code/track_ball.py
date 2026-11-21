"""Track the ball and send normalized positions over USB serial.

Run python track_ball.py to transmit, or use --preview to run without serial.
Use --serial-debug to monitor the Arduino's print statements as well.
Click to set a target, press C to center, O for a circle, T for a triangle,
or Q to quit.
"""

import argparse
import json
import math
import struct
import time

import cv2

from helper import CONFIG_PATH, SERVO_PINS, BallDetector, Platform, draw_ball, open_camera

SYNC = bytes((0x0B, 0x00, 0x0B))
PACKET = struct.Struct("<3sB3B4f")
ARDUINO_SERIAL_PORT = "COM5"
BAUD_RATE = 115200
WINDOW = "Platform Tracking"

TRAJECTORY_SPEED = 0.35
TRACKING_SLOW_ERROR = 0.10
TRACKING_PAUSE_ERROR = 0.30
MIN_SPEED_FACTOR = 0.30
CIRCLE_POINTS = 108              # More points make the displayed circle smoother
TRIANGLE_RADIUS = 0.8          # Triangle circumradius as fraction of platform radius
PLATFORM_MARGIN = 0.10         # Keep trajectories away from the platform edge


def open_serial():
    import serial

    port = serial.Serial(ARDUINO_SERIAL_PORT, BAUD_RATE, write_timeout=0.1)
    try:
        time.sleep(2)  # Allow boards that reset on serial open to boot.
    except BaseException:
        port.close()
        raise
    return port


def tracking_packet(pins, ball, target):
    """
    Pack sync, three CCW servo pins, ball XY, target XY, and ball validity
    for transmission to the Arduino.
    """
    valid = ball is not None
    if not valid:
        ball = (0.0, 0.0)
    return PACKET.pack(SYNC, valid, *pins, *ball, *target)


def make_trajectory(platform, start_px, shape):
    """Return a pixel path for one circle or triangle, starting at the ball."""
    ox, oy = platform.origin
    x, y = start_px
    radius = platform.radius
    safe_radius = radius * (1 - PLATFORM_MARGIN)
    distance = math.hypot(x - ox, y - oy)

    if shape == "circle":
        # Circle is centered on the platform, at the ball's current radius.
        r = min(distance, safe_radius)
        if r < radius * 0.05:
            return []  # No useful circular path at the origin.
        start_angle = math.atan2(y - oy, x - ox)
        return [
            (ox + r * math.cos(start_angle + 2 * math.pi * i / CIRCLE_POINTS),
             oy + r * math.sin(start_angle + 2 * math.pi * i / CIRCLE_POINTS))
            for i in range(CIRCLE_POINTS + 1)
        ]

    elif shape == "triangle":
        # Triangle is centered on the ball's current position.
        r = min(radius * TRIANGLE_RADIUS, safe_radius - distance)
        if r <= radius * 0.05:
            return []
        vertices = [
            (x + r * math.cos(2 * math.pi * i / 3),
             y + r * math.sin(2 * math.pi * i / 3))
            for i in range(3)
        ]
        # Approach vertex 0, then follow all three sides back to vertex 0.
        return [start_px, *vertices, vertices[0]]

    else:
        raise NotImplementedError("Only circles and triangles are supported for now")


def draw_preview(frame, platform, detection, target_px, ball, target, path):
    """Display the platform, ball, target, and desired trajectory."""
    platform.draw(frame)
    draw_ball(frame, detection)
    if path:
        for a, b in zip(path, path[1:]):
            cv2.line(frame, tuple(map(round, a)), tuple(map(round, b)), (0, 165, 255), 2)
    cv2.drawMarker(frame, tuple(map(round, target_px)), (0, 255, 0))
    status = "Ball lost" if ball is None else f"Ball ({ball[0]:.2f}, {ball[1]:.2f})"
    status += f"   Target ({target[0]:.2f}, {target[1]:.2f})"
    cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, "Click: target | O: circle | T: triangle | C: center | Q: quit",
                (10, frame.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imshow(WINDOW, frame)


def track(camera, platform, detector, port, serial_debug=False):
    """Track the ball and move the target smoothly along a trajectory."""
    target_px = platform.origin
    path = []
    segment = 0
    segment_distance = 0.0
    previous_time = time.monotonic()
    shape = None

    def select_target(event, x, y, flags, param):
        # Gather user mouse input to determine target location from frame
        nonlocal target_px, path, shape
        if event == cv2.EVENT_LBUTTONDOWN:
            target_px = (x, y)
            path = []
            shape = None

    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(WINDOW, select_target)

    # Main tracking loop
    while True:
        ok, frame = camera.read()
        now = time.monotonic()
        dt = min(now - previous_time, 0.1)  # Prevent a large jump after a slow frame.
        previous_time = now
        target = platform.normalize(target_px)
        if not ok:
            # Write default packet so Arduino is aware that the camera isn't returning frames
            if port is not None:
                port.write(tracking_packet(platform.pins, None, target))
            raise RuntimeError("Camera stopped returning frames")

        # Find the ball using BallDetector, normalize its location, then write the packet to Arduino
        detection = detector.detect(frame)
        ball = platform.normalize(detection[0]) if detection is not None else None

        if path and ball is not None:
            error = math.dist(ball, target)
            if error < TRACKING_PAUSE_ERROR:
                # Reduce target speed when the ball is not keeping up.
                if error <= TRACKING_SLOW_ERROR:
                    speed_factor = 1.0
                else:
                    fraction = (TRACKING_PAUSE_ERROR - error) / (TRACKING_PAUSE_ERROR - TRACKING_SLOW_ERROR)
                    speed_factor = MIN_SPEED_FACTOR + (1 - MIN_SPEED_FACTOR) * fraction

                # Slow near triangle corners to avoid sudden direction changes.
                if shape == "triangle":
                    a, b = path[segment], path[segment + 1]
                    length = math.dist(a, b)
                    corner_distance = min(segment_distance, length - segment_distance)
                    speed_factor *= max(0.25, min(1.0, corner_distance / (0.12 * platform.radius)))

                remaining = TRAJECTORY_SPEED * platform.radius * dt * speed_factor
                while remaining > 0 and segment < len(path) - 1:
                    a, b = path[segment], path[segment + 1]
                    length = math.dist(a, b)
                    step = min(remaining, length - segment_distance)
                    segment_distance += step
                    remaining -= step
                    if segment_distance >= length - 1e-6:
                        segment += 1
                        segment_distance = 0.0
                        if segment == len(path) - 1:
                            target_px = path[-1]
                            path = []
                            shape = None
                            break
                    if length <= 1e-6:
                        segment += 1
                        segment_distance = 0.0
                if path:
                    a, b = path[segment], path[segment + 1]
                    length = math.dist(a, b)
                    fraction = segment_distance / length if length else 0.0
                    target_px = (a[0] + fraction * (b[0] - a[0]),
                                 a[1] + fraction * (b[1] - a[1]))
                target = platform.normalize(target_px)

        if port is not None:
            port.write(tracking_packet(platform.pins, ball, target))
            if serial_debug:
                # Print the Arduino's serial print statements for debugging,
                # while keeping only one serial port open.
                output = port.read_all()
                if output:
                    print(output.decode("ascii", errors="replace"), end="", flush=True)

        draw_preview(frame, platform, detection, target_px, ball, target, path)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            return
        if key == ord("c"):
            target_px, path, shape = platform.origin, [], None
        if key in (ord("o"), ord("t")) and detection is not None:
            # Draw a circular or triangular trajectory
            next_shape = "circle" if key == ord("o") else "triangle"
            next_path = make_trajectory(platform, detection[0], next_shape)
            if next_path:
                path = next_path
                shape = next_shape
                segment = 0
                segment_distance = 0.0
                target_px = path[0]
            else:
                print(f"Cannot fit this {next_shape} at the ball's current position.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preview", action="store_true", help="Run without serial.")
    parser.add_argument("--serial-debug", action="store_true", help="Print Arduino serial output.")
    args = parser.parse_args()

    # Read the config.json parameters for use during tracking
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    geometry = config["calibration"]
    servos = sorted(config["servos"], key=lambda servo: servo["index"])
    platform = Platform(
        geometry["origin_px"],
        geometry["platform_radius_px"],
        [servo["point_px"] for servo in servos],
        x_index=geometry.get("zero_degree_servo", 0),
        pins=[servo.get("pin", SERVO_PINS[servo["index"]]) for servo in servos],
    )
    detector = BallDetector(**config["ball_detection"])

    camera = open_camera()
    try:
        if args.preview:
            # Run as preview, with no transmission to Arduino
            track(camera, platform, detector, None)
        else:
            # Run full algorithm with transmission to Arduino
            with open_serial() as port:
                track(camera, platform, detector, port, args.serial_debug)
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
