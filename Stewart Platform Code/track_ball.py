"""Track the ball and send normalized positions over USB serial.

Run python track_ball.py to transmit, or use --preview to run without serial.
Use --serial-debug to monitor the Arduino's print statements as well.
Click to set the target, press C to center it, or press Q to quit.
"""

import argparse
import json
import struct
import time

import cv2

from helper import CONFIG_PATH, SERVO_PINS, BallDetector, Platform, draw_ball, open_camera

SYNC = bytes((0x0B, 0x00, 0x0B))
PACKET = struct.Struct("<3sB3B4f")
ARDUINO_SERIAL_PORT = "COM6"
BAUD_RATE = 115200
WINDOW = "Platform Tracking"

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


def draw_preview(frame, platform, detection, target_px, ball, target):
    """
    Display and update the platform and ball previews, along with the target
    location.
    """
    platform.draw(frame)
    draw_ball(frame, detection)
    cv2.drawMarker(frame, tuple(map(round, target_px)), (0, 255, 0))
    status = "Ball lost" if ball is None else f"Ball ({ball[0]:.2f}, {ball[1]:.2f})"
    status += f"   Target ({target[0]:.2f}, {target[1]:.2f})"
    cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imshow(WINDOW, frame)


def track(camera, platform, detector, port, serial_debug=False):
    """
    Continuously keep track of the ball in the video frame,
    while showing previews of where the ball is and where we
    want it to move. 
    """
    target_px = platform.origin

    def select_target(event, x, y, flags, param):
        # Gather user mouse input to determine target location from frame
        nonlocal target_px
        if event == cv2.EVENT_LBUTTONDOWN:
            target_px = (x, y)

    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(WINDOW, select_target)

    # Main tracking loop
    while True:
        ok, frame = camera.read()
        target = platform.normalize(target_px)
        if not ok:
            # Write default packet so Arduino is aware that the camera isn't returning frames
            if port is not None:
                port.write(tracking_packet(platform.pins, None, target))
            raise RuntimeError("Camera stopped returning frames")

        # Find the ball using BallDetector, normalize its location, then write the packet to Arduino
        detection = detector.detect(frame)
        ball = platform.normalize(detection[0]) if detection is not None else None
        if port is not None:
            port.write(tracking_packet(platform.pins, ball, target))
            if serial_debug:
                # Print the Arduino's serial print statements for debugging,
                # while keeping only one serial port open.
                output = port.read_all()
                if output:
                    print(output.decode("ascii", errors="replace"), end="", flush=True)

        draw_preview(frame, platform, detection, target_px, ball, target)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            return
        if key == ord("c"):
            target_px = platform.origin


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
    camera_settings = config["camera"]
    frame_size = (camera_settings["frame_width"], camera_settings["frame_height"])
    
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
