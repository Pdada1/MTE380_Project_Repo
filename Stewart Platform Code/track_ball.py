"""Ball tracking, trajectory motion, measurement history, and Arduino I/O."""

from collections import deque
from enum import IntEnum
import math
from queue import Empty, Full, Queue
import struct
from threading import Event, Thread
import time

import cv2

from helper import SERVO_PINS, open_camera


SYNC = bytes((0x0B, 0x00, 0x0B))
PACKET = struct.Struct("<3sB3B4fI3f")
ARDUINO_SERIAL_PORT = "COM6"
BAUD_RATE = 115200
DEFAULT_GAINS = (4.2, 0.5, 3.9)
TRAJECTORY_GAINS = (5.0, 0.8, 4.5)  # Kp, Ki, Kd for circle and triangle

TRAJECTORY_SPEED = 0.5
TRACKING_SLOW_ERROR = 0.05
TRACKING_PAUSE_ERROR = 0.30
MIN_SPEED_FACTOR = 0.30
CIRCLE_POINTS = 108              # More points make the displayed circle smoother
TRIANGLE_RADIUS = 0.8          # Triangle circumradius as fraction of platform radius
PLATFORM_MARGIN = 0.10         # Keep trajectories away from the platform edge

HISTORY_SECONDS = 30
TRIM_PINS = SERVO_PINS
TRIM_LIMIT = 10


class Status(IntEnum):
    BALL_LOST = 0
    TRACKING = 1
    LEVEL = 2
    AUTOTUNE = 3
    AUTOLEVEL = 4
    SWEEP = 5
    SERVO_TRIM = 6
    CALIBRATE_SERVOS = 7


def put_latest(queue, item):
    """Replace an unconsumed item instead of building up latency."""
    try:
        queue.put_nowait(item)
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            pass
        queue.put_nowait(item)


class Hardware:
    """Own the camera and serial port on separate worker threads."""

    def __init__(self, preview, port_name, debug):
        self.stop = Event()
        self.frames = Queue(maxsize=1)
        self.commands = Queue(maxsize=1)
        self.messages = Queue()
        self.camera_thread = Thread(target=self.capture, daemon=True)
        self.camera_thread.start()
        self.serial_thread = None
        if not preview:
            self.serial_thread = Thread(target=self.transmit, args=(port_name, debug), daemon=True)
            self.serial_thread.start()

    def capture(self):
        camera = None
        try:
            camera = open_camera()
            camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            while not self.stop.is_set():
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError("Camera stopped returning frames")
                put_latest(self.frames, (time.monotonic(), frame))
        except Exception as error:
            self.messages.put(("camera", str(error)))
        finally:
            if camera is not None:
                camera.release()

    def transmit(self, port_name, debug):
        try:
            import serial

            with serial.Serial(port_name, BAUD_RATE, write_timeout=0.1) as port:
                if self.stop.wait(2):
                    return
                self.messages.put(("serial", f"{port_name} connected"))
                last_trim = None
                last_level = None
                try:
                    while not self.stop.is_set():
                        try:
                            trim, command, last_level = self.commands.get(timeout=0.05)
                        except Empty:
                            continue
                        if trim != last_trim:
                            port.write(trim)
                            last_trim = trim
                        port.write(command)
                        if debug:
                            output = port.read_all()
                            if output:
                                print(output.decode("ascii", errors="replace"), end="", flush=True)
                finally:
                    if last_level is not None:
                        port.write(last_level)
        except Exception as error:
            self.messages.put(("serial", str(error)))


class Trajectory:
    def __init__(self, points, shape):
        self.points = points
        self.shape = shape
        self.segment = 0
        self.distance = 0.0

    def advance(self, platform, ball, target, dt):
        error = math.dist(ball, platform.normalize(target))
        if error >= TRACKING_PAUSE_ERROR:
            return target, False
        factor = min(1.0, (TRACKING_PAUSE_ERROR - error) / (TRACKING_PAUSE_ERROR - TRACKING_SLOW_ERROR))
        speed = MIN_SPEED_FACTOR + (1 - MIN_SPEED_FACTOR) * factor
        if self.shape == "triangle":
            length = math.dist(*self.points[self.segment:self.segment + 2])
            corner = min(self.distance, length - self.distance)
            speed *= max(0.25, min(1.0, corner / (0.12 * platform.radius)))

        distance = TRAJECTORY_SPEED * platform.radius * dt * speed
        while self.segment < len(self.points) - 1:
            a, b = self.points[self.segment:self.segment + 2]
            length = math.dist(a, b)
            remaining = length - self.distance
            if distance < remaining:
                self.distance += distance
                fraction = self.distance / length
                return tuple(p + fraction * (q - p) for p, q in zip(a, b)), False
            distance -= remaining
            self.segment += 1
            self.distance = 0.0
        return self.points[-1], True


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


class History:
    """Plot target-relative position and derivatives of the ball's position."""

    def __init__(self):
        self.samples = deque()
        self.reset_derivatives()

    def reset_derivatives(self):
        self.previous = None
        self.velocity = None

    def prune(self, now):
        while self.samples and self.samples[0][0] < now - HISTORY_SECONDS:
            self.samples.popleft()

    def add(self, now, ball, target):
        if ball is None:
            self.reset_derivatives()
            self.samples.append((now, (None, None, None)))
            return

        error = tuple(b - t for b, t in zip(ball, target))
        velocity = acceleration = None
        if self.previous is not None:
            timestamp, last_ball = self.previous
            dt = now - timestamp
            if 0 < dt <= 0.25:
                velocity = tuple((b - p) / dt for b, p in zip(ball, last_ball))
                if self.velocity is not None:
                    last_velocity, last_dt = self.velocity
                    acceleration = tuple((v - p) / ((dt + last_dt) / 2)
                                         for v, p in zip(velocity, last_velocity))
        self.velocity = (velocity, dt) if velocity is not None else None
        self.previous = now, ball
        self.samples.append((now, (error, velocity, acceleration)))


class Tracker:
    def __init__(self):
        self.platform = self.detector = self.target = None
        self.frame = self.frame_time = self.detection = None
        self.path = None
        self.history = History()
        self.trajectory_gains = [round(value * 10) for value in TRAJECTORY_GAINS]

    def active_gains(self, manual_gains):
        return self.trajectory_gains if self.path is not None else manual_gains

    def start_path(self, points, shape):
        self.path = Trajectory(points, shape)
        self.target = points[0]

    def process_frame(self, now, frame, detector):
        dt = min(now - self.frame_time, 0.1) if self.frame_time is not None else 0
        self.frame, self.frame_time = frame, now
        self.detection = detector.detect(frame) if detector is not None else None
        if self.platform is None:
            return False
        ball = self.platform.normalize(self.detection[0]) if self.detection is not None else None
        complete = False
        if self.path is not None and ball is not None:
            self.target, complete = self.path.advance(self.platform, ball, self.target, dt)
        self.history.add(now, ball, self.platform.normalize(self.target))
        return complete

    def send_command(self, hardware, now, action, gains, trims, sweep_id):
        pins = self.platform.pins if self.platform is not None else SERVO_PINS
        manual_gains = gains
        gains = tuple(value / 10 for value in manual_gains)
        frame_ms = int((self.frame_time if self.frame_time is not None else now) * 1000) & 0xFFFFFFFF
        ball = (0.0, 0.0)
        target = self.platform.normalize(self.target) if self.platform is not None else (0.0, 0.0)
        status = Status.BALL_LOST
        if isinstance(action, Status):
            status = action
        elif action in ("ball", "platform"):
            status = Status.LEVEL
        elif (self.platform is not None and self.detection is not None
              and now - self.frame_time <= 0.25):
            status = Status.TRACKING
            ball = self.platform.normalize(self.detection[0])
        trim = PACKET.pack(SYNC, Status.SERVO_TRIM, *TRIM_PINS, *trims, 0.0, 0, *DEFAULT_GAINS)
        timestamp = sweep_id if status == Status.SWEEP else frame_ms
        tracking_gains = gains
        if status == Status.TRACKING:
            tracking_gains = tuple(value / 10 for value in self.active_gains(manual_gains))
        command = PACKET.pack(SYNC, status, *pins, *ball, *target, timestamp, *tracking_gains)
        level = PACKET.pack(SYNC, Status.LEVEL, *pins, 0, 0, 0, 0, frame_ms, *gains)
        put_latest(hardware.commands, (trim, command, level))
