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
# Wire fields in order: sync, status, pins, ball XY, target XY, timestamp, PID gains.
PACKET = struct.Struct("<3sB3B4fI3f")
ARDUINO_SERIAL_PORT = "COM6"
BAUD_RATE = 115200
DEFAULT_GAINS = (4.5, 0.5, 3.7)
TRAJECTORY_GAINS = (5.0, 0.8, 4.5)  # Kp, Ki, Kd for circle tracking

TRAJECTORY_SPEED = 0.5
TRACKING_SLOW_ERROR = 0.05
TRACKING_PAUSE_ERROR = 0.30
MIN_SPEED_FACTOR = 1.0
CIRCLE_POINTS = 108              # More points make the displayed circle smoother
WAYPOINT_TOLERANCE = 0.10      # Arrival distance as a fraction of platform radius
PLATFORM_MARGIN = 0.10         # Keep trajectories away from the platform edge

HISTORY_SECONDS = 30
TRIM_PINS = SERVO_PINS
TRIM_LIMIT = 10


class Status(IntEnum):
    """Match the command and tracking status values used by the Arduino."""
    BALL_LOST = 0
    TRACKING = 1
    LEVEL = 2
    AUTOTUNE = 3
    AUTOLEVEL = 4
    SWEEP = 5
    SERVO_TRIM = 6
    CALIBRATE_SERVOS = 7


# These modes from the Status class send ONE packet then do not send more, i.e.,
# they act like a latch. All other modes spam packets with their Status type
HOLD_LEVEL_MODES = (Status.LEVEL, Status.SWEEP, Status.AUTOTUNE, Status.AUTOLEVEL)


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
        """Start camera capture and open serial unless preview mode is active."""
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
        """Queue the latest camera frame with its host timestamp."""
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
        """Send current commands and request level on shutdown."""
        try:
            import serial

            with serial.Serial(port_name, BAUD_RATE, write_timeout=0.1) as port:
                # Give the Arduino time to reset after opening the serial port.
                if self.stop.wait(2):
                    return
                self.messages.put(("serial", f"{port_name} connected"))
                last_trim = None
                last_mode_request = None
                last_level = None
                try:
                    while not self.stop.is_set():
                        try:
                            trim, command, last_level, mode_request = self.commands.get(timeout=0.05)
                        except Empty:
                            continue
                        if trim != last_trim:
                            port.write(trim)
                            last_trim = trim
                        # Deduplicate after sending so replacing a queued update can't lose the request.
                        if mode_request is None or mode_request != last_mode_request:
                            port.write(command)
                            last_mode_request = mode_request
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
    """Track progress along a circle or polygon in image pixels."""
    def __init__(self, points, shape):
        """Store the path and initialize progress at its first point."""
        self.points = points
        self.shape = shape
        self.segment = 0
        self.distance = 0.0

    def advance(self, platform, ball, target, dt):
        """Return the next target and whether the path is complete."""
        error = math.dist(ball, platform.normalize(target))
        if self.shape == "polygon":
            # Hold the current vertex until the ball reaches it.
            if error > WAYPOINT_TOLERANCE:
                return target, False
            if self.segment == len(self.points) - 1:
                return target, True
            self.segment += 1
            return self.points[self.segment], False

        if error >= TRACKING_PAUSE_ERROR:
            return target, False
        # Slow the target as the ball falls behind.
        factor = min(1.0, (TRACKING_PAUSE_ERROR - error) / (TRACKING_PAUSE_ERROR - TRACKING_SLOW_ERROR))
        speed = MIN_SPEED_FACTOR + (1 - MIN_SPEED_FACTOR) * factor
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
    """Return a pixel path for one circle, starting at the ball's angle."""
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

    else:
        raise NotImplementedError("Only circles are generated automatically")



class History:
    """Plot target-relative position and derivatives of the ball's position."""

    def __init__(self):
        """Initialize the plot buffer and derivative state."""
        self.samples = deque()
        self.reset_derivatives()

    def reset_derivatives(self):
        """Clear the previous position and velocity used for differences."""
        self.previous = None
        self.velocity = None

    def prune(self, now):
        """Remove samples older than the plot window."""
        while self.samples and self.samples[0][0] < now - HISTORY_SECONDS:
            self.samples.popleft()

    def add(self, now, ball, target):
        """Record position error and estimate ball velocity and acceleration."""
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
    """Maintain tracking state and prepare commands for the Arduino."""
    def __init__(self):
        """Initialize frame, target, history, and trajectory gains."""
        self.platform = self.detector = self.target = None
        self.frame = self.frame_time = self.detection = None
        self.path = None
        self.history = History()
        # Store editable gains in tenths to match the GUI step size.
        self.trajectory_gains = [round(value * 10) for value in TRAJECTORY_GAINS]

    def active_gains(self, manual_gains):
        """Return the editable gain set for the current tracking mode."""
        if self.path is not None and self.path.shape == "circle":
            return self.trajectory_gains
        return manual_gains

    def start_path(self, points, shape):
        """Create a trajectory and set its first point as the target."""
        self.path = Trajectory(points, shape)
        self.target = points[0]

    def process_frame(self, now, frame, detector):
        """Detect the ball, update the path and history, and report completion."""
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

    def send_command(self, hardware, now, action, gains, trims, mode_request_id):
        """Queue servo trims, the active command, and a shutdown level packet."""
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
        # Each button press gets its own ID, even if it selects the same mode again.
        mode_request = (status, mode_request_id) if status in HOLD_LEVEL_MODES else None
        timestamp = mode_request_id if status == Status.SWEEP else frame_ms
        tracking_gains = gains
        if status == Status.TRACKING:
            tracking_gains = tuple(value / 10 for value in self.active_gains(manual_gains))
        command = PACKET.pack(SYNC, status, *pins, *ball, *target, timestamp, *tracking_gains)
        level = PACKET.pack(SYNC, Status.LEVEL, *pins, 0, 0, 0, 0, frame_ms, *gains)
        put_latest(hardware.commands, (trim, command, level, mode_request))
