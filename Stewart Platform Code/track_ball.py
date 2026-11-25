"""Ball tracking, trajectory motion, measurement history, and Arduino I/O."""

from collections import deque
from enum import IntEnum
import math
from queue import Empty, Full, Queue
import struct
from threading import Event, Thread
import time

import cv2

from circle_feedforward import CircleFeedforward
from helper import SERVO_PINS, open_camera


SYNC = bytes((0x0B, 0x00, 0x0B))
# Wire fields in order: sync, status, pins, ball XY, target XY, timestamp, PID gains.
PACKET = struct.Struct("<3sB3B4fI3f")
ARDUINO_SERIAL_PORT = "COM6"
BAUD_RATE = 115200
DEFAULT_GAINS = (6.0, 1.0, 4.7)
TRAJECTORY_GAINS = (6.0, 0.8, 4.8)  # Kp, Ki, Kd for circle and polygon tracking

# Speeds are in platform radii per second. Errors are fractions of the radius.
CIRCLE_SPEED = 0.35
CIRCLE_SLOW_ERROR = 0.15
CIRCLE_PAUSE_ERROR = 0.40
CIRCLE_POINTS = 72
CIRCLE_MAX_RADIUS = 0.80  # Fraction of the platform radius.

POLYGON_VERTEX_TOLERANCE = 0.10

HISTORY_SECONDS = 30
TRIM_LIMIT = 60


# @MM
DEFAULT_RISE_TIME_MS = 1000
RISE_TIME_LIMITS = (400, 1000) # ms, accepted entered range in the GUI
DEFAULT_OVERSHOOT_PCT = 10
OVERSHOOT_LIMITS = (5, 50) # percent; the firmware ignores entered values outside this
DEFAULT_MAX_ERR_SS = 0.1
MAX_ERR_SS_LIMITS = (0.01, 0.2) # Relative to ball radii 

class Status(IntEnum):
    """Match the command and tracking status values used by the Arduino."""
    BALL_LOST = 0
    TRACKING = 1
    LEVEL = 2
    AUTOTUNE = 3
    AUTOLEVEL = 4
    SWEEP = 5
    SERVO_TRIM = 6
    AUTOROTATION = 7
    AUTOTUNE_MM = 8
    SWEEP_DATA = 9
    FREQ_RUN = 10   # one frequency on one axis: ball x = axis code, ball y = frequency in Hz
    IDENTIFY_SERVOS = 11
    AUTOTILT = 12
    FIXED_TARGET = 13
    CIRCLE_TRACKING = 14


# These modes from the Status class send ONE packet then do not send more, i.e.,
# they act like a latch. All other modes spam packets with their Status type
HOLD_LEVEL_MODES = (Status.LEVEL, Status.SWEEP, Status.AUTOTUNE, Status.AUTOLEVEL, Status.FREQ_RUN)
# Modes where the firmware runs the frequency sweep and needs the ball stream to hold the other axis.
SWEEP_MODES = (Status.AUTOTUNE, Status.FREQ_RUN)
SWEEP_AXIS_CODE = {"x": 1.0, "y": 2.0}  # FREQ_RUN axis codes


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
        if debug and not preview:
            self.serial_output = Queue(maxsize=16)
            Thread(target=self.print_serial_output, daemon=True).start()
        self.camera_thread = Thread(target=self.capture, daemon=True)
        self.camera_thread.start()
        self.serial_thread = None
        self.firmware = Queue()
        self.proceed = Event()
        self.rerun = Event()
        self.abort = Event()
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

    def print_serial_output(self):
        """Keep terminal output off the serial command thread."""
        while not self.stop.is_set():
            try:
                output = self.serial_output.get(timeout=0.1)
            except Empty:
                continue
            print(output, end="", flush=True)

    def transmit(self, port_name, debug):
        """Send current commands, read Arduino lines, and request level on shutdown."""
        try:
            import serial

            last_mode_request = None
            while not self.stop.is_set():
                try:
                    with serial.Serial(port_name, BAUD_RATE, write_timeout=0.1) as port:
                        # Give the Arduino time to reset after opening the serial port.
                        if self.stop.wait(2):
                            return
                        port.reset_input_buffer()
                        self.messages.put(("serial", f"{port_name} connected"))
                        last_trim = None
                        last_level = None
                        rx = b""
                        try:
                            while not self.stop.is_set():
                                if port.in_waiting:
                                    rx += port.read(port.in_waiting)
                                    *lines, rx = rx.split(b"\n")
                                    for line in lines:
                                        text = line.decode("ascii", errors="replace").strip()
                                        if text:
                                            self.firmware.put((time.monotonic(), text))
                                            if debug:
                                                put_latest(self.serial_output, text + "\n")

                                # Human pressed the "go" key: PROCEED_CHAR (0x0A in the firmware).
                                if self.proceed.is_set():
                                    self.proceed.clear()
                                    port.write(b"\x0a")
                                # Redo key: the firmware offers the previous frequency again.
                                if self.rerun.is_set():
                                    self.rerun.clear()
                                    port.write(b"r")
                                if self.abort.is_set():
                                    self.abort.clear()
                                    port.write(b"x")
                                try:
                                    trim, command, last_level, mode_request, stream = self.commands.get(timeout=0.005)
                                except Empty:
                                    continue
                                if trim != last_trim:
                                    port.write(trim)
                                    last_trim = trim
                                # Deduplicate after sending so replacing a queued update can't lose the request.
                                if mode_request is None or mode_request != last_mode_request:
                                    port.write(command)
                                    last_mode_request = mode_request
                                if stream is not None:
                                    port.write(stream)
                        finally:
                            if self.stop.is_set():
                                port.write(b"x")
                                if last_level is not None:
                                    port.write(last_level)
                except serial.SerialTimeoutException:
                    if self.stop.is_set():
                        return
                    # Reopen the port to clear a possibly partial packet.
                    self.messages.put(("serial", f"{port_name} write timeout. Reconnecting..."))
                    if last_mode_request is not None and last_mode_request[0] == Status.LEVEL:
                        last_mode_request = None
                    self.stop.wait(0.25)
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
        self.circle = CircleFeedforward() if shape == "circle" else None

    def advance(self, platform, ball, target, dt):
        """Return the next target and whether the path is complete."""
        error = math.dist(ball, platform.normalize(target))
        if self.shape == "polygon":
            return self.advance_polygon(target, error)

        # Fade to zero as tracking error grows, with no jump at either threshold.
        factor = (CIRCLE_PAUSE_ERROR - error) / (CIRCLE_PAUSE_ERROR - CIRCLE_SLOW_ERROR)
        factor = max(0.0, min(1.0, factor))
        speed = factor * factor * (3 - 2 * factor)
        # Circle chords all have the same length. Slow down before the final target.
        chord = math.dist(self.points[0], self.points[1])
        remaining = (len(self.points) - 1 - self.segment) * chord - self.distance
        if remaining <= 1e-6:
            self.circle.stop()
            return self.points[-1], True
        distance = platform.radius * self.circle.advance(
            dt, CIRCLE_SPEED * speed, CIRCLE_SPEED, remaining / platform.radius
        )
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
        self.circle.stop()
        return self.points[-1], True


    def advance_polygon(self, target, error):
        """Hold each vertex as the target until the ball reaches it."""
        if error >= POLYGON_VERTEX_TOLERANCE:
            return target, False
        if self.segment == len(self.points) - 1:
            return target, True

        self.segment += 1
        return self.points[self.segment], False


def make_trajectory(platform, start_px, shape):
    """Return a pixel path for one circle, starting at the ball's angle."""
    ox, oy = platform.origin
    x, y = start_px
    radius = platform.radius
    max_radius = radius * CIRCLE_MAX_RADIUS
    distance = math.hypot(x - ox, y - oy)

    if shape == "circle":
        # Circle is centered on the platform, at the ball's current radius.
        r = min(distance, max_radius)
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
        self.freq = None
        self.reset_derivatives()

    def reset_derivatives(self):
        """Clear the previous position and velocity used for differences."""
        self.previous = None
        self.velocity = None

    def prune(self, now):
        """Remove samples older than the plot window."""
        while self.samples and self.samples[0][0] < now - HISTORY_SECONDS:
            self.samples.popleft()

    def add(self, now, ball, target, freq=None):
        """Record position error and estimate ball velocity and acceleration."""
        if freq is None:
            freq=self.freq
        if ball is None:
            self.reset_derivatives()
            self.samples.append((now, (None, None, None,freq)))
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
        self.samples.append((now, (error, velocity, acceleration, freq)))


class Tracker:
    """Maintain tracking state and prepare commands for the Arduino."""
    def __init__(self):
        """Initialize frame, target, history, and trajectory gains."""
        self.platform = self.detector = self.target = None
        self.frame = self.frame_time = self.detection = None
        self.path = None
        self.fixed_target = False
        self.circle_id = 0
        self.history = History()
        self.ball = None
        # Store editable gains in tenths to match the GUI step size.
        self.trajectory_gains = [round(value * 10) for value in TRAJECTORY_GAINS]
        self.rise_time_ms = DEFAULT_RISE_TIME_MS # @MM
        self.max_overshoot_pct = DEFAULT_OVERSHOOT_PCT # @MM
        self.max_error_ss = DEFAULT_MAX_ERR_SS # @MM

    def active_gains(self, manual_gains):
        """Return the editable gain set for the current tracking mode."""
        if self.path is not None and self.path.shape in ("circle"):
            return self.trajectory_gains
        return manual_gains

    def set_fixed_target(self, point):
        """Mark an explicitly selected target for firmware integral reset."""
        self.target = point
        self.fixed_target = True

    def start_path(self, points, shape):
        """Create a trajectory and set its first point as the target."""
        self.path = Trajectory(points, shape)
        if self.path.circle is not None:
            self.circle_id = self.circle_id % 255 + 1
        self.fixed_target = False
        self.target = points[0]

    def process_frame(self, now, frame, detector):
        """Detect the ball, update the path and history, and report completion."""
        frame_gap = now - self.frame_time if self.frame_time is not None else 0
        dt = min(frame_gap, 0.1)
        self.frame, self.frame_time = frame, now
        self.detection = detector.detect(frame) if detector is not None else None
        if self.platform is None:
            return False
        ball = self.platform.normalize(self.detection[0]) if self.detection is not None else None
        self.ball = ball
        complete = False
        if self.path is not None:
            if self.path.circle is not None and (ball is None or dt == 0 or frame_gap > 0.25):
                self.path.circle.stop()
            elif ball is not None:
                self.target, complete = self.path.advance(self.platform, ball, self.target, dt)
        self.history.add(now, ball, self.platform.normalize(self.target))
        return complete

    def send_command(self, hardware, now, action, gains, trims, mode_request_id, freq_run=(0.0, 0.0), identify_pin=None, mapped_pins=None):
        """Queue servo trims, the active command, and a shutdown level packet."""
        pins = self.platform.pins if self.platform is not None else SERVO_PINS
        manual_gains = gains
        gains = tuple(value / 10 for value in manual_gains)
        frame_ms = int((self.frame_time if self.frame_time is not None else now) * 1000) & 0xFFFFFFFF
        ball = (0.0, 0.0)
        target = self.platform.normalize(self.target) if self.platform is not None else (0.0, 0.0)
        status = Status.BALL_LOST
        if isinstance(action, Status) and action not in (Status.AUTOROTATION, Status.AUTOTILT, Status.AUTOLEVEL, Status.AUTOTUNE_MM): # @MM
            status = action
        elif action == "platform" and identify_pin is not None:
            status = Status.IDENTIFY_SERVOS
            pins = (identify_pin, 0, 0)
        elif action in ("ball", "platform"):
            status = Status.LEVEL
            if mapped_pins is not None:
                pins = mapped_pins
        elif (self.platform is not None and self.detection is not None
              and now - self.frame_time <= 0.25):
            status = action if action in (Status.AUTOROTATION, Status.AUTOTILT, Status.AUTOLEVEL,
                                          Status.AUTOTUNE_MM) else Status.TRACKING
            if status == Status.TRACKING and self.fixed_target:
                status = Status.FIXED_TARGET
            elif status == Status.TRACKING and self.path is not None and self.path.circle is not None:
                status = Status.CIRCLE_TRACKING
            ball = self.platform.normalize(self.detection[0])
        if status == Status.FREQ_RUN:
            ball = freq_run  # (axis code, Hz) rides in the ball fields, which are unused in this mode
        level_pins = mapped_pins or (self.platform.pins if self.platform is not None else SERVO_PINS)
        
        # time domain parameters for PID autotuning is sent through the trim packet gains
        if action == Status.AUTOTUNE_MM:
            trim = PACKET.pack(SYNC, Status.SERVO_TRIM, *level_pins, *trims, 0.0, 0,
                               self.rise_time_ms, self.max_overshoot_pct, self.max_error_ss)
        else:
            trim = PACKET.pack(SYNC, Status.SERVO_TRIM, *level_pins, *trims, 0.0, 0, *DEFAULT_GAINS)
        # Each button press gets its own ID, even if it selects the same mode again.
        mode_request = (status, mode_request_id) if status in HOLD_LEVEL_MODES else None
        timestamp = mode_request_id if status == Status.SWEEP else frame_ms
        tracking_gains = gains
        if status in (Status.TRACKING, Status.CIRCLE_TRACKING, Status.AUTOROTATION, Status.AUTOTILT, Status.AUTOLEVEL):
            tracking_gains = tuple(value / 10 for value in self.active_gains(manual_gains))
        if status == Status.CIRCLE_TRACKING:
            # Keep the frame ID in the low 24 bits and the circle ID in the top byte.
            timestamp = (frame_ms & 0xFFFFFF) | (self.circle_id << 24)
            kp, _, kd = tracking_gains
            target = self.path.circle.command_target(target, kp, kd)
        command = PACKET.pack(SYNC, status, *pins, *ball, *target, timestamp, *tracking_gains)
        level = PACKET.pack(SYNC, Status.LEVEL, *level_pins, 0, 0, 0, 0, frame_ms, *gains)
        # During the autotune sweep the firmware keeps the ball in place from this stream.
        stream = None
        if (status in SWEEP_MODES and self.platform is not None and self.detection is not None
                and now - self.frame_time <= 0.25):
            stream = PACKET.pack(SYNC, Status.SWEEP_DATA, *pins,
                                 *self.platform.normalize(self.detection[0]), *target, frame_ms, *gains)
        put_latest(hardware.commands, (trim, command, level, mode_request, stream))
