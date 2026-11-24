"""Graphical interface for the MTE380 Stewart Platform."""

import argparse
import math
from queue import Empty
import time
import tkinter as tk
from tkinter import ttk

import cv2

from calibrate import calibration_objects, find_platform, identify_marker, read_config, sample_ball_color, save_config
from helper import Platform, draw_ball
from track_ball import (
    ARDUINO_SERIAL_PORT, DEFAULT_GAINS, HISTORY_SECONDS, HOLD_LEVEL_MODES, TRIM_LIMIT,
    Hardware, Status, Tracker, make_trajectory,
)


# Label, help text, callback, optional callback arguments.
ACTIONS = (
    ("Calibrate Platform", "Find the red, green, and blue platform markers.", "start_platform_calibration"),
    ("Calibrate Ball", "Sample the ball color in the camera view.", "start_ball_calibration"),
    ("Autotune Motors", "Run the X and Y frequency sweeps, then hold level.", "start_mode", Status.AUTOTUNE),
    ("Autorotation Mode", "Track the ball independent of the platform orientation.", "start_mode", Status.AUTOROTATION),
    ("Autotilt Mode", "Track with automatic tilt compensation.", "start_mode", Status.AUTOTILT),
    ("Autolevel Platform", "Track the ball with the separate autolevel PID controller.", "start_mode", Status.AUTOLEVEL),
    ("Level Platform", "Move to the neutral platform position.", "start_mode", Status.LEVEL),
    ("Draw Circle", "Follow a circle starting at the ball.", "start_path", "circle"),
    ("Draw Polygon", "Click vertices, then press Go to follow the polygon.", "start_path", "polygon"),
    ("Sweep Platform", "Sweep X and Y once, then hold level.", "start_mode", Status.SWEEP),
    ("Quit", "Close the interface.", "close"),
)
MODE_LABELS = {args[0]: label for label, _, method, *args in ACTIONS if method == "start_mode"}


def draw_history(history, canvas, now, index):
    """Plot the selected metric for both axes over the latest 30 seconds."""
    history.prune(now)
    width, height = canvas.winfo_width(), canvas.winfo_height()
    if width < 100 or height < 75:
        return
    canvas.delete("all")
    left, right, top, bottom = 42, width - 12, 14, height - 23
    data = [(t, values[index]) for t, values in history.samples]
    limit = max((abs(v) for _, xy in data if xy is not None for v in xy), default=0)
    limit = max(limit, (0.05, 0.1, 1.0)[index])
    for fraction, label in ((0, "-30 s"), (0.5, "-15 s"), (1, "Now")):
        x = left + fraction * (right - left)
        canvas.create_line(x, top, x, bottom, fill="#324052")
        canvas.create_text(x, bottom + 11, text=label, fill="#a0afc1")
    for fraction, value in ((0, limit), (0.5, 0), (1, -limit)):
        y = top + fraction * (bottom - top)
        canvas.create_line(left, y, right, y, fill="#324052")
        canvas.create_text(left - 5, y, text=f"{value:.2g}", anchor="e", fill="#a0afc1")
    for axis, color in enumerate(("#48c9ee", "#ffa35c")):
        line = []
        previous_time = None
        for timestamp, xy in data:
            # Break the line across missed detections or long sample gaps.
            if xy is None or (previous_time is not None and timestamp - previous_time > 0.25):
                if len(line) >= 4:
                    canvas.create_line(*line, fill=color, width=2)
                line = []
            if xy is not None:
                x = right - (now - timestamp) / HISTORY_SECONDS * (right - left)
                y = top + (1 - xy[axis] / limit) * (bottom - top) / 2
                line.extend((x, y))
            previous_time = timestamp
        if len(line) >= 4:
            canvas.create_line(*line, fill=color, width=2)


class App:
    """Manage GUI controls, calibration actions, and display updates."""
    def __init__(self, root, args):
        """Initialize the interface, load calibration, and start the update loop."""
        self.root = root
        self.preview = args.preview
        self.config = {}
        self.tracker = Tracker()
        self.action = None
        self.tracking_mode = Status.TRACKING
        self.polygon_points = []
        self.candidate = None
        self.gains = [round(value * 10) for value in DEFAULT_GAINS]
        self.trims = [0, 0, 0]
        self.display_transform = None
        self.photo = None
        self.job = None
        self.last_graph = 0
        self.last_sent = 0
        self.mode_request_id = 0
        self.serial_ready = False
        self.identify_started = 0
        self.identify_baseline = None
        self.scan_baseline = None
        self.scan_channel = 0
        self.identify_confirmed = False
        self.identify_preview = None
        self.identified_pins = {}
        self.message = tk.StringVar(value="Starting camera...")
        self.connection = tk.StringVar(value="Preview" if args.preview else f"Connecting to {args.port}...")
        self.camera_error = None
        self.build_ui()
        try:
            self.config = read_config()
            self.load_calibration()
        except (KeyError, TypeError, ValueError, OSError) as error:
            self.config = {}
            self.notice(f"Calibration could not be loaded: {error}")
        self.hardware = Hardware(args.preview, args.port, args.serial_debug)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<Escape>", lambda event: self.cancel_action())
        self.root.bind("c", lambda event: self.center_target())
        self.tick()

    def build_ui(self):
        """Lay out the actions, camera view, controls, and history graphs."""
        self.root.title("MTE380 Stewart Platform")
        self.root.geometry("1310x760")
        self.root.minsize(1000, 690)
        self.root.columnconfigure(1, weight=3)
        self.root.columnconfigure(2, weight=2)
        self.root.rowconfigure(0, weight=1)

        actions = ttk.LabelFrame(self.root, text="Actions", padding=10)
        actions.grid(row=0, column=0, sticky="ns", padx=(10, 5), pady=10)
        actions.columnconfigure(0, weight=1)
        for row, (label, description, method, *args) in enumerate(ACTIONS):
            callback = getattr(self, method)
            ttk.Button(actions, text=label, command=lambda fn=callback, a=args: fn(*a)).grid(
                row=2 * row, column=0, sticky="ew")
            ttk.Label(actions, text=description, wraplength=175, justify="left").grid(
                row=2 * row + 1, column=0, sticky="w", pady=(1, 4))

        middle = ttk.Frame(self.root)
        middle.grid(row=0, column=1, sticky="nsew", padx=5, pady=10)
        middle.columnconfigure(0, weight=1)
        middle.rowconfigure(0, weight=1)
        self.camera = tk.Canvas(middle, background="#111923", width=560, height=420, highlightthickness=0)
        self.camera.grid(row=0, column=0, sticky="nsew")
        self.image_id = self.camera.create_image(0, 0, anchor="nw")
        self.camera.bind("<Button-1>", self.click_camera)

        status = ttk.Frame(middle)
        status.grid(row=1, column=0, sticky="ew", pady=5)
        ttk.Label(status, textvariable=self.connection).pack(anchor="w")
        ttk.Label(status, textvariable=self.message, wraplength=420).pack(anchor="w")
        self.action_controls = ttk.Frame(middle)
        self.action_controls.grid(row=2, column=0, sticky="ew", pady=(0, 5))
        self.save_button = ttk.Button(self.action_controls, text="Save Calibration", command=self.save_calibration)
        self.confirm_circle_button = ttk.Button(self.action_controls, text="Confirm Circle",
                                                command=self.confirm_platform_circle)
        self.redetect_circle_button = ttk.Button(self.action_controls, text="Re-detect Circle",
                                                 command=self.redetect_platform_circle)
        self.reset_button = ttk.Button(self.action_controls, text="Reset Samples", command=self.reset_candidate)
        self.go_button = ttk.Button(self.action_controls, text="Go", command=self.start_polygon)
        self.cancel_button = ttk.Button(self.action_controls, text="Cancel Action", command=self.cancel_action)

        self.cancel_tracking_button = ttk.Button(self.action_controls,
                                                 command=self.cancel_tracking_mode)

        controls = ttk.LabelFrame(middle, text="Controls", padding=8)
        controls.grid(row=3, column=0, sticky="ew")
        self.refresh_pid_controls = self.control_row(
            controls, 0, "PID gains", ("Kp", "Ki", "Kd"),
            lambda: self.tracker.active_gains(self.gains), 1, 10, 0, None,
        )
        self.control_row(controls, 1, "Servo trim", ("R", "G", "B"),
                         lambda: self.trims, 1, 1, -TRIM_LIMIT, TRIM_LIMIT)

        graphs = ttk.LabelFrame(self.root, text="Ball History · 30 seconds", padding=8)
        graphs.grid(row=0, column=2, sticky="nsew", padx=(5, 10), pady=10)
        legend = tk.Frame(graphs, bg="#182331")
        legend.pack(fill="x")
        for label, color in (("X", "#48c9ee"), ("Y", "#ffa35c")):
            tk.Label(legend, text=label, fg=color, bg="#182331", padx=9).pack(side="left")
        self.graphs = []
        for title in ("Position relative to target (radii)", "Ball velocity (radii/s)", "Ball acceleration (radii/s²)"):
            ttk.Label(graphs, text=title).pack(anchor="w", pady=(7, 2))
            canvas = tk.Canvas(graphs, width=285, height=150, bg="#182331", highlightthickness=0)
            canvas.pack(fill="both", expand=True)
            self.graphs.append(canvas)

    def control_row(self, parent, row, label, names, get_values, step, scale, low, high):
        """Build +/- controls and return a callback to refresh their values."""
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=(0, 5))
        labels = []
        unit = "°" if scale == 1 else ""
        precision = 0 if scale == 1 else 1

        def refresh():
            """Read the current values into the displayed labels."""
            for name, value, widget in zip(names, get_values(), labels):
                widget.configure(text=f"{name} {value / scale:.{precision}f}{unit}")

        def adjust(index, delta):
            """Apply one step to the selected value within its limits."""
            values = get_values()
            value = max(low, values[index] + delta)
            values[index] = min(high, value) if high is not None else value
            refresh()

        for i in range(len(names)):
            group = ttk.Frame(parent)
            group.grid(row=row, column=i + 1, sticky="ew", padx=2, pady=3)
            parent.columnconfigure(i + 1, weight=1)
            value_label = ttk.Label(group)
            labels.append(value_label)

            ttk.Button(group, text="−", width=2,
                       command=lambda j=i: adjust(j, -step)).pack(side="left")
            value_label.pack(side="left", padx=2)
            ttk.Button(group, text="+", width=2,
                       command=lambda j=i: adjust(j, step)).pack(side="left")
        refresh()
        return refresh

    def load_calibration(self):
        """Apply saved calibration and center the target when possible."""
        self.tracker.detector, self.tracker.platform = calibration_objects(self.config, self.tracker.detector, self.tracker.platform)
        if "calibration" in self.config and "servos" in self.config:
            self.tracker.target = self.tracker.platform.origin

    def notice(self, message):
        """Update the action message below the camera."""
        self.message.set(message)

    def set_action(self, action, message):
        """Switch actions, clear the previous path, and update the controls."""
        self.action = action
        self.polygon_points.clear()
        self.tracker.path = None
        self.refresh_pid_controls()
        self.candidate = None
        for button in (self.save_button, self.confirm_circle_button, self.redetect_circle_button,
                       self.reset_button, self.go_button, self.cancel_button):
            button.pack_forget()
        if action == "ball":
            self.save_button.state(["disabled"])
            self.save_button.pack(side="left", padx=(0, 4))
        if action == "ball":
            self.reset_button.pack(side="left", padx=(0, 4))
        if action == "polygon":
            self.go_button.state(["disabled"])
            self.go_button.pack(side="left", padx=(0, 4))
        if action is not None:
            self.cancel_button.pack(side="left")
        self.notice(message)

    def cancel_action(self):
        """End the current action and return to manual targeting."""
        if self.action == "platform":
            self.finish_identification("Platform calibration canceled.")
        else:
            self.set_action(None, "Click the camera view to set a target.")

    def center_target(self):
        """Set the target to the platform center and cancel any path."""
        if self.tracker.platform is not None:
            self.tracker.target = self.tracker.platform.origin
            self.cancel_action()

    def start_ball_calibration(self):
        """Start collecting ball color samples from camera clicks."""
        self.set_action("ball", "Click several spots on the ball, then save.")

    def start_platform_calibration(self):
        """Start detecting RGB markers for platform calibration."""
        if not self.serial_ready:
            self.notice("Connect the Arduino before calibrating the platform.")
            return
        self.mode_request_id = (self.mode_request_id + 1) & 0xFFFFFFFF
        self.identify_started = time.monotonic()
        self.identify_baseline = None
        self.scan_baseline = None
        self.scan_channel = 0
        self.identify_confirmed = False
        self.identify_preview = None
        self.identified_pins.clear()
        self.set_action("platform", "Looking for the red, green, and blue platform markers.")

    def finish_identification(self, message):
        self.mode_request_id = (self.mode_request_id + 1) & 0xFFFFFFFF
        self.set_action(Status.LEVEL, message)

    def identify_pin(self):
        if self.action == "platform" and self.identify_confirmed:
            if len(self.identified_pins) < 3 and self.scan_channel < 16:
                # Channel 16 tells firmware to put every channel at the lower bound.
                return 16 if self.scan_baseline is None else self.scan_channel
        return None

    def mapped_servo_pins(self):
        if self.action == "platform" and len(self.identified_pins) == 3:
            return tuple(self.identified_pins[i] for i in range(3))
        return None

    def redetect_platform_circle(self):
        if self.action != "platform" or self.identify_confirmed:
            return
        self.start_platform_calibration()

    def confirm_platform_circle(self):
        if self.action != "platform" or self.identify_baseline is None or self.identify_confirmed:
            return
        self.identify_confirmed = True
        self.identify_started = time.monotonic()
        self.confirm_circle_button.pack_forget()
        self.redetect_circle_button.pack_forget()
        self.notice("Measuring the scan baseline...")

    def update_platform_calibration(self, now, frame):
        if self.candidate is not None or (self.identify_baseline is not None and not self.identify_confirmed):
            return
        elapsed = now - self.identify_started
        if elapsed < 0.75:
            return

        geometry = find_platform(frame)
        if geometry is None:
            if elapsed >= 5:
                self.finish_identification("RGB markers could not be detected. Calibration not saved.")
            return

        pins = self.mapped_servo_pins()
        if pins is not None:
            self.candidate = (*geometry, pins)
            self.identify_preview = geometry
            self.save_button.state(["!disabled"])
            self.save_button.pack(side="left", padx=(0, 4), before=self.cancel_button)
            self.notice("Review the pin mapping, then select Save Calibration or Cancel.")
            return

        if self.identify_baseline is None:
            self.identify_baseline = geometry
            self.confirm_circle_button.pack(side="left", padx=(0, 4), before=self.cancel_button)
            self.redetect_circle_button.pack(side="left", padx=(0, 4), before=self.cancel_button)
            self.notice("Confirm the circle or re-detect it.")
            return

        self.identify_preview = geometry
        self.identify_started = time.monotonic()
        if self.scan_baseline is None:
            self.scan_baseline = geometry
        else:
            marker = identify_marker(self.scan_baseline, geometry)
            if marker is not None and marker not in self.identified_pins:
                self.identified_pins[marker] = self.scan_channel
            self.scan_channel += 1

        next_pin = self.identify_pin()
        if self.mapped_servo_pins() is not None:
            self.mode_request_id = (self.mode_request_id + 1) & 0xFFFFFFFF
            self.notice("Servo pins identified. Returning to neutral.")
        elif next_pin is None:
            self.finish_identification("Could not identify all three servo channels. Calibration not saved.")
        else:
            self.notice(f"Identifying servo pin {next_pin}...")

    def reset_candidate(self):
        """Discard the unsaved calibration and disable saving."""
        self.candidate = None
        self.save_button.state(["disabled"])

    def save_calibration(self):
        """Save and apply the candidate calibration, then clear graph history."""
        if self.candidate is None or self.tracker.frame is None:
            return
        try:
            config = save_config(self.config, self.tracker.frame, self.action, self.candidate)
        except OSError as error:
            self.notice(f"Could not save calibration: {error}")
            return
        target = self.tracker.target
        self.config = config
        self.load_calibration()
        if self.action == "ball" and target is not None:
            self.tracker.target = target
        self.tracker.detection = None
        self.tracker.history.samples.clear()
        self.tracker.history.reset_derivatives()
        self.set_action(None, "Calibration saved. Click the camera view to set a target.")

    def cancel_tracking_mode(self):
        """Return to normal PID without interrupting the current action."""
        self.tracking_mode = Status.TRACKING
        self.cancel_tracking_button.pack_forget()

    def start_mode(self, mode):
        """Select a firmware mode and identify each new hold-level request."""
        message = MODE_LABELS[mode]
        if mode in (Status.AUTOROTATION, Status.AUTOTILT):
            self.tracking_mode = mode
            name = "Autorotation" if mode == Status.AUTOROTATION else "Autotilt"
            self.cancel_tracking_button.configure(text=f"Cancel {name}")
            self.cancel_tracking_button.pack(side="right")
            self.notice(f"{name} selected." if not self.preview else f"{name} selected (preview).")
            return
        if mode in HOLD_LEVEL_MODES:
            self.mode_request_id = (self.mode_request_id + 1) & 0xFFFFFFFF
            message += " · holds level" if mode == Status.LEVEL else " · holds level afterward"
        if self.preview:
            message += " · preview"
        self.set_action(mode, message)

    def start_path(self, shape):
        """Collect polygon vertices or start a circle from the current ball."""
        if shape == "polygon":
            if self.tracker.platform is None:
                self.notice("Calibrate the platform before choosing polygon points.")
                return
            self.set_action("polygon", "Click at least three polygon vertices, then press Go.")
            return
        if (self.tracker.platform is None or self.tracker.detection is None
                or time.monotonic() - self.tracker.frame_time > 0.25):
            self.notice("Calibrate the platform and bring the ball into view first.")
            return
        points = make_trajectory(self.tracker.platform, self.tracker.detection[0], shape)
        if not points:
            self.notice(f"Cannot fit a {shape} at the ball's current position.")
            return
        self.set_action(shape, f"Drawing {shape}.")
        self.tracker.start_path(points, shape)
        self.refresh_pid_controls()

    def click_camera(self, event):
        """Map a canvas click to a ball sample or a platform target."""
        if self.tracker.frame is None or self.display_transform is None:
            return
        left, top, scale_x, scale_y = self.display_transform
        x, y = int((event.x - left) / scale_x), int((event.y - top) / scale_y)
        height, width = self.tracker.frame.shape[:2]
        if not (0 <= x < width and 0 <= y < height and event.x >= left and event.y >= top):
            return
        if self.action == "ball":
            self.candidate = sample_ball_color(self.tracker.frame, self.candidate, x, y)
            self.save_button.state(["!disabled"])
        elif self.action != "platform" and self.tracker.platform is not None:
            if math.dist((x, y), self.tracker.platform.origin) > self.tracker.platform.radius:
                self.notice("Choose a target inside the platform circle.")
                return
            if self.action == "polygon":
                if self.tracker.path is None:
                    self.add_polygon_point((x, y))
                return
            self.tracker.target = (x, y)
            self.cancel_action()

    def add_polygon_point(self, point):
        """Add a vertex without changing the current target."""
        if point in self.polygon_points:
            self.notice("Choose a different polygon point.")
            return
        self.polygon_points.append(point)
        count = len(self.polygon_points)
        if count >= 3:
            self.go_button.state(["!disabled"])
            self.notice(f"{count} vertices selected. Add another or press Go.")
        else:
            self.notice(f"{count} of 3 minimum vertices selected.")

    def start_polygon(self):
        """Visit the selected vertices in order and close the polygon."""
        if self.action != "polygon" or self.tracker.path is not None or len(self.polygon_points) < 3:
            return
        (ax, ay), (bx, by), *remaining = self.polygon_points
        if all((bx - ax) * (y - ay) == (by - ay) * (x - ax) for x, y in remaining):
            self.notice("Add a vertex off the line before pressing Go.")
            return

        self.tracker.start_path([*self.polygon_points, self.polygon_points[0]], "polygon")
        self.go_button.state(["disabled"])
        self.notice("Following the polygon back to point 1.")

    def process_frame(self, now, frame):
        """Update calibration previews and pass the frame to the tracker."""
        if self.action == "platform":
            self.update_platform_calibration(now, frame)

        detector = self.tracker.detector
        if self.action == "ball" and self.candidate is not None:
            detector = self.candidate
        if self.tracker.process_frame(now, frame, detector):
            self.set_action(None, "Trajectory complete.")

    def show_frame(self):
        """Draw overlays and fit the camera image to the canvas."""
        frame = self.tracker.frame.copy()
        platform = self.tracker.platform
        marker_colors = ((0, 0, 255), (0, 255, 0), (255, 0, 0))
        if self.action == "platform" and self.identify_baseline is not None:
            platform = Platform(*self.identify_baseline)
        if platform is not None:
            platform.draw(frame)
            for point, color in zip(platform.servo_points, marker_colors):
                cv2.circle(frame, tuple(point), 4, color, -1)
        if self.action == "platform" and self.identify_baseline is not None:
            cv2.drawMarker(frame, tuple(map(round, self.identify_baseline[0])),
                           (255, 255, 0), cv2.MARKER_CROSS, 16, 2)
            if self.identify_preview is not None:
                cv2.drawMarker(frame, tuple(map(round, self.identify_preview[0])),
                               (255, 0, 255), cv2.MARKER_TILTED_CROSS, 16, 2)
            if self.candidate is not None:
                _, _, marker_points, pins = self.candidate
                for color, (x, y), pin in zip(marker_colors, marker_points, pins):
                    label = str(pin)
                    cv2.putText(frame, label, (x + 9, y - 7),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4, cv2.LINE_AA)
                    cv2.putText(frame, label, (x + 9, y - 7),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
        points = self.tracker.path.points if self.tracker.path is not None else self.polygon_points
        if self.tracker.path is None and len(points) >= 3:
            points = [*points, points[0]]
        for a, b in zip(points, points[1:]):
            cv2.line(frame, tuple(map(round, a)), tuple(map(round, b)), (0, 165, 255), 2)
        for i, point in enumerate(self.polygon_points, start=1):
            cv2.circle(frame, point, 5, (0, 165, 255), -1)
            cv2.putText(frame, str(i), (point[0] + 7, point[1] - 7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1, cv2.LINE_AA)
        if self.tracker.target is not None and self.action != "platform":
            cv2.drawMarker(frame, tuple(map(round, self.tracker.target)), (0, 255, 0))
        draw_ball(frame, self.tracker.detection)

        h, w = frame.shape[:2]
        canvas_w, canvas_h = self.camera.winfo_width(), self.camera.winfo_height()
        scale = min(canvas_w / w, canvas_h / h)
        width, height = max(1, round(w * scale)), max(1, round(h * scale))
        left, top = (canvas_w - width) // 2, (canvas_h - height) // 2
        rgb = cv2.cvtColor(cv2.resize(frame, (width, height)), cv2.COLOR_BGR2RGB)
        # Encode the frame in a format Tk can load directly.
        ppm = f"P6\n{width} {height}\n255\n".encode("ascii") + rgb.tobytes()
        self.photo = tk.PhotoImage(data=ppm, format="PPM")
        self.camera.coords(self.image_id, left, top)
        self.camera.itemconfigure(self.image_id, image=self.photo)
        self.display_transform = left, top, width / w, height / h

    def tick(self):
        """Consume camera updates, queue commands, and refresh the displays."""
        while True:
            try:
                source, message = self.hardware.messages.get_nowait()
            except Empty:
                break
            if source == "camera":
                self.camera_error = message
            else:
                self.connection.set(message)
                self.serial_ready = message.endswith(" connected")
        try:
            sample = self.hardware.frames.get_nowait()
        except Empty:
            sample = None
        if sample is not None:
            first_frame = self.tracker.frame is None
            self.process_frame(*sample)
            if first_frame and self.action is None:
                self.notice("Click the camera view to set a target." if self.tracker.platform and self.tracker.detector
                            else "Calibrate the ball and platform to begin tracking.")
            self.show_frame()
        now = time.monotonic()
        if sample is not None or now - self.last_sent >= 0.08:
            if not self.preview:
                action = self.action
                if action == "platform" and not self.identify_confirmed:
                    action = Status.BALL_LOST
                elif not isinstance(action, Status) and action not in ("ball", "platform"):
                    if self.tracking_mode != Status.TRACKING:
                        action = self.tracking_mode
                self.tracker.send_command(self.hardware, now, action, self.gains, self.trims, self.mode_request_id,
                                          self.identify_pin(), self.mapped_servo_pins())
            self.last_sent = now
        if now - self.last_graph >= 0.10:
            for i, canvas in enumerate(self.graphs):
                draw_history(self.tracker.history, canvas, now, i)
            self.last_graph = now
        if self.camera_error is not None:
            if self.action == "platform":
                self.finish_identification("Camera stopped. Platform calibration not saved.")
            self.notice(self.camera_error)
        self.job = self.root.after(15, self.tick)

    def close(self):
        """Stop GUI updates and signal the hardware workers to stop."""
        if self.job is not None:
            self.root.after_cancel(self.job)
        self.hardware.stop.set()
        if self.hardware.serial_thread is not None:
            self.hardware.serial_thread.join(timeout=0.25)
        self.root.destroy()


def main():
    """Parse startup options and run the Tk event loop."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preview", action="store_true", help="Run the interface without Arduino serial.")
    parser.add_argument("--port", default=ARDUINO_SERIAL_PORT, help="Arduino serial port.")
    parser.add_argument("--serial-debug", action="store_true", help="Print Arduino serial output.")
    args = parser.parse_args()
    root = tk.Tk()
    App(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
