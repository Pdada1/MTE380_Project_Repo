"""Log ball position between the firmware's RUN_START and RUN_END lines."""
import csv
import math
from pathlib import Path

ABORT_RADIUS = 0.8  # abort a run if the ball gets farther than this from the platform center (radii)


class RunLogger:
    def __init__(self, folder="freq_logs", abort_radius=ABORT_RADIUS):
        self.folder = Path(folder)
        self.abort_radius = abort_radius
        self.run = None        # (axis, freq_hz, amp_deg, t_start) while logging
        self.rows = []         # (t_s, position change along the swept axis)
        self.diag = []         # (t_s, x, y, radius) absolute ball position, both axes
        self.start = None      # ball position (x, y) when the run began
        self.last_ball = None  # most recent ball position, used as the start point
        self.abort_requested = False  # on_ball already asked for an abort during this run
        self.abort_reason = None      # why on_ball asked for it
        self.abort_message = None     # reason to show with the latest SWEEP_ABORTED, or None

    def on_line(self, t, line):
        parts = line.split(",")
        if parts[0] == "RUN_START" and len(parts) == 4:
            self.run = (parts[1], float(parts[2]), float(parts[3]), t)
            self.rows = []
            self.diag = []
            self.abort_requested = False
            self.abort_reason = None
            # Start point: where the ball was resting just before the sweep began.
            self.start = self.last_ball
        elif parts[0] == "RUN_END" and self.run is not None:
            self.save()
            self.save_diag(aborted=False)
            self.run = None
        elif parts[0] == "SWEEP_ABORTED":
            # Keep the partial run for diagnosis; it is not valid for the frequency fit.
            if self.run is not None:
                self.save_diag(aborted=True)
            self.abort_message = self.abort_reason
            self.abort_reason = None
            self.abort_requested = False
            self.run = None
            self.rows = []
            self.diag = []

    def on_ball(self, t, ball):
        """Record a ball sample. Returns True once per run when it should be aborted."""
        if ball is None:
            return False
        self.last_ball = ball
        if self.run is None:
            return False
        if self.start is None:
            self.start = ball  # no earlier sample: use the first one in the run
        axis = 0 if self.run[0] == "x" else 1
        elapsed = t - self.run[3]
        self.rows.append((elapsed, ball[axis] - self.start[axis]))
        radius = math.hypot(ball[0], ball[1])
        self.diag.append((elapsed, ball[0], ball[1], radius))

        if radius > self.abort_radius and not self.abort_requested:
            self.abort_requested = True
            self.abort_reason = (f"Ball left the safe zone at {elapsed:.1f} s "
                                 f"(x={ball[0]:+.2f}, y={ball[1]:+.2f}, {radius:.2f} radii).")
            return True
        return False

    def save(self):
        axis, freq, amp, _ = self.run
        self.folder.mkdir(exist_ok=True)
        path = self.folder / f"{axis}_{freq:.2f}Hz.csv"
        if path.exists():
            # Re-running a frequency replaces its log; keep the previous one just in case.
            path.replace(path.with_name(path.name + ".bak"))
        with path.open("w", newline="") as f:
            f.write(f"# axis={axis} freq_hz={freq} amp_deg={amp}\n")
            writer = csv.writer(f)
            writer.writerow(["t_s", f"{axis}_change_radii"])
            writer.writerows(self.rows)

    def save_diag(self, aborted):
        """Write absolute x/y/radius for the whole run, including the axis being held.

        The name ends in _diag.csv or _diag_aborted.csv, so the x_*Hz.csv / y_*Hz.csv
        glob used by the frequency-response fit never picks these up.
        """
        axis, freq, amp, _ = self.run
        self.folder.mkdir(exist_ok=True)
        suffix = "_diag_aborted" if aborted else "_diag"
        path = self.folder / f"{axis}_{freq:.2f}Hz{suffix}.csv"
        with path.open("w", newline="") as f:
            f.write(f"# axis={axis} freq_hz={freq} amp_deg={amp} aborted={int(aborted)}\n")
            if aborted and self.abort_reason:
                f.write(f"# reason={self.abort_reason}\n")
            writer = csv.writer(f)
            writer.writerow(["t_s", "x_radii", "y_radii", "radius"])
            writer.writerows(self.diag)