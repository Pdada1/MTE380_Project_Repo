"""Log ball position between the firmware's RUN_START and RUN_END lines."""
import csv
from pathlib import Path


class RunLogger:
    def __init__(self, folder="freq_logs"):
        self.folder = Path(folder)
        self.run = None        # (axis, freq_hz, amp_deg, t_start) while logging
        self.rows = []         # (t_s, position change along the swept axis)
        self.start = None      # ball position (x, y) when the run began
        self.last_ball = None  # most recent ball position, used as the start point

    def on_line(self, t, line):
        parts = line.split(",")
        if parts[0] == "RUN_START" and len(parts) == 4:
            self.run = (parts[1], float(parts[2]), float(parts[3]), t)
            self.rows = []
            # Start point: where the ball was resting just before the sweep began.
            self.start = self.last_ball
        elif parts[0] == "RUN_END" and self.run is not None:
            self.save()
            self.run = None
        elif parts[0] == "SWEEP_ABORTED":
            self.run = None
            self.rows = []

    def on_ball(self, t, ball):
        if ball is None:
            return
        self.last_ball = ball
        if self.run is None:
            return
        if self.start is None:
            self.start = ball  # no earlier sample: use the first one in the run
        axis = 0 if self.run[0] == "x" else 1
        self.rows.append((t - self.run[3], ball[axis] - self.start[axis]))

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