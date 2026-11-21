"""Log ball position between the firmware's RUN_START and RUN_END lines."""
import csv
from pathlib import Path


class RunLogger:
    def __init__(self, folder="freq_logs"):
        self.folder = Path(folder)
        self.run = None   # (axis, freq_hz, amp_deg, t_start) while logging
        self.rows = []

    def on_line(self, t, line):
        parts = line.split(",")
        if parts[0] == "RUN_START" and len(parts) == 4:
            self.run = (parts[1], float(parts[2]), float(parts[3]), t)
            self.rows = []
        elif parts[0] == "RUN_END" and self.run is not None:
            self.save()
            self.run = None

    def on_ball(self, t, ball):
        if self.run is not None and ball is not None:
            self.rows.append((t - self.run[3], ball[0], ball[1]))

    def save(self):
        axis, freq, amp, _ = self.run
        self.folder.mkdir(exist_ok=True)
        path = self.folder / f"{axis}_{freq:.2f}Hz.csv"
        with path.open("w", newline="") as f:
            f.write(f"# axis={axis} freq_hz={freq} amp_deg={amp}\n")
            writer = csv.writer(f)
            writer.writerow(["t_s", "x_radii", "y_radii"])
            writer.writerows(self.rows)