import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText
import numpy as np

import matplotlib
matplotlib.use("TkAgg")

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg,
    NavigationToolbar2Tk,
)


TIGHT_DOTTED = (0, (1.5, 1.5))

# Column numbers, starting at zero:
# 0  t
# 1  x
# 2  y
# 3  -sin(thetax)
# 4  -sin(thetay)
# 5  acc_x
# 6  acc_y
# 7  -sin(thetax_shifted)
# 8  -sin(thetay_shifted)
# 9  status
# 10 RMS_deg
# 11 msx
# 12 msy
# 13 theta0
# 14 fit_us


def parse_data(text):
    """Parse pasted whitespace-, comma-, or semicolon-separated data."""
    rows = []

    for line_no, line in enumerate(text.splitlines(), 1):
        line = line.strip()

        if not line or line.startswith("#"):
            continue

        parts = line.replace(",", " ").replace(";", " ").split()

        try:
            row = [float(v) for v in parts]
        except ValueError:
            raise ValueError(
                f"Invalid number on line {line_no}. "
                "Remove column headers and paste numeric data only."
            )

        if len(row) < 15:
            raise ValueError(
                f"Line {line_no} has {len(row)} columns; "
                "the complete dataset requires 15 columns."
            )

        rows.append(row[:15])

    if len(rows) < 2:
        raise ValueError("Please paste at least two rows of data.")

    data = np.asarray(rows, dtype=float)

    if not np.all(np.isfinite(data)):
        raise ValueError("The data contains NaN or infinite values.")

    if np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError("Time must be in strictly ascending order.")

    return data


def common_scaling_factor(sine_arrays, acc_arrays):
    """Use one common scale factor across both X and Y sine curves and accelerations."""
    sine_max = max(np.max(np.abs(arr)) for arr in sine_arrays)
    acc_max = max(np.max(np.abs(arr)) for arr in acc_arrays)

    if sine_max == 0:
        return 1.0

    return acc_max / sine_max


class PlotterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("All Data vs Time — X and Y")
        self.root.geometry("1550x1000")
        self.root.minsize(1050, 700)

        self.root.columnconfigure(0, weight=0, minsize=330)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        self.build_input_panel()
        self.build_plot_panel()

    def build_input_panel(self):
        panel = ttk.Frame(self.root, padding=10)
        panel.grid(row=0, column=0, sticky="nsew")
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(2, weight=1)

        ttk.Label(
            panel,
            text="Paste Data",
            font=("Segoe UI", 14, "bold")
        ).grid(row=0, column=0, sticky="w", pady=(0, 8))

        description = (
            "Paste the complete 15-column dataset.\n\n"
            "t, x, y, -sin(thetax), -sin(thetay), "
            "acc_x, acc_y, -sin(thetax_shifted), "
            "-sin(thetay_shifted), status, RMS_deg, "
            "msx, msy, theta0, fit_us"
        )

        ttk.Label(
            panel,
            text=description,
            wraplength=300,
            justify="left"
        ).grid(row=1, column=0, sticky="ew", pady=(0, 8))

        self.input_box = ScrolledText(
            panel,
            wrap="none",
            font=("Consolas", 10),
            undo=True
        )
        self.input_box.grid(
            row=2, column=0, sticky="nsew", pady=(0, 10)
        )

        ttk.Button(
            panel, text="Plot All Data",
            command=self.plot_data
        ).grid(row=3, column=0, sticky="ew", pady=3)

        ttk.Button(
            panel, text="Clear Input",
            command=self.clear_input
        ).grid(row=4, column=0, sticky="ew", pady=3)

        self.status_label = ttk.Label(
            panel, text="Ready", wraplength=300
        )
        self.status_label.grid(
            row=5, column=0, sticky="ew", pady=(10, 0)
        )

    def build_plot_panel(self):
        panel = ttk.Frame(self.root, padding=(0, 8, 8, 8))
        panel.grid(row=0, column=1, sticky="nsew")
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(0, weight=1)

        # All graphs share a common time axis.
        # X-related plots are in the left column;
        # Y-related plots are in the right column.
        self.figure = Figure(figsize=(12, 18), dpi=90)

        self.axes = []
        self.canvas = FigureCanvasTkAgg(
            self.figure, master=panel
        )
        self.canvas.get_tk_widget().grid(
            row=0, column=0, sticky="nsew"
        )

        toolbar_frame = ttk.Frame(panel)
        toolbar_frame.grid(row=1, column=0, sticky="ew")

        self.toolbar = NavigationToolbar2Tk(
            self.canvas, toolbar_frame, pack_toolbar=False
        )
        self.toolbar.update()
        self.toolbar.pack(side="left")

    def clear_input(self):
        self.input_box.delete("1.0", "end")
        self.status_label.config(text="Input cleared.")

    def plot_data(self):
        try:
            data = parse_data(
                self.input_box.get("1.0", "end")
            )
        except ValueError as error:
            messagebox.showerror("Data Error", str(error))
            return

        t = data[:, 0]

        # Extract columns
        x = data[:, 1]
        y = data[:, 2]
        sx = data[:, 3]
        sy = data[:, 4]
        ax = data[:, 5]
        ay = data[:, 6]
        sx_shift = data[:, 7]
        sy_shift = data[:, 8]
        status = data[:, 9]
        rms = data[:, 10]
        msx = data[:, 11]
        msy = data[:, 12]
        theta0 = data[:, 13]

        # Common scale factor for both X and Y
        f_scale = common_scaling_factor(
            [sx, sx_shift, sy, sy_shift],
            [ax, ay]
        )

        self.figure.clear()

        # Four rows, two columns:
        # row 1: positions
        # row 2: combined X and Y plots
        # row 3: msx and msy
        # row 4: theta0 (left) and status / RMS_deg (right)
        grid = self.figure.add_gridspec(
            4, 2,
            hspace=0.58,
            wspace=0.28,
            left=0.07,
            right=0.98,
            top=0.96,
            bottom=0.06
        )

        self.axes = []

        # --------------------------------------------------------
        # ROW 1: POSITION
        # --------------------------------------------------------

        self.add_plot(
            grid[0, 0],
            [("x", t, x, {})],
            "x"
        )

        self.add_plot(
            grid[0, 1],
            [("y", t, y, {})],
            "y"
        )

        # --------------------------------------------------------
        # ROW 2: COMBINED GRAPHS
        # --------------------------------------------------------

        self.add_plot(
            grid[1, 0],
            [
                ("acc_x", t, ax, {}),
                (f"-sin(thetax) × {f_scale:.4g}", t, sx * f_scale, {"linestyle": TIGHT_DOTTED}),
                (f"-sin(thetax_shifted) × {f_scale:.4g}", t, sx_shift * f_scale, {}),
            ],
            "Acceleration-scale units"
        )

        self.add_plot(
            grid[1, 1],
            [
                ("acc_y", t, ay, {}),
                (f"-sin(thetay) × {f_scale:.4g}", t, sy * f_scale, {"linestyle": TIGHT_DOTTED}),
                (f"-sin(thetay_shifted) × {f_scale:.4g}", t, sy_shift * f_scale, {}),
            ],
            "Acceleration-scale units"
        )

        # --------------------------------------------------------
        # ROW 3: FIT PARAMETERS
        # --------------------------------------------------------

        self.add_plot(
            grid[2, 0],
            [("msx", t, msx, {})],
            "msx"
        )

        self.add_plot(
            grid[2, 1],
            [("msy", t, msy, {})],
            "msy"
        )

        # --------------------------------------------------------
        # ROW 4: DIAGNOSTICS
        # --------------------------------------------------------

        self.add_plot(
            grid[3, 0],
            [("theta0", t, theta0, {})],
            "theta0"
        )

        self.add_plot(
            grid[3, 1],
            [
                ("status", t, status, {}),
                ("RMS_deg", t, rms, {}),
            ],
            "Value"
        )

        self.canvas.draw_idle()

        self.status_label.config(
            text=f"Plotted {len(t)} rows."
        )

    def add_plot(self, position, series, ylabel):
        sharex = self.axes[0] if self.axes else None
        ax = self.figure.add_subplot(position, sharex=sharex)
        self.axes.append(ax)

        for label, x_vals, y_vals, plot_kwargs in series:
            ax.plot(x_vals, y_vals, label=label, linewidth=1.5, **plot_kwargs)

        ax.set_ylabel(ylabel, fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=7)
        ax.tick_params(axis="both", labelsize=8)
        ax.set_xlabel("Time", fontsize=8)


if __name__ == "__main__":
    root = tk.Tk()
    app = PlotterApp(root)
    root.mainloop()