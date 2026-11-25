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

# Supported input formats:
# Compact (5 columns): t, RMS_deg, msx, msy, theta0 (no status)
# Compact (6 columns): t, fit_status, RMS_deg, msx, msy, theta0
# Original format (14+ columns): status, RMS_deg, msx, msy, theta0
# are columns 9, 10, 11, 12, and 13.

def parse_data(text):
    """Parse compact or original whitespace/comma/semicolon-separated logs."""
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

        if len(row) == 5:
            # t, RMS_deg, msx, msy, theta0; no status available.
            t, rms, msx, msy, theta0 = row
            status = np.nan
        elif len(row) == 6:
            # t, fit_status, RMS_deg, msx, msy, theta0
            t, status, rms, msx, msy, theta0 = row
        elif len(row) >= 14:
            # Original full log format: status is column 9.
            t, status, rms, msx, msy, theta0 = (
                row[0], row[9], row[10], row[11], row[12], row[13]
            )
        else:
            raise ValueError(
                f"Line {line_no} has {len(row)} columns. Expected 5 columns "
                "(t, RMS_deg, msx, msy, theta0), 6 columns "
                "(t, fit_status, RMS_deg, msx, msy, theta0), or the original "
                "14+ column format."
            )

        rows.append([t, status, rms, msx, msy, theta0])

    if len(rows) < 2:
        raise ValueError("Please paste at least two rows of data.")

    data = np.asarray(rows, dtype=float)
    # Status may be NaN only for the 5-column format, where no status was logged.
    if not np.all(np.isfinite(data[:, [0, 2, 3, 4, 5]])):
        raise ValueError("The data contains NaN or infinite values in a plotted column.")
    if np.any(np.diff(data[:, 0]) <= 0):
        raise ValueError("Time must be in strictly ascending order.")

    return data


class PlotterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Fit Parameters vs Time")
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
            "Paste compact data or the original full log.\n\n"
            "Compact: t, fit_status, RMS_deg, msx, msy, theta0\n"
            "(or 5 columns without fit_status)\n\n"
            "The original 14+ column format is also supported. "
            "Graphs fit_status (when present), RMS_deg, msx, msy, and theta0 against t."
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
            panel, text="Plot Selected Data",
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

        # The requested parameters and fit status share the time axis.
        self.figure = Figure(figsize=(12, 12), dpi=90)

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

        # parse_data normalizes inputs to six columns:
        # t, fit_status (NaN if unavailable), RMS_deg, msx, msy, theta0.
        t = data[:, 0]
        status = data[:, 1]
        rms = data[:, 2]
        msx = data[:, 3]
        msy = data[:, 4]
        theta0 = data[:, 5]
        has_status = np.all(np.isfinite(status))

        self.figure.clear()
        self.figure.suptitle("Fit Parameters vs Time", fontsize=13)

        plot_count = 5 if has_status else 4
        grid = self.figure.add_gridspec(
            plot_count, 1,
            hspace=0.42,
            left=0.10,
            right=0.97,
            top=0.93,
            bottom=0.08
        )

        self.axes = []
        plot_row = 0

        if has_status:
            self.add_plot(
                grid[plot_row, 0],
                [("fit_status", t, status, {"drawstyle": "steps-post", "marker": "."})],
                "fit_status"
            )
            plot_row += 1

        self.add_plot(
            grid[plot_row, 0],
            [("RMS_deg", t, rms, {})],
            "RMS_deg"
        )
        plot_row += 1

        self.add_plot(
            grid[plot_row, 0],
            [("msx", t, msx, {})],
            "msx"
        )
        plot_row += 1

        self.add_plot(
            grid[plot_row, 0],
            [("msy", t, msy, {})],
            "msy"
        )
        plot_row += 1

        self.add_plot(
            grid[plot_row, 0],
            [("theta0", t, theta0, {})],
            "theta0"
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
