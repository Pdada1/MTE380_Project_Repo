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


# Input columns: x, velx, intvelx, accx, time, thetax
COLUMN_NAMES = ["x", "velx", "intvelx", "accx", "time", "thetax"]

# Plot all X-axis signals, including position x.
X_PLOTS = ["x", "velx", "intvelx", "accx", "thetax"]


def parse_data(raw_text):
    """Parse rows containing exactly 6 numeric columns.

    Whitespace, tabs, commas, and semicolons are accepted as separators.
    Blank lines and lines beginning with # are ignored.
    """
    rows = []

    for line_number, line in enumerate(raw_text.splitlines(), start=1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue

        parts = line.replace(",", " ").replace(";", " ").split()

        if len(parts) != 6:
            raise ValueError(
                f"Line {line_number}: expected 6 columns, got {len(parts)}.\n"
                "Required order:\n"
                "x velx intvelx accx time thetax"
            )

        try:
            row = [float(value) for value in parts]
        except ValueError:
            raise ValueError(
                f"Line {line_number}: contains a non-numeric value. "
                "Remove headers and paste numeric data only."
            )

        if not np.all(np.isfinite(row)):
            raise ValueError(f"Line {line_number}: contains NaN or infinite values.")

        rows.append(row)

    if not rows:
        raise ValueError("No valid data rows were found.")

    data = np.asarray(rows, dtype=float)
    time = data[:, 4]

    if len(time) > 1 and np.any(np.diff(time) <= 0):
        raise ValueError("Time (column 5) must be in strictly ascending order.")

    return data


class TimeSeriesPlotterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Motion Data Plotter")
        self.root.geometry("1650x1050")
        self.root.minsize(1100, 750)

        self.axes = []
        self._build_interface()

    def _build_interface(self):
        paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        # Input panel
        input_frame = ttk.Frame(paned, padding=6)
        paned.add(input_frame, weight=1)

        ttk.Label(
            input_frame,
            text="Paste Data (6 columns)",
            font=("Segoe UI", 12, "bold"),
        ).pack(anchor="w", pady=(0, 5))

        ttk.Label(
            input_frame,
            text=(
                "Column order:\n"
                "x, velx, intvelx, accx, time, thetax\n\n"
                "Separate values with spaces, tabs, commas, or semicolons.\n"
                "One sample per line."
            ),
            justify="left",
            wraplength=330,
        ).pack(anchor="w", pady=(0, 8))

        self.input_box = ScrolledText(
            input_frame,
            wrap="none",
            font=("Consolas", 9),
            undo=True,
        )
        self.input_box.pack(fill=tk.BOTH, expand=True)

        self.show_markers_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            input_frame,
            text="Show data point markers",
            variable=self.show_markers_var,
            command=self.plot_data,
        ).pack(anchor="w", pady=(8, 0))

        self.zero_line_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            input_frame,
            text="Show y = 0 reference lines",
            variable=self.zero_line_var,
            command=self.plot_data,
        ).pack(anchor="w")

        buttons = ttk.Frame(input_frame)
        buttons.pack(fill=tk.X, pady=8)

        ttk.Button(
            buttons, text="Plot Data", command=self.plot_data
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))

        ttk.Button(
            buttons, text="Clear", command=self.clear_all
        ).pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.status_label = ttk.Label(
            input_frame,
            text="Ready. Paste data and click 'Plot Data'.",
            wraplength=330,
            justify="left",
        )
        self.status_label.pack(anchor="w", pady=(2, 0))

        # Plot panel
        plot_frame = ttk.Frame(paned)
        paned.add(plot_frame, weight=4)

        self.figure = Figure(figsize=(13, 12), dpi=90)
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        toolbar_frame = ttk.Frame(plot_frame)
        toolbar_frame.pack(fill=tk.X)
        self.toolbar = NavigationToolbar2Tk(
            self.canvas, toolbar_frame, pack_toolbar=False
        )
        self.toolbar.update()
        self.toolbar.pack(side=tk.LEFT)

        self._draw_empty_layout()
        self.root.bind("<Control-Return>", lambda event: self.plot_data())

    def _draw_empty_layout(self):
        self.figure.clear()
        self.axes = []

        self.figure.suptitle(
            "X-Axis Signals",
            fontsize=13,
        )

        for row, name in enumerate(X_PLOTS, start=1):
            ax = self.figure.add_subplot(5, 1, row)
            ax.set_title(name)
            ax.set_ylabel(name)
            ax.set_xlabel("Time")
            ax.grid(True, linestyle="--", alpha=0.4)
            self.axes.append(ax)

        self.figure.tight_layout(rect=(0, 0, 1, 0.97))
        self.canvas.draw_idle()

    def plot_data(self):
        raw_text = self.input_box.get("1.0", tk.END)

        try:
            data = parse_data(raw_text)
        except ValueError as error:
            messagebox.showerror("Data Error", str(error))
            return

        values = {name: data[:, i] for i, name in enumerate(COLUMN_NAMES)}
        time = values["time"]

        self.figure.clear()
        self.axes = []
        self.figure.suptitle(
            f"Motion Data vs Time ({len(time)} samples)",
            fontsize=13,
        )

        marker = "." if self.show_markers_var.get() else None

        for row, name in enumerate(X_PLOTS, start=1):
            ax = self.figure.add_subplot(5, 1, row)
            ax.plot(
                time,
                values[name],
                linewidth=1.3,
                marker=marker,
                markersize=3,
                label=name,
            )

            if self.zero_line_var.get():
                ax.axhline(0, color="black", linewidth=0.7, linestyle=":")

            ax.set_title(name)
            ax.set_ylabel(name)
            ax.set_xlabel("Time")
            ax.grid(True, linestyle="--", alpha=0.4)
            ax.legend(loc="best", fontsize=8)
            ax.tick_params(axis="both", labelsize=8)
            self.axes.append(ax)

        self.figure.tight_layout(rect=(0, 0, 1, 0.97))
        self.canvas.draw_idle()

        self.status_label.config(
            text=(
                f"Plotted {len(time)} samples.\n"
                f"Time range: {time[0]:.6g} to {time[-1]:.6g}\n"
                "Plotted X-axis signals: velx, intvelx, accx, thetax"
            )
        )

    def clear_all(self):
        self.input_box.delete("1.0", tk.END)
        self._draw_empty_layout()
        self.status_label.config(text="Cleared.")


if __name__ == "__main__":
    root = tk.Tk()
    app = TimeSeriesPlotterApp(root)
    root.mainloop()
