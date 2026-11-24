import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk


class TimeSeriesPlotterApp:
    def __init__(self, root):
        self.root = root
        self.root.title("3-Column Time Series Plotter")
        self.root.geometry("1150x680")

        # Main split layout: Left (Input) | Right (Plot)
        paned = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        # --- Left Frame: Data Input ---
        left_frame = ttk.Frame(paned, width=320)
        paned.add(left_frame, weight=1)

        ttk.Label(
            left_frame,
            text="Paste Data (Time  Col2  Col3):",
            font=("Segoe UI", 10, "bold")
        ).pack(anchor="w", pady=(0, 4))

        # Text box with vertical scrollbar
        text_container = ttk.Frame(left_frame)
        text_container.pack(fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(text_container)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.text_box = tk.Text(
            text_container,
            width=34,
            font=("Consolas", 9),
            yscrollcommand=scrollbar.set,
            undo=True
        )
        self.text_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.text_box.yview)

        # Options & Buttons
        opts_frame = ttk.Frame(left_frame)
        opts_frame.pack(fill=tk.X, pady=(8, 4))

        self.show_markers_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            opts_frame,
            text="Show data point markers",
            variable=self.show_markers_var,
            command=self.plot_data
        ).pack(anchor="w")

        self.zero_line_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            opts_frame,
            text="Show y = 0 reference line",
            variable=self.zero_line_var,
            command=self.plot_data
        ).pack(anchor="w")

        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(fill=tk.X, pady=(6, 4))

        ttk.Button(
            btn_frame,
            text="Plot Data",
            command=self.plot_data
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))

        ttk.Button(
            btn_frame,
            text="Clear",
            command=self.clear_all
        ).pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.status_label = ttk.Label(
            left_frame,
            text="Ready. Paste rows and click 'Plot Data'.",
            foreground="#444444",
            wraplength=300
        )
        self.status_label.pack(anchor="w", pady=(4, 0))

        # --- Right Frame: Matplotlib Plot ---
        right_frame = ttk.Frame(paned)
        paned.add(right_frame, weight=3)

        self.fig = Figure(figsize=(7.5, 5.5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.grid(True, linestyle="--", alpha=0.5)
        self.ax.set_xlabel("Time (Column 1)")
        self.ax.set_ylabel("Value")
        self.ax.set_title("Data vs. Time")

        self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Interactive toolbar (Zoom, Pan, Home, Save PNG)
        toolbar = NavigationToolbar2Tk(self.canvas, right_frame)
        toolbar.update()

        # Keyboard shortcut: Ctrl+Enter to plot immediately
        self.root.bind("<Control-Return>", lambda event: self.plot_data())

    def parse_input(self, raw_text):
        times, col2, col3 = [], [], []
        skipped = 0

        for line in raw_text.strip().splitlines():
            line = line.strip()
            if not line:
                continue
            # Support whitespace, tabs, or commas
            parts = line.replace(",", " ").split()
            if len(parts) < 3:
                skipped += 1
                continue
            try:
                t = float(parts[0])
                v2 = float(parts[1])
                v3 = float(parts[2])
                times.append(t)
                col2.append(v2)
                col3.append(v3)
            except ValueError:
                skipped += 1  # Skips headers or malformed lines

        return times, col2, col3, skipped

    def plot_data(self):
        raw_text = self.text_box.get("1.0", tk.END)
        times, col2, col3, skipped = self.parse_input(raw_text)

        if not times:
            self.status_label.config(text="No valid 3-column numeric rows found.")
            return

        self.ax.clear()
        marker = "." if self.show_markers_var.get() else None

        self.ax.plot(times, col2, label="Column 2", linewidth=1.5, marker=marker, color="#1f77b4")
        self.ax.plot(times, col3, label="Column 3", linewidth=1.5, marker=marker, color="#d62728")

        if self.zero_line_var.get():
            self.ax.axhline(0, color="black", linewidth=0.8, linestyle=":")

        # Auto-scale tightly with a 5% vertical margin so small fluctuations stand out
        all_vals = col2 + col3
        y_min, y_max = min(all_vals), max(all_vals)
        span = y_max - y_min if y_max != y_min else 1.0
        self.ax.set_ylim(y_min - 0.05 * span, y_max + 0.05 * span)
        self.ax.set_xlim(min(times), max(times))

        self.ax.grid(True, linestyle="--", alpha=0.5)
        self.ax.set_xlabel("Time (Column 1)")
        self.ax.set_ylabel("Value")
        self.ax.set_title(f"Time Series ({len(times)} points)")
        self.ax.legend(loc="best")
        self.fig.tight_layout()
        self.canvas.draw()

        skip_note = f" ({skipped} skipped)" if skipped else ""
        self.status_label.config(
            text=(
                f"Plotted {len(times)} rows{skip_note}\n"
                f"Col 2: [{min(col2):.5f}, {max(col2):.5f}]\n"
                f"Col 3: [{min(col3):.5f}, {max(col3):.5f}]"
            )
        )

    def clear_all(self):
        self.text_box.delete("1.0", tk.END)
        self.ax.clear()
        self.ax.grid(True, linestyle="--", alpha=0.5)
        self.ax.set_xlabel("Time (Column 1)")
        self.ax.set_ylabel("Value")
        self.ax.set_title("Data vs. Time")
        self.canvas.draw()
        self.status_label.config(text="Cleared.")


if __name__ == "__main__":
    root = tk.Tk()
    app = TimeSeriesPlotterApp(root)
    root.mainloop()