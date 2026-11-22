import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import mplcursors

from scipy.optimize import least_squares

import matplotlib
matplotlib.use("TkAgg")

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# ============================================================
# Mathematical model
# ============================================================

def calculate_ahat(msx, msy, theta0, nx, ny):
    """
    Calculate ahatx and ahaty.

    nx = -sin(thetax)
    ny = -sin(thetay)

    theta0 is in degrees.
    nx and ny are already calculated in the input data.
    """

    theta0_rad = np.deg2rad(theta0)

    nps = np.column_stack((nx, ny, np.ones(len(nx))))
    nps /= np.linalg.norm(nps, axis=1)[:, None]

    s = np.array([1.0, 0.0, msx])
    ns = np.array([-msx, -msy, 1.0])

    s /= np.linalg.norm(s)
    ns /= np.linalg.norm(ns)

    cross_ns_s = np.cross(ns, s)

    x0 = (
        s * np.cos(theta0_rad)
        + cross_ns_s * np.sin(theta0_rad)
    )

    y0 = (
        cross_ns_s * np.cos(theta0_rad)
        - s * np.sin(theta0_rad)
    )

    # Columns of R are x0, y0, ns
    R = np.column_stack((x0, y0, ns))

    # A removes the third component
    ahat = (R @ nps.T).T[:, :2]

    # Normalize the calculated 2D vector
    norms = np.linalg.norm(ahat, axis=1)

    norms = np.maximum(norms, 1e-12)
    ahat /= norms[:, None]

    return ahat[:, 0], ahat[:, 1]


def residual_function(guesses, ahatx, ahaty, nx, ny):
    """
    Return residuals for all selected samples.

    guesses = [msx, msy, theta0]
    """

    msx, msy, theta0 = guesses

    calc_x, calc_y = calculate_ahat(
        msx, msy, theta0, nx, ny
    )

    residual_x = ahatx - calc_x
    residual_y = ahaty - calc_y

    return np.concatenate((residual_x, residual_y))


# ============================================================
# Data parsing
# ============================================================

def parse_data(text):
    """
    Parse five whitespace-separated columns:

    t, ahatx, ahaty, -sin(thetax), -sin(thetay)

    Comma-separated values are also accepted.
    """

    rows = []

    for line in text.strip().splitlines():

        line = line.strip()

        if not line:
            continue

        values = line.replace(",", " ").split()

        if len(values) != 5:
            raise ValueError(
                f"Expected 5 columns, got {len(values)}:\n{line}"
            )

        rows.append([float(v) for v in values])

    if not rows:
        raise ValueError("No data was entered.")

    data = np.asarray(rows, dtype=float)

    if not np.all(np.isfinite(data)):
        raise ValueError("Data contains invalid values.")

    return data


# ============================================================
# R squared
# ============================================================

def calculate_r_squared(measured, calculated):
    """
    Standard coefficient of determination.
    """

    ss_res = np.sum((measured - calculated) ** 2)
    ss_tot = np.sum((measured - np.mean(measured)) ** 2)

    if ss_tot < 1e-15:
        return float("nan")

    return 1.0 - ss_res / ss_tot


# ============================================================
# GUI
# ============================================================

class FittingApplication:

    def __init__(self, root):

        self.root = root
        self.root.title("Ahat Levenberg-Marquardt Fitting")
        self.root.geometry("1450x950")

        self.data = None

        self.setup_interface()

    def setup_interface(self):

        # ----------------------------------------------------
        # Main layout
        # ----------------------------------------------------

        main = ttk.Frame(self.root, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        # ----------------------------------------------------
        # Data input
        # ----------------------------------------------------

        data_frame = ttk.LabelFrame(
            main,
            text="Input Data (5 columns)",
            padding=5
        )

        data_frame.pack(fill=tk.X)

        ttk.Label(
            data_frame,
            text="t    ahatx    ahaty    -sin(thetax)    -sin(thetay)"
        ).pack(anchor=tk.W)

        self.data_text = tk.Text(
            data_frame,
            height=7,
            width=110,
            font=("Consolas", 10)
        )

        self.data_text.pack(fill=tk.X)

        # ----------------------------------------------------
        # Parameters
        # ----------------------------------------------------

        settings = ttk.LabelFrame(
            main,
            text="Fitting Parameters",
            padding=10
        )

        settings.pack(fill=tk.X, pady=10)

        self.tstart_var = tk.StringVar(value="0")
        self.samples_var = tk.StringVar(value="100")
        self.spacing_var = tk.StringVar(value="1")
        self.iterations_var = tk.StringVar(value="1000")

        fields = [
            ("t_start:", self.tstart_var),
            ("Number of samples:", self.samples_var),
            ("Sample spacing:", self.spacing_var),
            ("Max LS evaluations:", self.iterations_var)
        ]

        for i, (label, variable) in enumerate(fields):

            ttk.Label(
                settings,
                text=label
            ).grid(row=0, column=2*i, padx=5, pady=5)

            ttk.Entry(
                settings,
                textvariable=variable,
                width=12
            ).grid(row=0, column=2*i+1, padx=5, pady=5)

        # ----------------------------------------------------
        # Ready button
        # ----------------------------------------------------

        ttk.Button(
            settings,
            text="READY / FIT",
            command=self.run_fit
        ).grid(row=1, column=0, columnspan=2, pady=10)

        # ----------------------------------------------------
        # Results
        # ----------------------------------------------------

        results = ttk.LabelFrame(
            main,
            text="Fitting Results",
            padding=10
        )

        results.pack(fill=tk.X, pady=5)

        self.results_label = ttk.Label(
            results,
            text="No fit performed yet.",
            font=("Consolas", 10),
            justify=tk.LEFT
        )

        self.results_label.pack(anchor=tk.W)

        # ----------------------------------------------------
        # Plots
        # ----------------------------------------------------

        plot_frame = ttk.Frame(main)
        plot_frame.pack(fill=tk.BOTH, expand=True)

        self.fig = Figure(
            figsize=(12, 8),
            dpi=100
        )

        self.ax1 = self.fig.add_subplot(211)
        self.ax2 = self.fig.add_subplot(212)

        self.ax1.set_title("ahatx")
        self.ax1.set_ylabel("ahatx")
        self.ax1.set_xlabel("t")
        self.ax1.grid(True)
        self.ax1.legend()

        self.ax2.set_title("ahaty")
        self.ax2.set_ylabel("ahaty")
        self.ax2.set_xlabel("t")
        self.ax2.grid(True)
        self.ax2.legend()

        self.fig.tight_layout()

        self.canvas = FigureCanvasTkAgg(
            self.fig,
            master=plot_frame
        )

        self.canvas.get_tk_widget().pack(
            fill=tk.BOTH,
            expand=True
        )

    # ========================================================
    # Main fitting routine
    # ========================================================

    def run_fit(self):

        try:

            # ------------------------------------------------
            # Read input data
            # ------------------------------------------------

            data = parse_data(
                self.data_text.get("1.0", tk.END)
            )

            t = data[:, 0]
            ahatx = data[:, 1]
            ahaty = data[:, 2]
            nx = data[:, 3]
            ny = data[:, 4]

            # Normalize -sin(theta) components
            norm = np.sqrt(nx**2 + ny**2)

            norm = np.maximum(norm, 1e-12)

            nx_normalized = nx / norm
            ny_normalized = ny / norm

            # ------------------------------------------------
            # Read fitting parameters
            # ------------------------------------------------

            tstart = float(self.tstart_var.get())

            num_samples = int(self.samples_var.get())

            spacing = int(self.spacing_var.get())

            max_iterations = int(
                self.iterations_var.get()
            )

            if num_samples < 2:
                raise ValueError(
                    "At least 2 samples are required."
                )

            if spacing < 1:
                raise ValueError(
                    "Sample spacing must be at least 1."
                )

            if max_iterations < 1:
                raise ValueError(
                    "Number of iterations must be positive."
                )

            # ------------------------------------------------
            # Select fitting samples
            # ------------------------------------------------

            # Find the first sample with t >= tstart

            start_index = np.searchsorted(
                t, tstart, side="left"
            )

            if start_index >= len(t):
                raise ValueError(
                    "t_start is beyond the available data."
                )

            # Select samples at the specified spacing

            indices = (
                start_index
                + np.arange(num_samples) * spacing
            )

            if indices[-1] >= len(t):
                raise ValueError(
                    "Not enough data for the requested "
                    "number of samples and spacing."
                )

            fit_ahatx = ahatx[indices]
            fit_ahaty = ahaty[indices]
            fit_nx = nx[indices]
            fit_ny = ny[indices]

            # ------------------------------------------------
            # Levenberg-Marquardt optimization
            # ------------------------------------------------

            # Initial guesses
            initial_guess = np.array([
                0.0,   # msx
                0.0,   # msy
                0.0    # theta0 (degrees)
            ])

            result = least_squares(
                residual_function,
                initial_guess,
                args=(
                    fit_ahatx,
                    fit_ahaty,
                    fit_nx,
                    fit_ny
                ),
                method="lm",
                max_nfev=max_iterations
            )

            msx, msy, theta0 = result.x

            # ------------------------------------------------
            # Calculate ahat for ALL original samples
            # ------------------------------------------------

            calculated_x, calculated_y = calculate_ahat(
                msx,
                msy,
                theta0,
                nx,
                ny
            )

            # ------------------------------------------------
            # Calculate R squared
            # ------------------------------------------------

            # Calculate R² ONLY on the samples used for fitting

            r2_x = calculate_r_squared(
                fit_ahatx, calculated_x[indices]
            )

            r2_y = calculate_r_squared(
                fit_ahaty, calculated_y[indices]
            )

            measured_combined = np.concatenate(
                (fit_ahatx, fit_ahaty)
            )

            calculated_combined = np.concatenate(
                (calculated_x[indices], calculated_y[indices])
            )

            r2_combined = calculate_r_squared(
                measured_combined, calculated_combined
            )

            # ------------------------------------------------
            # Update plots
            # ------------------------------------------------

            self.ax1.clear()
            self.ax2.clear()

            # ---------------- PLOT 1: X ----------------

            self.ax1.scatter(
                t, ahatx,
                color="blue",
                s=2,
                label="ahatx measured"
            )

            self.ax1.scatter(
                t,
                calculated_x,
                color="red",
                s=2,
                label="ahatx calculated"
            )

            self.ax1.scatter(
                t,
                nx_normalized,
                color="green",
                s=2,
                label="Normalized -sin(thetax)"
            )

            # Fitting samples
            self.ax1.scatter(
                t[indices],
                ahatx[indices],
                color="blue",
                marker="x",
                s=20,
                label="Fitting samples"
            )

            self.ax1.scatter(
                t[indices],
                nx_normalized[indices],
                color="green",
                marker="x",
                s=20,
                label="Fitting samples"
            )

            self.ax1.set_title("X Components")
            self.ax1.set_ylabel("Value")
            self.ax1.set_xlabel("t")
            self.ax1.grid(True)
            self.ax1.legend(loc="best")


            # ---------------- PLOT 2: Y ----------------

            self.ax2.scatter(
                t,
                ahaty,
                color="blue",
                s=2,
                label="ahaty measured"
            )

            self.ax2.scatter(
                t,
                calculated_y,
                color="red",
                s=2,
                label="ahaty calculated"
            )

            self.ax2.scatter(
                t,
                ny_normalized,
                color="green",
                s=2,
                label="Normalized -sin(thetay)"
            )

            # Fitting samples
            self.ax2.scatter(
                t[indices],
                ahaty[indices],
                color="blue",
                marker="x",
                s=20,
                label="Fitting samples"
            )

            self.ax2.scatter(
                t[indices],
                ny_normalized[indices],
                color="green",
                marker="x",
                s=20,
                label="Fitting samples"
            )

            self.ax2.set_title("Y Components")
            self.ax2.set_ylabel("Value")
            self.ax2.set_xlabel("t")
            self.ax2.grid(True)
            self.ax2.legend(loc="best")


            self.fig.tight_layout()

            # ------------------------------------------------
            # Interactive hover tooltips
            # ------------------------------------------------

            # Remove previous cursor if one exists
            if hasattr(self, "cursor"):
                self.cursor.remove()

            # Enable hovering over all plotted lines and points
            self.cursor = mplcursors.cursor(
                self.fig.axes,
                hover=True
            )

            @self.cursor.connect("add")
            def on_add(sel):
                x, y = sel.target

                sel.annotation.set_text(
                    f"t = {x:.6f}\n"
                    f"value = {y:.6f}"
                )

                sel.annotation.get_bbox_patch().set(
                    facecolor="white",
                    alpha=0.9
                )

            self.canvas.draw()

            # ------------------------------------------------
            # Display results
            # ------------------------------------------------

            result_text = (
                f"msx = {msx:.6f}    "
                f"msy = {msy:.6f}    "
                f"theta0 = {theta0:.4f} deg    "
                f"R² = {r2_combined:.6f}"
            )

            self.results_label.config(text=result_text)

        except Exception as e:

            messagebox.showerror(
                "Fitting Error",
                str(e)
            )


# ============================================================
# Start application
# ============================================================

if __name__ == "__main__":

    root = tk.Tk()

    app = FittingApplication(root)

    root.mainloop()
