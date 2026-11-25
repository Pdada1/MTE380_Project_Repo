import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import mplcursors

import matplotlib
matplotlib.use("TkAgg")

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# ============================================================
# Mathematical model
# ============================================================

DEFAULT_AZ_CONST = 5.89


def build_vector_pairs(ax, ay, nx, ny, az_const=DEFAULT_AZ_CONST):
    """
    Build the vector pairs used by Wahba's problem.

    nps = (-sin(thetax), -sin(thetay), 1) / |...|

    a   = (ax, ay, -az)
    az  = (c - sqrt(c^2 - 4(ax^2 + ay^2))) / 2,   c = 5.89 by default

    np  = (a x (-ay, ax, 0)) / |a x (-ay, ax, 0)|

    Returns (np_vectors, nps_vectors), each of shape (N, 3).
    """

    ax = np.asarray(ax, dtype=float)
    ay = np.asarray(ay, dtype=float)

    nps = np.column_stack((nx, ny, np.ones(len(nx))))
    nps /= np.linalg.norm(nps, axis=1)[:, None]

    discriminant = az_const**2 - 4.0 * (ax**2 + ay**2)

    if np.any(discriminant < 0):
        raise ValueError(
            "az is undefined for some samples: need "
            f"ax^2 + ay^2 <= {az_const**2 / 4:.4f}."
        )

    az = (az_const - np.sqrt(discriminant)) / 2.0

    a = np.column_stack((ax, ay, -az))
    perp = np.column_stack((-ay, ax, np.zeros(len(ax))))

    n_plane = np.cross(a, perp)
    lengths = np.linalg.norm(n_plane, axis=1)

    if np.any(lengths < 1e-12):
        raise ValueError(
            "np is undefined for some samples "
            "(ax = ay = 0 gives a zero cross product)."
        )

    n_plane /= lengths[:, None]

    return n_plane, nps


def solve_wahba(np_vectors, nps_vectors, weights=None):
    """
    Wahba's problem, SVD method:

        minimize  sum_i w_i |np_i - R nps_i|^2

    Returns the proper rotation matrix R (det = +1).
    """

    if weights is None:
        weights = np.ones(len(np_vectors))

    B = (np_vectors * weights[:, None]).T @ nps_vectors

    U, _, Vt = np.linalg.svd(B)

    d = np.sign(np.linalg.det(U) * np.linalg.det(Vt))

    if d == 0:
        d = 1.0

    return U @ np.diag([1.0, 1.0, d]) @ Vt


def rotation_from_ms(msx, msy):
    """
    Build R from msx, msy with theta0 fixed at 0.
    Columns of R are x0 = s, y0 = ns x s, and ns.
    """

    s = np.array([1.0, 0.0, msx])
    ns = np.array([-msx, -msy, 1.0])

    s /= np.linalg.norm(s)
    ns /= np.linalg.norm(ns)

    return np.column_stack((s, np.cross(ns, s), ns))


def ms_from_rotation(R):
    """
    msx, msy from the third column of R: ns = (-msx, -msy, 1)/|...|.
    """

    ns = R[:, 2]

    if ns[2] <= 1e-12:
        raise ValueError(
            "The fitted rotation has ns_z <= 0, so it cannot be "
            "written with ns = (-msx, -msy, 1)/norm."
        )

    return -ns[0] / ns[2], -ns[1] / ns[2]


def _ry(b):
    c, s = np.cos(b), np.sin(b)
    return np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def fit_ms_wahba(np_vectors, nps_vectors, max_iter=500, tol=1e-14):
    """
    Direct two-variable nonlinear least-squares fit of msx and msy,
    with theta0 fixed at 0.

    Minimize the stacked 3D vector residuals:

        r_i = np_i - R(msx, msy) @ nps_i

    where R is constructed by rotation_from_ms().

    Returns (msx, msy, R_theta0_zero, n_iter, converged).
    """
    from scipy.optimize import least_squares

    np_vectors = np.asarray(np_vectors, dtype=float)
    nps_vectors = np.asarray(nps_vectors, dtype=float)

    def residuals(parameters):
        msx, msy = parameters
        R = rotation_from_ms(msx, msy)
        # Each row is a vector, so multiply by R.T to apply R to
        # the corresponding column-vector representation.
        return (np_vectors - nps_vectors @ R.T).ravel()

    result = least_squares(
        residuals,
        x0=np.array([0.0, 0.0]),
        method="trf",
        max_nfev=max_iter,
        ftol=tol,
        xtol=tol,
        gtol=tol,
        x_scale="jac",
    )

    msx, msy = result.x
    R_fit = rotation_from_ms(msx, msy)

    return msx, msy, R_fit, result.nfev, result.success


def calculate_ahat(R, nps):
    """
    Horizontal direction (unit) of R * nps, as ahatx and ahaty.
    """

    v = (nps @ R.T)[:, :2]

    norms = np.maximum(np.linalg.norm(v, axis=1), 1e-12)
    v = v / norms[:, None]

    return v[:, 0], v[:, 1]


# ============================================================
# Data parsing
# ============================================================

def parse_data(text):
    """
    Parse five whitespace-separated columns:

    t, ax, ay, -sin(thetax), -sin(thetay)

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
        self.root.title("Wahba Fit: msx, msy (theta0 = 0)")
        self.root.geometry("1700x820")

        self.data = None

        self.setup_interface()

    def setup_interface(self):

        # ----------------------------------------------------
        # Main layout
        # ----------------------------------------------------

        main = ttk.Frame(self.root, padding=5)
        main.pack(fill=tk.BOTH, expand=True)

        # Use the window's width: controls stay in a left sidebar,
        # while the plots occupy the full height on the right.
        controls = ttk.Frame(main, width=440)
        controls.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        controls.pack_propagate(False)

        # ----------------------------------------------------
        # Data input
        # ----------------------------------------------------

        data_frame = ttk.LabelFrame(
            controls,
            text="Input Data (5 columns)",
            padding=5
        )

        data_frame.pack(fill=tk.X, expand=False, pady=(0, 6))

        ttk.Label(
            data_frame,
            text="t    ax    ay    -sin(thetax)    -sin(thetay)"
        ).pack(anchor=tk.W)

        data_text_frame = ttk.Frame(data_frame)
        data_text_frame.pack(fill=tk.BOTH, expand=True)

        self.data_text = tk.Text(
            data_text_frame,
            height=7,
            width=48,
            font=("Consolas", 9),
            wrap=tk.NONE
        )
        data_scrollbar = ttk.Scrollbar(
            data_text_frame, orient=tk.VERTICAL,
            command=self.data_text.yview
        )
        self.data_text.configure(yscrollcommand=data_scrollbar.set)
        self.data_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        data_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # ----------------------------------------------------
        # Parameters
        # ----------------------------------------------------

        settings = ttk.LabelFrame(
            controls,
            text="Regression Parameters",
            padding=10
        )

        settings.pack(fill=tk.X, pady=(0, 6))

        self.tstart_var = tk.StringVar(value="0")
        self.samples_var = tk.StringVar(value="100")
        self.spacing_var = tk.StringVar(value="1")
        self.azconst_var = tk.StringVar(value=str(DEFAULT_AZ_CONST))
        self.iterations_var = tk.StringVar(value="500")

        fields = [
            ("t_start:", self.tstart_var),
            ("Number of samples:", self.samples_var),
            ("Sample spacing:", self.spacing_var),
            ("az constant:", self.azconst_var),
            ("Max iterations:", self.iterations_var)
        ]

        # Stack each setting on its own row so the controls fit
        # within the sidebar instead of extending underneath the plots.
        for row, (label, variable) in enumerate(fields):
            ttk.Label(settings, text=label).grid(
                row=row, column=0, sticky=tk.W, padx=4, pady=2
            )
            ttk.Entry(settings, textvariable=variable, width=14).grid(
                row=row, column=1, sticky=tk.EW, padx=4, pady=2
            )
        settings.columnconfigure(1, weight=1)

        # ----------------------------------------------------
        # Manual model parameters (independent of the fit)
        # ----------------------------------------------------

        manual = ttk.LabelFrame(
            controls,
            text="Fourth Data Set: Manual Parameters (theta0 = 0)",
            padding=10
        )
        manual.pack(fill=tk.X, pady=(0, 6))

        self.manual_msx_var = tk.StringVar(value="0")
        self.manual_msy_var = tk.StringVar(value="0")
        manual_fields = [
            ("Manual msx:", self.manual_msx_var),
            ("Manual msy:", self.manual_msy_var),
        ]

        for row, (label, variable) in enumerate(manual_fields):
            ttk.Label(manual, text=label).grid(
                row=row, column=0, sticky=tk.W, padx=4, pady=2
            )
            ttk.Entry(manual, textvariable=variable, width=14).grid(
                row=row, column=1, sticky=tk.EW, padx=4, pady=2
            )
        manual.columnconfigure(1, weight=1)

        ttk.Label(
            manual,
            text="Orange diamonds: manual ahatx / ahaty",
            wraplength=390,
            justify=tk.LEFT
        ).grid(row=3, column=0, columnspan=2, sticky=tk.W, padx=4, pady=(4, 0))

        # ----------------------------------------------------
        # Ready button
        # ----------------------------------------------------

        ttk.Button(
            settings,
            text="READY / FIT",
            command=self.run_fit
        ).grid(row=len(fields), column=0, columnspan=2, sticky=tk.EW, pady=(6, 2))

        # ----------------------------------------------------
        # Results
        # ----------------------------------------------------

        results = ttk.LabelFrame(
            controls,
            text="Fitting Results",
            padding=10
        )

        results.pack(fill=tk.X, pady=(0, 4))

        self.results_label = ttk.Label(
            results,
            text="No fit performed yet.",
            font=("Consolas", 9),
            justify=tk.LEFT,
            wraplength=390
        )

        self.results_label.pack(anchor=tk.W)

        # ----------------------------------------------------
        # Plots
        # ----------------------------------------------------

        plot_frame = ttk.Frame(main)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.fig = Figure(
            figsize=(12, 7.5),
            dpi=100
        )

        self.ax1 = self.fig.add_subplot(211)
        self.ax2 = self.fig.add_subplot(212)

        self.ax1.set_title("ahatx")
        self.ax1.set_ylabel("ahatx")
        self.ax1.set_xlabel("t")
        self.ax1.grid(True)

        self.ax2.set_title("ahaty")
        self.ax2.set_ylabel("ahaty")
        self.ax2.set_xlabel("t")
        self.ax2.grid(True)

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
            ax = data[:, 1]
            ay = data[:, 2]
            nx = data[:, 3]
            ny = data[:, 4]

            # Measured horizontal direction (for plotting / R squared)
            a_norm = np.maximum(np.hypot(ax, ay), 1e-12)
            ahatx = ax / a_norm
            ahaty = ay / a_norm

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

            az_const = float(self.azconst_var.get())

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

            if az_const <= 0:
                raise ValueError(
                    "The az constant must be positive."
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

            # ------------------------------------------------
            # Build the full 3D vectors np and nps (all samples)
            # ------------------------------------------------

            np_all, nps_all = build_vector_pairs(
                ax, ay, nx, ny, az_const
            )

            # ------------------------------------------------
            # Wahba fit of msx, msy (theta0 fixed at 0)
            # ------------------------------------------------

            msx, msy, R_fit, n_iter, converged = fit_ms_wahba(
                np_all[indices], nps_all[indices],
                max_iter=max_iterations
            )

            calculated_x, calculated_y = calculate_ahat(
                R_fit, nps_all
            )

            # Second prediction from the user's manual parameters
            manual_msx = float(self.manual_msx_var.get())
            manual_msy = float(self.manual_msy_var.get())
            manual_x, manual_y = calculate_ahat(
                rotation_from_ms(manual_msx, manual_msy), nps_all
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
            self.ax1.plot(
                t, ahatx,
                color="blue",
                linewidth=0.5,
            )

            self.ax1.scatter(
                t,
                calculated_x,
                color="red",
                s=2,
                label="ahatx calculated (fitted data set)"
            )

            self.ax1.scatter(
                t,
                manual_x,
                color="darkorange",
                marker="D",
                s=12,
                label="ahatx manual"
            )

            self.ax1.scatter(
                t,
                nx_normalized,
                color="green",
                s=2,
                label="Normalized -sin(thetax)"
            )
            self.ax1.plot(
                t, nx_normalized,
                color="green",
                linewidth=0.5,
            )

            # Fitting samples
            self.ax1.scatter(
                t[indices],
                ahatx[indices],
                color="blue",
                marker="x",
                s=20
            )

            self.ax1.scatter(
                t[indices],
                nx_normalized[indices],
                color="green",
                marker="x",
                s=20
            )

            self.ax1.set_title("X Components")
            self.ax1.set_ylabel("Value")
            self.ax1.set_xlabel("t")
            self.ax1.grid(True)
            self.ax1.legend(loc="best", fontsize=7, ncol=2, markerscale=0.8, labelspacing=0.3, handlelength=1.2, borderpad=0.4)


            # ---------------- PLOT 2: Y ----------------

            self.ax2.scatter(
                t,
                ahaty,
                color="blue",
                s=2,
                label="ahaty measured"
            )
            self.ax2.plot(
                t, ahaty,
                color="blue",
                linewidth=0.5,
            )

            self.ax2.scatter(
                t,
                calculated_y,
                color="red",
                s=2,
                label="ahaty calculated (fitted data set)"
            )

            self.ax2.scatter(
                t,
                manual_y,
                color="darkorange",
                marker="D",
                s=12,
                label="ahaty manual"
            )

            self.ax2.scatter(
                t,
                ny_normalized,
                color="green",
                s=2,
                label="Normalized -sin(thetay)"
            )
            self.ax2.plot(
                t, ny_normalized,
                color="green",
                linewidth=0.5,
            )

            # Fitting samples
            self.ax2.scatter(
                t[indices],
                ahaty[indices],
                color="blue",
                marker="x",
                s=20
                )

            self.ax2.scatter(
                t[indices],
                ny_normalized[indices],
                color="green",
                marker="x",
                s=20
            )

            self.ax2.set_title("Y Components")
            self.ax2.set_ylabel("Value")
            self.ax2.set_xlabel("t")
            self.ax2.grid(True)
            self.ax2.legend(loc="best", fontsize=7, ncol=2, markerscale=0.8, labelspacing=0.3, handlelength=1.2, borderpad=0.4)


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
                f"Fit (theta0 = 0): msx = {msx:.6f}    "
                f"msy = {msy:.6f}\n"
                f"Combined R² = {r2_combined:.6f}\n"
                f"Iterations = {n_iter}"
                f"{'' if converged else ' (NOT converged)'}\n"
                f"Manual (theta0 = 0): msx = {manual_msx:.6f}    "
                f"msy = {manual_msy:.6f}"
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
