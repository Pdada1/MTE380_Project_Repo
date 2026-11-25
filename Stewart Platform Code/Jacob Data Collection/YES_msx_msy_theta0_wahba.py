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

    The vertical component of the acceleration points downward, so the
    acceleration vector is a = (ax, ay, -az) with az > 0 from the
    quadratic az^2 - c*az + (ax^2 + ay^2) = 0. Then
    a x (-ay, ax, 0) = (az*ax, az*ay, ax^2 + ay^2), which points up and
    tilts in the same direction as (ax, ay), matching nps when
    msx = msy = theta0 = 0.

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
    Solve Wahba's problem with the SVD method:

        minimize  sum_i w_i |np_i - R nps_i|^2

    Returns the proper rotation matrix R (det = +1).
    """

    if weights is None:
        weights = np.ones(len(np_vectors))

    # Attitude profile matrix B = sum_i w_i np_i nps_i^T
    B = (np_vectors * weights[:, None]).T @ nps_vectors

    U, _, Vt = np.linalg.svd(B)

    d = np.sign(np.linalg.det(U) * np.linalg.det(Vt))

    if d == 0:
        d = 1.0

    return U @ np.diag([1.0, 1.0, d]) @ Vt


def parameters_to_rotation(msx, msy, theta0):
    """
    Build R from msx, msy, theta0 (degrees).
    The columns of R are x0, y0, ns.
    """

    theta0_rad = np.deg2rad(theta0)

    s = np.array([1.0, 0.0, msx])
    ns = np.array([-msx, -msy, 1.0])

    s /= np.linalg.norm(s)
    ns /= np.linalg.norm(ns)

    cross_ns_s = np.cross(ns, s)

    x0 = s * np.cos(theta0_rad) + cross_ns_s * np.sin(theta0_rad)
    y0 = cross_ns_s * np.cos(theta0_rad) - s * np.sin(theta0_rad)

    return np.column_stack((x0, y0, ns))


def rotation_to_parameters(R):
    """
    Back-calculate msx, msy, theta0 (degrees) from R.

        ns = (-msx, -msy, 1) / |...|     -> third column of R
        s  = (1, 0, msx) / |...|
        x0 = s cos(theta0) + (ns x s) sin(theta0)   -> first column of R
    """

    ns = R[:, 2]

    if ns[2] <= 1e-12:
        raise ValueError(
            "The fitted rotation has ns_z <= 0, so it cannot be "
            "written with ns = (-msx, -msy, 1)/norm."
        )

    msx = -ns[0] / ns[2]
    msy = -ns[1] / ns[2]

    s = np.array([1.0, 0.0, msx])
    s /= np.linalg.norm(s)

    cross_ns_s = np.cross(ns, s)

    x0 = R[:, 0]

    theta0 = np.degrees(
        np.arctan2(np.dot(x0, cross_ns_s), np.dot(x0, s))
    )

    return msx, msy, theta0


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
        self.root.title("Wahba Rotation Fitting (np = R * nps)")
        self.root.geometry("1700x820")

        self.data = None

        self.setup_interface()

    def setup_interface(self):

        # ----------------------------------------------------
        # Main layout
        # ----------------------------------------------------

        main = ttk.Frame(self.root, padding=5)
        main.pack(fill=tk.BOTH, expand=True)

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
            text="Wahba Fit Settings",
            padding=10
        )

        settings.pack(fill=tk.X, pady=(0, 6))

        self.tstart_var = tk.StringVar(value="0")
        self.samples_var = tk.StringVar(value="100")
        self.spacing_var = tk.StringVar(value="1")
        self.azconst_var = tk.StringVar(value=str(DEFAULT_AZ_CONST))

        fields = [
            ("t_start:", self.tstart_var),
            ("Number of samples:", self.samples_var),
            ("Sample spacing:", self.spacing_var),
            ("az constant (5.89):", self.azconst_var),
        ]

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
            text="Manual Parameters",
            padding=10
        )
        manual.pack(fill=tk.X, pady=(0, 6))

        self.manual_msx_var = tk.StringVar(value="0")
        self.manual_msy_var = tk.StringVar(value="0")
        self.manual_theta0_var = tk.StringVar(value="0")

        manual_fields = [
            ("Manual msx:", self.manual_msx_var),
            ("Manual msy:", self.manual_msy_var),
            ("Manual theta0 (deg):", self.manual_theta0_var),
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
            text="Orange diamonds: R * nps using the manual "
                 "msx / msy / theta0",
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
            text="Wahba Results",
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

        self.axes = [
            self.fig.add_subplot(311),
            self.fig.add_subplot(312),
            self.fig.add_subplot(313),
        ]

        for axis, comp in zip(self.axes, ("x", "y", "z")):
            axis.set_title(f"np_{comp} vs (R*nps)_{comp}")
            axis.set_ylabel(f"{comp} component")
            axis.set_xlabel("t")
            axis.grid(True)

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

            # ------------------------------------------------
            # Read settings
            # ------------------------------------------------

            tstart = float(self.tstart_var.get())
            num_samples = int(self.samples_var.get())
            spacing = int(self.spacing_var.get())
            az_const = float(self.azconst_var.get())

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

            start_index = np.searchsorted(
                t, tstart, side="left"
            )

            if start_index >= len(t):
                raise ValueError(
                    "t_start is beyond the available data."
                )

            indices = (
                start_index
                + np.arange(num_samples) * spacing
            )

            if indices[-1] >= len(t):
                raise ValueError(
                    "Not enough data for the requested "
                    "number of samples and spacing."
                )

            # ------------------------------------------------
            # Build np and nps for ALL samples
            # ------------------------------------------------

            np_all, nps_all = build_vector_pairs(
                ax, ay, nx, ny, az_const
            )

            np_fit = np_all[indices]
            nps_fit = nps_all[indices]

            # ------------------------------------------------
            # Wahba's problem (SVD solution)
            # ------------------------------------------------

            R_fit = solve_wahba(np_fit, nps_fit)

            msx, msy, theta0 = rotation_to_parameters(R_fit)

            # R * nps for all samples, from the fitted rotation
            fitted_all = nps_all @ R_fit.T

            # ------------------------------------------------
            # Manual parameters
            # ------------------------------------------------

            manual_msx = float(self.manual_msx_var.get())
            manual_msy = float(self.manual_msy_var.get())
            manual_theta0 = float(self.manual_theta0_var.get())

            R_manual = parameters_to_rotation(
                manual_msx, manual_msy, manual_theta0
            )
            manual_all = nps_all @ R_manual.T

            # ------------------------------------------------
            # Fit quality (fitting samples only)
            # ------------------------------------------------

            cos_err = np.clip(
                np.sum(np_fit * fitted_all[indices], axis=1),
                -1.0, 1.0
            )
            rms_err_deg = np.degrees(
                np.sqrt(np.mean(np.arccos(cos_err) ** 2))
            )

            r2_combined = calculate_r_squared(
                np_fit.ravel(), fitted_all[indices].ravel()
            )

            # ------------------------------------------------
            # Update plots
            # ------------------------------------------------

            for axis in self.axes:
                axis.clear()

            for k, comp in enumerate(("x", "y", "z")):

                axis = self.axes[k]

                axis.scatter(
                    t, np_all[:, k],
                    color="blue", s=2,
                    label=f"np_{comp} (from a)"
                )
                axis.plot(
                    t, np_all[:, k],
                    color="blue", linewidth=0.5
                )

                axis.scatter(
                    t, fitted_all[:, k],
                    color="red", s=2,
                    label=f"(R*nps)_{comp} fitted"
                )

                axis.scatter(
                    t, manual_all[:, k],
                    color="darkorange", marker="D", s=12,
                    label=f"(R*nps)_{comp} manual"
                )

                # nps scaled (for plotting only) so its largest magnitude
                # matches that of np
                nps_peak = np.max(np.abs(nps_all[:, k]))
                np_peak = np.max(np.abs(np_all[:, k]))
                scale = np_peak / nps_peak if nps_peak > 1e-12 else 1.0

                axis.scatter(
                    t, scale * nps_all[:, k],
                    color="green", s=2,
                    label=f"nps_{comp} (scaled x{scale:.3g})"
                )
                axis.plot(
                    t, scale * nps_all[:, k],
                    color="green", linewidth=0.5
                )

                # Samples used in the fit
                axis.scatter(
                    t[indices], np_all[indices, k],
                    color="blue", marker="x", s=20,
                    label="fit samples"
                )

                axis.set_title(f"np_{comp} vs (R*nps)_{comp}")
                axis.set_ylabel(f"{comp} component")
                axis.set_xlabel("t")
                axis.grid(True)
                axis.legend(
                    loc="best", fontsize=7, ncol=2,
                    markerscale=0.8, labelspacing=0.3,
                    handlelength=1.2, borderpad=0.4
                )

            self.fig.tight_layout()

            # ------------------------------------------------
            # Interactive hover tooltips
            # ------------------------------------------------

            if hasattr(self, "cursor"):
                self.cursor.remove()

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
                f"Wahba fit:\n"
                f"  msx    = {msx:.6f}\n"
                f"  msy    = {msy:.6f}\n"
                f"  theta0 = {theta0:.4f} deg\n"
                f"  RMS angle error (np vs R*nps) = "
                f"{rms_err_deg:.4f} deg\n"
                f"  R² (np components) = {r2_combined:.6f}\n"
                f"Manual: msx = {manual_msx:.6f}    "
                f"msy = {manual_msy:.6f}    "
                f"theta0 = {manual_theta0:.4f} deg"
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
