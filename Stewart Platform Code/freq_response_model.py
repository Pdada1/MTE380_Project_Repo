"""Model-based PID design from the frequency-response logs.

Why not Ziegler-Nichols here: the ball on the plate is a (nearly) double
integrator, so its phase sits close to -180 deg over the whole measured band.
There is no sharp ultimate point to read off, and a free-form fit extrapolates
badly to wherever it happens to cross.  Instead:

1. Fit a small physical model to the measured response (complex least squares):
       G(s) = K * exp(-s*tau) / ( s * (s + a) * (T*s + 1) )
   K   : rolling-ball gain (radii/deg/s^2; physics says ~ (5/7)*g*(pi/180)/R)
   a   : velocity damping (rolling friction), 1/s
   tau : total dead time (camera latency + servo + ball), s
   T   : optional servo lag, s
2. Loop-shape the PID on that model: pick a crossover wc and phase margin PM,
   then solve C(j*wc) = e^{j(-180+PM)} / G(j*wc) for Kp and Kd, with Ki set by
   the integral time Ti = ti_factor / wc.
3. If wc is not given, scan wc and take the fastest loop whose peak sensitivity
   Ms stays under ms_max.

Needs only numpy.  Gains are in the same units the firmware uses
(degrees of tilt per platform radius of error, parallel form).
"""
import numpy as np

from freq_response_process import frequency_response


# --------------------------------------------------------------------------
# Plant model
# --------------------------------------------------------------------------
def plant_response(w, K, a, tau, T=0.0):
    """G(jw) for K*exp(-s*tau) / (s*(s+a)*(T*s+1))."""
    s = 1j * np.asarray(w, float)
    return K * np.exp(-s * tau) / (s * (s + a) * (T * s + 1))


_LOG_BOUNDS = {            # bounds on log(parameter)
    "K": (-15.0, 10.0),
    "a": (-6.0, 4.0),
    "tau": (-7.0, 0.0),    # ~1 ms .. 1 s
    "T": (-7.0, 0.0),
}


def _levenberg_marquardt(fun, p0, lo, hi, iters=200, tol=1e-12):
    p = np.clip(np.asarray(p0, float), lo, hi)
    r = fun(p)
    cost = r @ r
    lam = 1e-2
    for _ in range(iters):
        J = np.empty((len(r), len(p)))
        for j in range(len(p)):
            dp = p.copy()
            dp[j] += 1e-6
            J[:, j] = (fun(dp) - r) / 1e-6
        g = J.T @ r
        H = J.T @ J
        improved = False
        for _ in range(25):
            step = np.linalg.solve(H + lam * np.diag(np.diag(H) + 1e-12), -g)
            pn = np.clip(p + step, lo, hi)
            rn = fun(pn)
            cn = rn @ rn
            if cn < cost:
                done = (cost - cn) < tol * max(cost, 1e-30)
                p, r, cost = pn, rn, cn
                lam = max(lam / 3, 1e-9)
                improved = True
                break
            lam *= 4
        if not improved or done:
            break
    return p, cost


def fit_plant_model(f, mag, phase_deg, with_lag=False):
    """Fit the plant model to measured (f [Hz], mag [radii/deg], phase [deg]).

    Residuals are the complex log-ratio of measured to model, so magnitude
    (in log terms) and phase are weighted equally and phase wrapping is harmless.
    Returns a dict with K, a, tau, T and fit-quality numbers.
    """
    f = np.asarray(f, float)
    w = 2 * np.pi * f
    H = np.asarray(mag, float) * np.exp(1j * np.deg2rad(np.asarray(phase_deg, float)))

    names = ["K", "a", "tau"] + (["T"] if with_lag else [])
    lo = np.array([_LOG_BOUNDS[n][0] for n in names])
    hi = np.array([_LOG_BOUNDS[n][1] for n in names])

    def residual(p):
        ratio = H / plant_response(w, *np.exp(p))
        return np.concatenate((np.log(np.abs(ratio)), np.angle(ratio)))

    K0 = float(np.mean(np.abs(H) * w ** 2))      # double-integrator estimate
    best = None
    for a0 in (0.05, 0.5, 3.0):
        for tau0 in (0.02, 0.08, 0.2):
            start = [np.log(K0), np.log(a0), np.log(tau0)] + ([np.log(0.05)] if with_lag else [])
            p, cost = _levenberg_marquardt(residual, start, lo, hi)
            if best is None or cost < best[1]:
                best = (p, cost)

    params = dict(zip(names, np.exp(best[0])))
    params.setdefault("T", 0.0)
    r = residual(best[0])
    n = len(f)
    params["rms_mag_db"] = float(np.sqrt(np.mean(r[:n] ** 2)) * 20 / np.log(10))
    params["rms_phase_deg"] = float(np.rad2deg(np.sqrt(np.mean(r[n:] ** 2))))
    return params


# --------------------------------------------------------------------------
# PID design on the model
# --------------------------------------------------------------------------
def _loop(w, model, Kp, Ki, Kd, controller_delay):
    s = 1j * w
    C = Kp + Ki / s + Kd * s
    G = plant_response(w, model["K"], model["a"], model["tau"], model["T"])
    return C * G * np.exp(-s * controller_delay)


def peak_sensitivity(model, gains, controller_delay=0.0):
    """Max |1/(1+L)| over frequency (smaller = more robust; ~1.2-1.6 is good)."""
    w = np.logspace(-2, 3, 4000)
    L = _loop(w, model, *gains, controller_delay)
    return float(np.max(np.abs(1.0 / (1.0 + L))))


def pid_at_crossover(model, wc, pm_deg=50.0, ti_factor=8.0, controller_delay=0.0):
    """Parallel-form (Kp, Ki, Kd) giving |L(jwc)| = 1 and phase -180 + pm_deg.

    Returns None if no valid (Kp > 0, Kd >= 0) PID exists at this wc.
    """
    G = plant_response(wc, model["K"], model["a"], model["tau"], model["T"])
    G = G * np.exp(-1j * wc * controller_delay)
    C = np.exp(1j * (-np.pi + np.deg2rad(pm_deg) - np.angle(G))) / abs(G)
    Kp = float(C.real)
    Ki = Kp * wc / ti_factor                   # Ti = ti_factor / wc
    Kd = float((C.imag + Ki / wc) / wc)
    if Kp <= 0 or Kd < 0:
        return None
    return Kp, Ki, Kd


def design_pid(model, wc=None, pm_deg=50.0, ti_factor=8.0, controller_delay=1 / 60,
               ms_max=1.6, w_range=None):
    """Design a PID on the fitted model.

    wc=None scans crossover frequencies and takes the fastest one with
    Ms <= ms_max (falling back to the lowest-Ms design if none qualify).
    controller_delay: extra delay the plant data does not contain
    (zero-order hold + backward-difference derivative, about half a frame).
    Returns dict with gains, wc, Ms, or None.
    """
    if wc is not None:
        candidates = [wc]
    else:
        lo, hi = w_range if w_range else (1.0, 15.0)
        candidates = np.logspace(np.log10(lo), np.log10(hi), 80)

    designs = []
    for c in candidates:
        gains = pid_at_crossover(model, c, pm_deg, ti_factor, controller_delay)
        if gains is None:
            continue
        designs.append((c, gains, peak_sensitivity(model, gains, controller_delay)))
    if not designs:
        return None

    good = [d for d in designs if d[2] <= ms_max]
    c, gains, ms = max(good, key=lambda d: d[0]) if good else min(designs, key=lambda d: d[2])
    return {"Kp": gains[0], "Ki": gains[1], "Kd": gains[2], "wc_rad_s": float(c), "Ms": ms}


# --------------------------------------------------------------------------
# Entry point: same role as autotune_from_logs, but model based
# --------------------------------------------------------------------------
def model_based_autotune_from_logs(folder="freq_logs", with_lag=False, wc=None, pm_deg=50.0,
                                   ti_factor=8.0, controller_delay=1 / 60, ms_max=1.6,
                                   **kwargs):
    """Fit the plant model per axis and design a PID.

    Returns {axis: {"gains": (Kp, Ki, Kd), "design": {...}, "model": {...}} or None}.
    kwargs go to frequency_response (fmin, fmax, skip_cycles, invert).
    """
    out = {}
    for axis in ("x", "y"):
        response = frequency_response(axis, folder, **kwargs)
        if response is None:
            out[axis] = None
            continue
        f, mag, phase = response
        model = fit_plant_model(f, mag, phase, with_lag=with_lag)
        w = 2 * np.pi * f
        design = design_pid(model, wc, pm_deg, ti_factor, controller_delay, ms_max,
                            w_range=(0.5 * w.min(), 1.5 * w.max()))
        out[axis] = None if design is None else {
            "gains": (design["Kp"], design["Ki"], design["Kd"]),
            "design": design,
            "model": model,
        }
    return out


if __name__ == "__main__":
    result = model_based_autotune_from_logs()
    for axis, r in result.items():
        if r is None:
            print(f"{axis}: not enough data or no valid design")
            continue
        m, d = r["model"], r["design"]
        print(f"{axis}: K={m['K']:.4g} a={m['a']:.3g} tau={m['tau'] * 1000:.0f} ms "
              f"(fit rms {m['rms_mag_db']:.2f} dB, {m['rms_phase_deg']:.1f} deg)")
        print(f"   Kp={d['Kp']:.3f} Ki={d['Ki']:.3f} Kd={d['Kd']:.3f}  "
              f"wc={d['wc_rad_s']:.2f} rad/s  Ms={d['Ms']:.2f}")
