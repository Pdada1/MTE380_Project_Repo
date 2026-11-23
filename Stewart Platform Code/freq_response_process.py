import numpy as np
from numpy.polynomial import polynomial as P
from pathlib import Path

def fit_sinusoid(t, y, f):
    """
    Least-squares sinusoid fit with known frequency.

    Model: y_fit = amp * cos(2*pi*f*t + phase) + offset

    Parameters
    ----------
    t : array_like, shape (N,)   time vector
    y : array_like, shape (N,)   data vector
    f : float                    known frequency (same units as 1/t)

    Returns
    -------
    amp, phase, offset : floats  (phase in radians)
    """
    t = np.asarray(t, dtype=float).ravel()
    y = np.asarray(y, dtype=float).ravel()

    w = 2 * np.pi * f * t
    A = np.column_stack((np.cos(w), np.sin(w), np.ones_like(t)))

    x, *_ = np.linalg.lstsq(A, y, rcond=None)

    amp = np.hypot(x[0], x[1])
    phase = np.arctan2(-x[1], x[0])
    offset = x[2]
    return amp, phase, offset

def fit_tf(w, mag, phase, n, m=None, mag_db=False, phase_deg=True,
           weights=None, iters=30, tol=1e-9):
    """
    Fit H(s) = B(s)/A(s) to frequency-response data (SK iteration).

    Parameters
    ----------
    w : (N,) angular frequencies in rad/s (use 2*pi*f if you have Hz)
    mag, phase : (N,) measured magnitude and phase
    n : denominator order
    m : numerator order (default n)
    mag_db : True if mag is in dB
    phase_deg : True if phase is in degrees
    weights : optional (N,) weights on the data points
    iters, tol : SK iteration limit and convergence tolerance

    Returns
    -------
    b, a : numerator and denominator coefficients, highest power first
           (same convention as scipy.signal.TransferFunction / freqs)
    """
    w = np.asarray(w, float).ravel()
    mag = np.asarray(mag, float).ravel()
    phase = np.asarray(phase, float).ravel()
    m = n if m is None else m

    if mag_db:
        mag = 10 ** (mag / 20)
    if phase_deg:
        phase = np.deg2rad(phase)
    H = mag * np.exp(1j * phase)

    wt = np.ones_like(w) if weights is None else np.asarray(weights, float)

    # Frequency scaling for numerical conditioning
    w0 = np.sqrt(w[w > 0].min() * w.max())
    s = 1j * w / w0

    # Model (A monic): sum_k b_k s^k - H * sum_{k<n} a_k s^k = H * s^n
    Pb = s[:, None] ** np.arange(m + 1)
    Pa = -H[:, None] * s[:, None] ** np.arange(n)
    Phi = np.hstack((Pb, Pa))
    rhs = H * s ** n

    W = wt.copy()
    a_old = None
    for _ in range(iters):
        M = Phi * W[:, None]
        r = rhs * W
        # Stack real and imaginary parts so the coefficients stay real
        Mr = np.vstack((M.real, M.imag))
        rr = np.concatenate((r.real, r.imag))
        x, *_ = np.linalg.lstsq(Mr, rr, rcond=None)

        b_lo = x[:m + 1]            # b_0 ... b_m  (ascending)
        a_lo = x[m + 1:]            # a_0 ... a_{n-1}
        a_full = np.concatenate(([1.0], a_lo[::-1]))   # descending, monic

        # SK reweighting by 1/|A(jw)| from this iteration
        W = wt / np.abs(np.polyval(a_full, s))

        if a_old is not None and np.linalg.norm(a_full - a_old) < tol * np.linalg.norm(a_full):
            break
        a_old = a_full

    # Undo frequency scaling: coeff of s^k scales by w0^(n-k)
    a = a_full * w0 ** (n - np.arange(n, -1, -1))
    b = b_lo[::-1] * w0 ** (n - np.arange(m, -1, -1))
    return b, a

def ultimate_gain(b, a):
    """
    Ziegler-Nichols ultimate gain and period from G(s) = B(s)/A(s).
    b, a: coefficients, highest power first (as returned by fit_tf).

    Returns Ku, Tu, w180  (rad/s). The first (lowest-frequency)
    -180 deg crossing is used. Returns None if there is no crossing.
    """
    b = np.atleast_1d(np.asarray(b, float))[::-1]   # ascending powers
    a = np.atleast_1d(np.asarray(a, float))[::-1]

    # B(jw) and A(jw) as polynomials in w (complex coefficients)
    Bc = b * (1j) ** np.arange(len(b))
    Ac = a * (1j) ** np.arange(len(a))

    # G(jw) is real when Im{ B(jw) * conj(A(jw)) } = 0
    N = P.polymul(Bc, np.conj(Ac))
    imag = P.polytrim(N.imag, tol=1e-12 * np.max(np.abs(N)))
    if len(imag) < 2:
        return None

    roots = P.polyroots(imag)
    cand = roots[(np.abs(roots.imag) < 1e-6 * np.maximum(1, np.abs(roots)))
                 & (roots.real > 1e-9)].real

    crossings = []
    for wc in np.sort(cand):
        G = np.polyval(b[::-1], 1j * wc) / np.polyval(a[::-1], 1j * wc)
        if G.real < 0:                      # real AND negative -> -180 deg
            crossings.append((wc, abs(G)))

    if not crossings:
        return None
    w180, Gmag = crossings[0]
    return 1.0 / Gmag, 2 * np.pi / w180, w180

def zn_pid(Ku, Tu, rule="classic"):
    """
    Ziegler-Nichols PID tuning from ultimate gain Ku and ultimate period Tu.

    Returns Kp, Ki, Kd for the parallel form:
        C(s) = Kp + Ki/s + Kd*s
    """
    # (Kp/Ku, Ti/Tu, Td/Tu)
    rules = {
        "classic":      (0.60, 1/2,  1/8),    # standard ZN
        "pessen":       (0.70, 2/5,  3/20),   # Pessen integral rule
        "some_overshoot": (0.33, 1/2, 1/3),   # reduced overshoot
        "no_overshoot": (0.20, 1/2,  1/3),    # minimal overshoot
    }
    kp_f, ti_f, td_f = rules[rule]

    Kp = kp_f * Ku
    Ti = ti_f * Tu
    Td = td_f * Tu

    Ki = Kp / Ti        # = 1.2*Ku/Tu for classic
    Kd = Kp * Td        # = 0.075*Ku*Tu for classic
    return Kp, Ki, Kd

def load_run(path):
    """Return (freq_hz, amp_deg, t, out) from one CSV, or None if it has no data.

    out is the ball's position change along the swept axis. Older 3-column logs
    (t, x, y) are still read by picking the swept axis column.
    """
    with open(path) as f:
        meta = dict(item.split("=") for item in f.readline().lstrip("# ").split())
    data = np.loadtxt(path, delimiter=",", skiprows=2, ndmin=2)  # skip comment + header
    if len(data) < 10:
        return None
    if data.shape[1] >= 3:   # old format: pick the swept axis and remove its starting value
        out = data[:, 1] if meta["axis"] == "x" else data[:, 2]
        out = out - out[0]
    else:
        out = data[:, 1]
    return float(meta["freq_hz"]), float(meta["amp_deg"]), data[:, 0], out

def frequency_response(axis, folder="freq_logs", skip_cycles=1,
                       fmin=0.6, fmax=1.1, invert=True):
    """Magnitude (radii/deg) and phase (deg) of ball position vs platform tilt.

    invert=True adds 180 deg: a +tilt rolls the ball toward -axis, and the
    controller's negative feedback relies on that, so the loop sees -G.
    """
    rows = []
    for path in Path(folder).glob(f"{axis}_*Hz.csv"):
        run = load_run(path)
        if run is None:
            continue
        f, amp_deg, t, out = run
        if not fmin <= f <= fmax:
            continue
        keep = t >= skip_cycles / f
        amp, phase, _ = fit_sinusoid(t[keep], out[keep], f)
        # The firmware's input is -amp*cos(wt), i.e. a cosine with phase pi.
        phase_deg = np.rad2deg(phase - np.pi) + (180 if invert else 0)
        phase_deg = (phase_deg + 180) % 360 - 180
        rows.append((f, amp / amp_deg, phase_deg))
    if len(rows) < 4:
        return None
    f, mag, phase = np.array(sorted(rows)).T
    return f, mag, phase


def autotune_from_logs(folder="freq_logs", n=3, m=0, rule="classic", **kwargs):
    """Fit each axis from its CSVs and return {axis: (Kp, Ki, Kd) or None}."""
    gains = {}
    for axis in ("x", "y"):
        response = frequency_response(axis, folder, **kwargs)
        if response is None:
            gains[axis] = None
            continue
        f, mag, phase = response
        b, a = fit_tf(2 * np.pi * f, mag, phase, n=n, m=m)
        result = ultimate_gain(b, a)
        gains[axis] = zn_pid(result[0], result[1], rule) if result else None
    return gains