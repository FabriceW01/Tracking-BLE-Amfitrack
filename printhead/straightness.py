"""
Straightness / tracking-precision analysis of a ``--profile-csv`` pass
======================================================================

Offline companion to ``--profile-csv`` (see ``profiling.PassProfiler``), for
one specific experiment: **run the cart along a straight edge (a ruler), then
ask how straight the tracked path came out.**

Every logged ``(u_mm, v_mm)`` should then lie on one straight line. Whatever
distance the points sit *off* that line is the combined error of everything
between the ruler and the CSV -- so this module fits the line and reports the
deviation, both overall and as a function of position along it.

What the deviation actually contains
------------------------------------
This is an **upper bound on tracking error, not a pure measurement of it**.
Four things add into the same number and this module cannot separate them
from the CSV alone:

  1. real tracker error (EM noise, and field distortion near metal),
  2. the cart not being held flush against the ruler the whole way,
  3. the ruler itself not being straight,
  4. cart ROTATION, which moves the logged point even when the sensor
     travels perfectly -- see below, this one is usually the big one.

Point 4 deserves its own warning. The ``u_mm``/``v_mm`` in a page-mode CSV are
**nozzle-bar-referenced**, not sensor-referenced: ``tracking.PageMapper``
adds the fixed sensor->nozzle-bar offset (``geometry.
SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM``, 62.36 mm on this rig) *rotated by the
cart's current yaw*. Rotating the cart by one degree therefore swings the
logged point by ``62.36 mm * 1 deg`` = **1.09 mm**, with the sensor itself
perfectly still. A hand that twists slightly while sliding along the ruler
produces millimetres of apparent deviation that are not tracking error at
all.

That is exactly why ``PassProfiler.record_page_sample`` logs the raw
quaternion (its docstring says so: logged "for offline correlation ...
investigating whether cart rotation, combined with the fixed sensor->nozzle-
bar lever arm ... explains observed freehand misalignment"). This module is
the offline reader that finally uses those columns: it reports how far the
cart rotated, how much apparent deviation that rotation could account for
through the lever arm, and how strongly the two actually correlate.

Method
------
The line fit is **total least squares** (principal component / orthogonal
regression), NOT the ordinary ``v = m*u + c`` least squares. Two reasons,
both of which would otherwise produce a wrong answer on real data:

  * OLS assumes ``u`` is exact and all error is in ``v``. Here the error is
    two-dimensional -- the tracker is equally wrong along both page axes --
    so the quantity being minimised has to be the PERPENDICULAR distance to
    the line, which is what TLS minimises and OLS does not.
  * OLS blows up as the line approaches vertical (infinite slope). A ruler
    run mostly along the ``v`` axis is a completely ordinary thing to do and
    must not need a different code path. TLS has no preferred axis at all.

Residuals are then split into two parts, because they mean different things:

  * a **systematic** part -- a smooth bend, fitted as a quadratic in the
    along-line coordinate. A consistent bow is the signature of field
    distortion (or a bent ruler); it does not average out and it will show
    up in a print.
  * a **random** part -- what is left after removing that bend. This is
    sample-to-sample jitter, which the ``PositionFilter`` (``--smooth-ms``)
    already suppresses and which partly averages out over a dose.

Reporting both separately matters: 0.3 mm of RMS deviation that is all
smooth bow is a very different problem from 0.3 mm that is all jitter.

Everything here is a pure function of arrays -- no tracker, no event loop,
no file I/O except the one CSV reader -- so it is directly unit-testable
(see ``tests/test_straightness.py``), the same reason
``diagnostics._calibration_check_summary`` is factored out of its own
streaming loop.
"""

from __future__ import annotations

import csv
import math
from typing import Dict, List, Optional, Sequence

import numpy as np

from .geometry import NOZZLE_PITCH_MM, SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM
from .rotation import _normalize_quat, _quat_conjugate, _quat_multiply

# Minimum distance the cart must have travelled along the fitted line before
# any verdict means anything. A "perfectly straight" 5 mm dab proves nothing:
# two points are always exactly collinear, and a short scrap of a long sweep
# never leaves the region where the tracker happens to be locally good. Same
# "did the operator actually do the experiment" guard
# diagnostics.CALIBRATION_CHECK_MIN_TRAVEL_MM exists for.
MIN_TRAVEL_MM = 50.0

# Verdict thresholds, expressed in NOZZLE ROWS rather than millimetres on
# purpose: the question this tool answers is "will this deviation show up in
# a print on THIS machine", and the unit that decides that is the nozzle
# pitch (0.0868 mm) -- one row of deviation is one row of misplaced ink.
# These are print-resolution thresholds, NOT a claim about what an Amfitrack
# is spec'd to achieve; a deviation this tool calls "sichtbar" may still be
# entirely normal for the sensor.
GOOD_ROWS = 1.0        # under one nozzle row: cannot show in a print
FAIR_ROWS = 3.0        # a few rows: visible on close inspection

# Cart rotation above which the lever-arm effect is called out as the
# dominant suspect rather than a footnote. 0.5 deg through the 62.36 mm
# lever arm is already ~0.54 mm of apparent deviation -- more than six
# nozzle rows, i.e. bigger than anything this tool would otherwise call
# "good".
ROTATION_NOTABLE_DEG = 0.5


# ---------------------------------------------------------------- CSV input
def read_profile_csv(path: str) -> Dict[str, np.ndarray]:
    """
    Read a page-mode ``--profile-csv`` file into arrays.

    Expects the page-mode header written by ``PassProfiler.start()``:
    ``t_s,row,col,u_mm,v_mm,speed_mm_s,writes_per_s,qx,qy,qz,qw``.

    A LINE-mode CSV (``t_s,column,advance_mm,write_latency_ms,speed_mm_s``)
    is rejected with a clear message rather than half-parsed: it records a
    1D column index and an advance distance, with no second page axis at
    all, so there is genuinely no 2D path in it to check straightness
    against. That is a property of the data, not a missing feature here.

    Quaternion fields are written blank (``,,,``) for any sample where the
    tracker returned no orientation packet; those become ``nan`` and are
    dropped by the rotation analysis rather than being read as a degenerate
    ``(0,0,0,0)`` "rotation".
    """
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        fields = reader.fieldnames or []
        if "u_mm" not in fields or "v_mm" not in fields:
            if "advance_mm" in fields:
                raise ValueError(
                    f"{path!r} is a LINE-mode profile CSV (columns: "
                    f"{','.join(fields)}). It records a 1D column index and "
                    f"advance distance only -- there is no second page axis "
                    f"in it, so straightness cannot be measured. Re-run the "
                    f"pass with --mode page --profile-csv to get u_mm/v_mm.")
            raise ValueError(
                f"{path!r} does not look like a --profile-csv file "
                f"(columns: {','.join(fields) or '<none>'}); expected at "
                f"least u_mm and v_mm.")

        cols: Dict[str, List[float]] = {name: [] for name in fields}
        for row in reader:
            for name in fields:
                raw = (row.get(name) or "").strip()
                try:
                    cols[name].append(float(raw))
                except ValueError:
                    cols[name].append(float("nan"))

    return {name: np.asarray(vals, dtype=float) for name, vals in cols.items()}


# ------------------------------------------------------------- line fitting
def fit_line_tls(u: Sequence[float], v: Sequence[float]) -> Optional[dict]:
    """
    Total-least-squares (orthogonal) line fit through the ``(u, v)`` points.

    Returns ``None`` when there is nothing to fit -- fewer than 2 points, or
    every point at the same place -- rather than a degenerate line that
    later maths would silently divide by.

    Returns a dict with:
      * ``centroid``   -- ``(u, v)`` mean, a point the line passes through.
      * ``direction``  -- unit vector along the line (principal axis).
      * ``normal``     -- unit vector perpendicular to it.
      * ``angle_deg``  -- direction's angle from the +u axis, in
        ``[-90, 90)``; the line is undirected, so the sign of ``direction``
        itself carries no meaning and is normalised here to always point
        into the +u half-plane (or +v when exactly vertical), so two runs
        along the same ruler in opposite directions report the same angle.
    """
    pts = np.column_stack([np.asarray(u, dtype=float),
                           np.asarray(v, dtype=float)])
    pts = pts[np.isfinite(pts).all(axis=1)]
    if pts.shape[0] < 2:
        return None

    centroid = pts.mean(axis=0)
    centered = pts - centroid
    if float(np.max(np.abs(centered))) < 1e-12:
        return None

    # SVD of the centered points: the first right-singular vector is the
    # direction that maximises variance == the TLS line direction, and the
    # second is the normal whose projection TLS minimises. Using SVD rather
    # than an eigendecomposition of the covariance matrix avoids squaring
    # the data's condition number.
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    direction = vt[0]
    normal = np.array([-direction[1], direction[0]])

    # Undirected line: pin the sign so a right-to-left run and a
    # left-to-right run over the same ruler report the same angle.
    if direction[0] < 0 or (direction[0] == 0.0 and direction[1] < 0):
        direction = -direction
        normal = -normal

    return {
        "centroid": centroid,
        "direction": direction,
        "normal": normal,
        "angle_deg": math.degrees(math.atan2(direction[1], direction[0])),
    }


def project_to_line(u: Sequence[float], v: Sequence[float],
                    fit: dict) -> "tuple[np.ndarray, np.ndarray]":
    """
    Express each point in the fitted line's own frame.

    Returns ``(along, perp)``:
      * ``along`` -- distance along the line from the centroid (mm), i.e.
        "where on the ruler", the x-axis of every position-dependent report.
      * ``perp``  -- SIGNED perpendicular distance from the line (mm). The
        sign is kept (not abs) on purpose: a residual that goes +0.2 then
        -0.2 is a bend, one that jitters +-0.2 randomly is noise, and
        collapsing to magnitude would erase exactly that distinction.
    """
    pts = np.column_stack([np.asarray(u, dtype=float),
                           np.asarray(v, dtype=float)])
    centered = pts - fit["centroid"]
    return centered @ fit["direction"], centered @ fit["normal"]


def fit_bow(along: Sequence[float], perp: Sequence[float]) -> Optional[dict]:
    """
    Fit a quadratic ``perp = a*along^2 + b*along + c`` to separate a smooth
    BEND from random jitter.

    A TLS fit already forces the mean and the linear trend of ``perp`` to
    ~zero, so ``a`` carries essentially all the shape: this measures how
    much the path bows away from the straight line it was fitted to.

    Returns ``None`` when fewer than 3 finite points make a quadratic
    meaningless (with 3 unknowns and <3 points the fit is not determined).

    Returns a dict with:
      * ``coeffs``       -- ``(a, b, c)``.
      * ``bow_mm``       -- peak-to-peak excursion of the fitted curve
        across the observed span. The headline "how bent is it" number,
        in the same millimetres as everything else.
      * ``systematic_rms_mm`` -- RMS of the fitted curve (the part of the
        deviation that has a shape).
      * ``random_rms_mm``     -- RMS of what the curve does NOT explain
        (the part that is jitter).
    """
    a_arr = np.asarray(along, dtype=float)
    p_arr = np.asarray(perp, dtype=float)
    mask = np.isfinite(a_arr) & np.isfinite(p_arr)
    a_arr, p_arr = a_arr[mask], p_arr[mask]
    if a_arr.size < 3:
        return None

    coeffs = np.polyfit(a_arr, p_arr, 2)
    model = np.polyval(coeffs, a_arr)
    residual = p_arr - model
    return {
        "coeffs": tuple(float(c) for c in coeffs),
        "bow_mm": float(model.max() - model.min()),
        "systematic_rms_mm": float(np.sqrt(np.mean(model ** 2))),
        "random_rms_mm": float(np.sqrt(np.mean(residual ** 2))),
    }


def binned_profile(along: Sequence[float], perp: Sequence[float],
                   n_bins: int = 10) -> List[dict]:
    """
    Deviation as a function of position along the line -- the direct answer
    to "how big is the error *where*".

    Splits the travelled span into ``n_bins`` equal-width bins and reports,
    per bin, both the MEAN signed deviation (a systematic offset in that
    stretch -- the number that tells you the path is off *over there*) and
    the RMS/max magnitude (how much it scatters within the stretch).

    Empty bins are returned with ``count == 0`` and ``None`` statistics
    rather than being dropped, so the caller can render a continuous axis
    and a gap stays visible as a gap.
    """
    a_arr = np.asarray(along, dtype=float)
    p_arr = np.asarray(perp, dtype=float)
    mask = np.isfinite(a_arr) & np.isfinite(p_arr)
    a_arr, p_arr = a_arr[mask], p_arr[mask]
    if a_arr.size == 0 or n_bins < 1:
        return []

    lo, hi = float(a_arr.min()), float(a_arr.max())
    if hi - lo < 1e-12:
        hi = lo + 1e-12
    edges = np.linspace(lo, hi, n_bins + 1)
    # Rightmost point belongs to the last bin, not a phantom bin past it.
    idx = np.clip(np.digitize(a_arr, edges[1:-1]), 0, n_bins - 1)

    out = []
    for b in range(n_bins):
        sel = p_arr[idx == b]
        entry = {
            "start_mm": float(edges[b]),
            "end_mm": float(edges[b + 1]),
            "count": int(sel.size),
        }
        if sel.size:
            entry.update(
                mean_mm=float(sel.mean()),
                rms_mm=float(np.sqrt(np.mean(sel ** 2))),
                max_abs_mm=float(np.max(np.abs(sel))),
            )
        else:
            entry.update(mean_mm=None, rms_mm=None, max_abs_mm=None)
        out.append(entry)
    return out


# ------------------------------------------------------- cart rotation
def relative_rotation_deg(qx, qy, qz, qw) -> Optional[np.ndarray]:
    """
    Total cart rotation (degrees) of each sample relative to the FIRST
    sample that carried a usable orientation.

    This is the full 3D rotation angle, deliberately NOT decomposed into
    yaw about the page normal: that decomposition needs the calibration's
    ``e_col``/``e_row``/boresight (see ``rotation.yaw_about_normal``), which
    a CSV alone does not carry. The consequence is stated wherever this is
    used: the lever-arm deviation derived from it is an UPPER BOUND, since
    roll and pitch are included here but do not swing the nozzle bar across
    the page the way yaw does.

    Samples with a missing/blank quaternion (``nan``, written as ``,,,`` by
    the profiler) come back as ``nan`` rather than being silently treated as
    "no rotation". Returns ``None`` if no sample has a usable quaternion.
    """
    q = np.column_stack([np.asarray(a, dtype=float) for a in (qx, qy, qz, qw)])
    usable = np.isfinite(q).all(axis=1) & (np.linalg.norm(q, axis=1) > 1e-9)
    if not usable.any():
        return None

    ref = _normalize_quat(q[np.argmax(usable)])
    ref_conj = _quat_conjugate(ref)

    out = np.full(q.shape[0], np.nan)
    for i in range(q.shape[0]):
        if not usable[i]:
            continue
        rel = _quat_multiply(_normalize_quat(q[i]), ref_conj)
        # Rotation angle of a unit quaternion: 2*acos(|w|). The abs folds
        # q and -q (the same rotation) onto the same angle, so a sign flip
        # in the sensor's output never reads as a 360-degree swing.
        out[i] = math.degrees(2.0 * math.acos(min(1.0, abs(rel[3]))))
    return out


def lever_arm_mm(rotation_deg: float,
                 lever_mm: float = abs(SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM)) -> float:
    """
    Apparent position shift of the nozzle-bar-referenced point produced by
    rotating the cart ``rotation_deg`` degrees, through the fixed
    sensor->nozzle-bar lever arm (arc length ``r * theta``).

    On this rig ``lever_mm`` is 62.36 mm, so one degree is ~1.09 mm -- which
    is why cart rotation, not tracker noise, is usually the largest term in
    a freehand straightness measurement.
    """
    return abs(lever_mm) * math.radians(abs(rotation_deg))


# ------------------------------------------------------------- top level
def analyze(data: Dict[str, np.ndarray], n_bins: int = 10) -> dict:
    """
    Full straightness analysis of a parsed page-mode profile CSV.

    Returns a dict of everything the report prints (see ``format_report``),
    or a dict with ``"error"`` set when the data cannot support a fit at
    all. Kept separate from the printing so the numbers can be asserted
    directly in tests and reused by any other consumer.
    """
    u, v = data.get("u_mm"), data.get("v_mm")
    if u is None or v is None:
        return {"error": "CSV has no u_mm/v_mm columns."}

    fit = fit_line_tls(u, v)
    if fit is None:
        return {"error": ("Not enough distinct points to fit a line "
                          f"({np.size(u)} sample(s)).")}

    along, perp = project_to_line(u, v, fit)
    finite = np.isfinite(along) & np.isfinite(perp)
    along_f, perp_f = along[finite], perp[finite]
    if along_f.size < 2:
        return {"error": "Fewer than 2 usable samples after filtering."}

    travel = float(along_f.max() - along_f.min())
    abs_perp = np.abs(perp_f)

    result = {
        "sample_count": int(along_f.size),
        "travel_mm": travel,
        "angle_deg": float(fit["angle_deg"]),
        "rms_mm": float(np.sqrt(np.mean(perp_f ** 2))),
        "max_abs_mm": float(abs_perp.max()),
        "p95_abs_mm": float(np.percentile(abs_perp, 95)),
        "span_mm": float(perp_f.max() - perp_f.min()),
        "bow": fit_bow(along_f, perp_f),
        "bins": binned_profile(along_f, perp_f, n_bins),
        "enough_travel": travel >= MIN_TRAVEL_MM,
    }

    # Cart rotation, when the CSV carries orientation.
    rot = None
    if all(k in data for k in ("qx", "qy", "qz", "qw")):
        rot = relative_rotation_deg(data["qx"], data["qy"],
                                    data["qz"], data["qw"])
    if rot is not None and np.isfinite(rot).sum() >= 2:
        rot_f = rot[np.isfinite(rot)]
        rot_span = float(rot_f.max() - rot_f.min())
        # Correlate rotation against |deviation| on the samples where both
        # exist -- if the two move together, the lever arm is the cause.
        both = finite & np.isfinite(rot)
        corr = None
        if both.sum() >= 2:
            a, b = rot[both], np.abs(perp[both])
            if float(np.std(a)) > 1e-9 and float(np.std(b)) > 1e-9:
                corr = float(np.corrcoef(a, b)[0, 1])
        result["rotation"] = {
            "span_deg": rot_span,
            "max_deg": float(rot_f.max()),
            "lever_arm_mm": lever_arm_mm(rot_span),
            "correlation_with_deviation": corr,
        }
    else:
        result["rotation"] = None

    return result


def _rows(mm: float) -> float:
    """Millimetres expressed in nozzle rows -- the unit that decides whether
    a deviation can show up in a print at all (see GOOD_ROWS/FAIR_ROWS)."""
    return mm / NOZZLE_PITCH_MM


def format_report(result: dict) -> str:
    """Human-readable report, in the style of the other diagnostics."""
    if "error" in result:
        return f"[straightness] {result['error']}"

    L: List[str] = []
    L.append("---- Geradheit / Tracking-Präzision ----")
    L.append(f"  Samples              : {result['sample_count']}")
    L.append(f"  Strecke entlang Linie: {result['travel_mm']:.1f} mm")
    L.append(f"  Linienwinkel         : {result['angle_deg']:+.2f} deg "
             f"(gegen die +u-Achse)")
    L.append("")
    L.append("  Abweichung von der Ausgleichsgeraden (senkrecht):")
    L.append(f"    RMS                : {result['rms_mm']:.3f} mm "
             f"({_rows(result['rms_mm']):.1f} Düsenreihen)")
    L.append(f"    p95                : {result['p95_abs_mm']:.3f} mm "
             f"({_rows(result['p95_abs_mm']):.1f} Reihen)")
    L.append(f"    max                : {result['max_abs_mm']:.3f} mm "
             f"({_rows(result['max_abs_mm']):.1f} Reihen)")
    L.append(f"    Spanne (min..max)  : {result['span_mm']:.3f} mm")

    bow = result.get("bow")
    if bow:
        L.append("")
        L.append("  Aufteilung systematisch / zufällig:")
        L.append(f"    Krümmung (Bogen)   : {bow['bow_mm']:.3f} mm "
                 f"({_rows(bow['bow_mm']):.1f} Reihen) über die ganze Strecke")
        L.append(f"    systematisch (RMS) : {bow['systematic_rms_mm']:.3f} mm"
                 f"   <- hat eine Form, mittelt sich NICHT weg")
        L.append(f"    zufällig (RMS)     : {bow['random_rms_mm']:.3f} mm"
                 f"   <- Jitter, teils durch --smooth-ms gedämpft")

    bins = result.get("bins") or []
    if bins:
        L.append("")
        L.append("  Abweichung nach Position entlang der Linie:")
        L.append("    von .. bis (mm)   n     Mittel      RMS      max")
        for b in bins:
            rng = f"{b['start_mm']:8.1f} ..{b['end_mm']:7.1f}"
            if b["count"]:
                L.append(f"    {rng} {b['count']:5d} "
                         f"{b['mean_mm']:+9.3f} {b['rms_mm']:8.3f} "
                         f"{b['max_abs_mm']:8.3f}")
            else:
                L.append(f"    {rng} {0:5d}         -        -        -")

    rot = result.get("rotation")
    L.append("")
    if rot is None:
        L.append("  Wagen-Drehung        : keine Orientierung im CSV "
                 "(qx/qy/qz/qw fehlen oder sind leer) -- der Hebelarm-Anteil "
                 "kann nicht abgeschätzt werden.")
    else:
        L.append("  Wagen-Drehung (Hebelarm-Effekt):")
        L.append(f"    Drehung Spanne     : {rot['span_deg']:.2f} deg")
        L.append(f"    daraus rechnerisch : bis zu {rot['lever_arm_mm']:.3f} mm "
                 f"scheinbare Abweichung")
        L.append(f"                         (Obergrenze: {abs(SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM):.2f} mm "
                 f"Hebel, 3D-Gesamtdrehung, nicht nur Gierwinkel)")
        if rot["correlation_with_deviation"] is None:
            L.append("    Korrelation        : n/a (zu wenig Variation)")
        else:
            L.append(f"    Korrelation mit |Abweichung|: "
                     f"{rot['correlation_with_deviation']:+.2f}")

    L.append("")
    L.extend(_verdict_lines(result))
    return "\n".join(L)


def _verdict_lines(result: dict) -> List[str]:
    """
    Verdict, ordered so a result that cannot be judged says so FIRST --
    a 5 mm dab is trivially "straight" and must not read as an all-clear
    (same ordering rule as diagnostics._calibration_check_summary's
    travel guard).
    """
    out: List[str] = []
    if not result["enough_travel"]:
        out.append(f"  VERDICT: zu kurze Strecke ({result['travel_mm']:.1f} mm < "
                   f"{MIN_TRAVEL_MM:.0f} mm) -- über so wenig Weg ist fast "
                   f"alles gerade. Fahre das Lineal weiter ab und miss erneut.")
        return out

    rms_rows = _rows(result["rms_mm"])
    if rms_rows <= GOOD_ROWS:
        head = (f"  VERDICT: sehr gerade -- RMS {result['rms_mm']:.3f} mm liegt "
                f"unter einer Düsenreihe ({NOZZLE_PITCH_MM:.4f} mm) und kann "
                f"im Druck nicht sichtbar werden.")
    elif rms_rows <= FAIR_ROWS:
        head = (f"  VERDICT: brauchbar -- RMS {result['rms_mm']:.3f} mm sind "
                f"{rms_rows:.1f} Düsenreihen, bei genauem Hinsehen sichtbar.")
    else:
        head = (f"  VERDICT: deutlich krumm -- RMS {result['rms_mm']:.3f} mm sind "
                f"{rms_rows:.1f} Düsenreihen.")
    out.append(head)

    # Which term dominates -- the actionable half of the verdict.
    rot = result.get("rotation")
    if rot and rot["span_deg"] >= ROTATION_NOTABLE_DEG:
        out.append(f"           Der Wagen hat sich um {rot['span_deg']:.2f} deg "
                   f"gedreht; allein das erklärt über den {abs(SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM):.1f} mm "
                   f"Hebelarm bis zu {rot['lever_arm_mm']:.3f} mm -- vergleichbar "
                   f"mit oder größer als die gemessene Abweichung. Wiederhole "
                   f"den Lauf und halte den Wagen bewusst verdrehungsfrei, "
                   f"bevor du das dem Tracking zuschreibst.")

    bow = result.get("bow")
    if bow and bow["systematic_rms_mm"] > bow["random_rms_mm"]:
        out.append(f"           Überwiegend SYSTEMATISCH ({bow['bow_mm']:.3f} mm "
                   f"Bogen): ein gleichmäßiger Verzug, kein Rauschen. Typisch "
                   f"für Feldverzerrung (Metall in der Nähe?) oder ein nicht "
                   f"gerades Lineal -- mittelt sich nicht weg.")
    elif bow:
        out.append(f"           Überwiegend ZUFÄLLIG (Jitter-RMS "
                   f"{bow['random_rms_mm']:.3f} mm): Sensorrauschen. "
                   f"Ein größeres --smooth-ms dämpft das (auf Kosten von "
                   f"Nachlauf).")

    out.append("           Hinweis: diese Zahl ist eine OBERGRENZE für den "
               "Tracking-Fehler -- Handführung am Lineal und die Geradheit "
               "des Lineals selbst stecken mit drin.")
    return out


def analyze_csv(path: str, n_bins: int = 10) -> str:
    """Read ``path`` and return the formatted report (the CLI entry point)."""
    try:
        data = read_profile_csv(path)
    except (OSError, ValueError) as exc:
        return f"[straightness] {exc}"
    return format_report(analyze(data, n_bins=n_bins))
