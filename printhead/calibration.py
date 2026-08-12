"""
Page-plane calibration for freehand page-mode printing
========================================================

The 1D pipeline locks a single travel direction from the first
``calib_distance_mm`` of motion (see ``AdvanceMapper.auto_calibrate`` in
``tracking.py``). Freehand page mode needs a full 2D page frame instead: the
user traces two adjacent edges of a sheet of paper with the tracked cart, and
this module fits an orthonormal ``(origin, e_col, e_row)`` frame from the two
traces, optionally scale-corrected against the sheet's known size.

Tracing a whole edge and fitting a line through it (PCA via SVD) is used
instead of tapping two endpoints or taking a plain start/end delta, because
hand tremor at a single tapped point -- or at just the two ends of a trace --
would otherwise go straight into the fitted direction uncorrected; averaging
over the whole trace cancels most of that out. This generalises the same
"collect real motion before locking a direction" idea ``AdvanceMapper.
auto_calibrate`` already uses, from one axis to two.
"""

from __future__ import annotations

import json
import math
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

# How far the two raw traced edges may deviate from perpendicular before
# calibrate_page() warns. A skewed trace or a non-rectangular sheet produces
# a real angle error here -- Gram-Schmidt forces orthogonality regardless,
# but silently doing so past this point would hide a bad calibration.
MAX_ANGLE_ERROR_DEG = 15.0

# Below these, a traced edge's samples don't give fit_axis's PCA line fit
# enough to reliably pin down the true edge direction -- calibrate_page()
# warns rather than errors (same "still usable, but suspect" philosophy as
# MAX_ANGLE_ERROR_DEG above; see CalibrationQualityWarning).
#
# Measured (synthetic straight edge, varying traced length / per-sample
# noise / sample count -> the resulting error in the FITTED page normal,
# 200 trials each, "max" = worst of the 200):
#
#  edge length  noise  samples | resulting page-normal error
#     210 mm   0.05mm    200   |   0.00 deg (max 0.01)
#     100 mm   0.5 mm    100   |   0.12 deg (max 0.37)
#      50 mm   1.0 mm     50   |   0.65 deg (max 1.40)
#      30 mm   2.0 mm     30   |   3.16 deg (max 6.25)
#      20 mm   3.0 mm     20   |   7.23 deg (max 18.63)
#
# and separately measured: yaw_error ~= tilt_angle * sin(page_normal_error).
# This is WHY a bad normal matters even though nothing in this codebase
# corrects roll/pitch directly (see rotation.cart_rotation_angles's
# docstring): yaw is measured about the fitted normal (rotation.
# yaw_about_normal), so a normal that is off by even a couple of degrees
# turns ordinary tracker tilt NOISE (rotation.py's module docstring:
# median 2.7 deg, max 7.8 deg on this rig) into apparent YAW error --
# tilt leaking into the one angle print correction actually trusts.
#
# The thresholds below sit at the "50mm / 1.0mm / 20 samples" row: still
# comfortably under a degree of normal error (0.65, max 1.40), the last row
# of the table before error starts climbing steeply. NOTE: the operator's
# own real calibration measures 0.63 deg normal tilt and 0.92 deg
# orthogonality error -- GOOD, well clear of these thresholds. They exist to
# catch a bad calibration in general, not to explain any current yaw
# problem on this rig (see rotation.py for that -- a 180-degree singularity
# in the old yaw math, unrelated to calibration quality).
MIN_TRACE_LENGTH_MM = 50.0
MAX_RMS_RESIDUAL_MM = 1.0
MIN_SAMPLE_COUNT = 20


def fit_axis(samples: np.ndarray) -> "tuple[np.ndarray, np.ndarray]":
    """
    Fit a straight line through an ``(N, 3)`` mm trace along one page edge.

    Returns ``(origin, direction)``: ``origin`` is the trace's first sample,
    ``direction`` is a unit vector along the PCA-fitted line, oriented from
    the first sample towards the last (not just whichever way SVD happens to
    return, which is sign-ambiguous).
    """
    samples = np.asarray(samples, dtype=float)
    if samples.ndim != 2 or samples.shape[1] != 3 or samples.shape[0] < 2:
        raise ValueError(f"fit_axis needs an (N>=2, 3) array, got shape {samples.shape}")

    origin = samples[0]
    centered = samples - samples.mean(axis=0)
    _, s, vt = np.linalg.svd(centered, full_matrices=False)
    # vt's rows are unit-norm by construction even when the input is
    # degenerate (SVD still returns *some* orthonormal basis for an all-zero
    # matrix) -- the largest singular value, not the direction's own norm, is
    # what actually says whether the samples spread out along a line at all.
    if s[0] < 1e-9:
        raise ValueError("fit_axis: samples are degenerate (all at one point)")
    direction = vt[0]
    if np.dot(samples[-1] - samples[0], direction) < 0:
        direction = -direction
    return origin, direction


def trace_length_mm(samples: np.ndarray, direction: np.ndarray) -> float:
    """
    Extent of ``samples`` projected onto ``direction``, i.e. how long the
    traced edge is in the tracker's own mm units (max - min projection).
    Used to derive a scale correction against a known real-world length.
    """
    samples = np.asarray(samples, dtype=float)
    proj = (samples - samples[0]) @ direction
    return float(proj.max() - proj.min())


@dataclass
class AxisFitQuality:
    """
    How good a single ``fit_axis()`` line fit actually is, as three numbers
    an operator (and ``calibrate_page()``'s warnings, see below) can act on.

    ``length_mm``: same as ``trace_length_mm(samples, direction)`` -- how
    far the trace actually spans along its own fitted direction. A short
    trace lets hand tremor and sensor noise dominate the fit.

    ``rms_residual_mm``: RMS of each sample's PERPENDICULAR distance from
    the fitted line -- how much the samples scatter off the straight line
    ``fit_axis`` found, not how far they travelled along it. A large value
    means either a genuinely wobbly trace or a sheet edge that is not
    actually straight, either way a fit worth a second look.

    ``sample_count``: how many samples went into the fit -- few samples let
    a single noisy one skew the whole PCA fit disproportionately.
    """
    length_mm: float
    rms_residual_mm: float
    sample_count: int


def fit_axis_quality(samples: np.ndarray, direction: np.ndarray) -> AxisFitQuality:
    """
    Quality of a ``fit_axis()`` line fit -- see :class:`AxisFitQuality` for
    what each number means.

    A SEPARATE function from ``fit_axis`` rather than a change to its return
    value: ``fit_axis``'s existing 2-value ``(origin, direction)`` return is
    relied on by ``calibrate_page`` and by every existing direct caller
    (``tests/test_calibration.py`` included) exactly as it is today -- widening
    it to 3 values would break all of them. This takes the same ``samples``/
    ``direction`` a caller already has in hand (typically straight from
    ``fit_axis`` itself) and computes quality from them for free, without
    ``fit_axis`` needing to change at all.
    """
    samples = np.asarray(samples, dtype=float)
    direction = np.asarray(direction, dtype=float)
    length_mm = trace_length_mm(samples, direction)

    # Perpendicular residual: distance of each sample from the FITTED LINE,
    # i.e. measured relative to the sample CENTROID, which is the point
    # fit_axis's PCA line actually passes through (it SVDs
    # `samples - samples.mean(axis=0)`).
    #
    # CORRECTION: this first measured residuals relative to `samples[0]`
    # instead -- copying trace_length_mm's "relative to the first sample"
    # convention, which is harmless THERE (it takes max-minus-min, so the
    # reference point cancels) but wrong here, because samples[0] is just
    # another noisy sample, not a point on the line. Measured consequences
    # of that version, all three of which made this metric unusable as the
    # threshold it feeds (MAX_RMS_RESIDUAL_MM):
    #   * a systematic 1.33x inflation (400 trials/level, perpendicular
    #     noise sigma 0.1-0.8mm) -- it reported each sample's distance from
    #     sample 0 rather than from the line, so sample 0's own noise was
    #     added to every residual, and a real 0.71mm trace tripped the
    #     1.0mm threshold;
    #   * order dependence: the SAME 60 points, merely rotated so a
    #     different sample came first, gave 0.49 to 1.04mm -- a "quality"
    #     number that straddled its own threshold based on nothing but
    #     which sample arrived first;
    #   * a single outlier landing FIRST reported 4.97mm against a true
    #     0.81mm -- a 6x false alarm on an otherwise clean trace.
    # The centroid has none of those properties: it is what the fit is
    # actually anchored to, and no single sample can move it far.
    rel = samples - samples.mean(axis=0)
    along = rel @ direction
    perp = rel - np.outer(along, direction)
    rms_residual_mm = float(np.sqrt(np.mean(np.sum(perp * perp, axis=1))))

    return AxisFitQuality(length_mm=length_mm, rms_residual_mm=rms_residual_mm,
                          sample_count=int(samples.shape[0]))


class CalibrationAngleWarning(UserWarning):
    """
    The two traced page edges are far from perpendicular. Raised as a
    warning rather than an error: Gram-Schmidt still produces a valid
    orthogonal frame, but a large error usually means a skewed trace or a
    non-rectangular sheet, not just sensor noise -- worth a human look
    before trusting the calibration.
    """


class CalibrationQualityWarning(UserWarning):
    """
    A traced edge is short, noisy, or sparse enough (see
    ``MIN_TRACE_LENGTH_MM``/``MAX_RMS_RESIDUAL_MM``/``MIN_SAMPLE_COUNT`` for
    the measurement behind these thresholds) that ``fit_axis``'s line fit
    may not reliably reflect the true page edge. A sibling of
    ``CalibrationAngleWarning`` rather than folded into it: the two flag
    genuinely different problems (edges that do not meet at ~90 degrees, vs.
    an edge whose OWN fit is shaky regardless of the other edge), and a
    caller may reasonably want to treat them differently. Raised as a
    warning, not an error, for the same reason ``CalibrationAngleWarning``
    is: the fit still produces SOME frame, just one worth a human look
    before trusting it.
    """


# Shared by PageCalibration.to_dict/from_dict below -- the optional
# fit-quality field names, kept in one place so the two stay in sync.
_QUALITY_FIELDS = (
    "col_trace_length_mm", "row_trace_length_mm",
    "col_rms_residual_mm", "row_rms_residual_mm",
    "col_sample_count", "row_sample_count",
    "normal_tilt_deg",
)


@dataclass
class PageCalibration:
    """
    An orthonormal 2D page coordinate frame, in the tracker's 3D world space.

    ``e_col``/``e_row`` are unit vectors (mm world space) along the page's
    column ("u", travel/sweep) and row ("v", along the 152-nozzle bar) axes.
    ``scale_col``/``scale_row`` correct a systematic tracker scale error (raw
    sensor mm -> true mm), derived from a known sheet size instead of a
    manually typed reference distance; both default to 1.0 (trust the
    tracker's own mm) when no known sheet size was given.

    Not to be confused with ``TrackingSettings.mm_per_column``, which is the
    *image* resolution of a print job (how many mm of paper one printed
    column spans), not a tracker scale correction.

    The remaining fields are OPTIONAL fit-quality metrics (see
    ``fit_axis_quality``/``calibrate_page``): trace length, RMS
    perpendicular residual and sample count for each traced edge, plus the
    fitted page normal's tilt (degrees) away from the tracker's own z axis.
    They default to ``None``, not ``0.0`` -- a calibration built directly
    (as most of this file's own tests do) or loaded from a JSON saved before
    this feature existed genuinely has no measured quality, and ``None``
    keeps that visible rather than letting it read as "measured perfect".
    Diagnostic only, same as ``angle_error_deg``: nothing here feeds
    ``project()``'s math.
    """
    origin: np.ndarray
    e_col: np.ndarray
    e_row: np.ndarray
    scale_col: float = 1.0
    scale_row: float = 1.0
    boresight_quat: Optional[np.ndarray] = None
    angle_error_deg: float = 0.0
    col_trace_length_mm: Optional[float] = None
    row_trace_length_mm: Optional[float] = None
    col_rms_residual_mm: Optional[float] = None
    row_rms_residual_mm: Optional[float] = None
    col_sample_count: Optional[int] = None
    row_sample_count: Optional[int] = None
    normal_tilt_deg: Optional[float] = None

    @classmethod
    def simple_frame(cls, boresight_quat: Optional[np.ndarray] = None) -> "PageCalibration":
        """
        The calibration-free ("simple") page frame: the tracker's own axes
        taken as the page's, with no edge tracing at all.

        ``e_col``/``e_row`` are the raw Amfitrack **x** and **y** axes, so
        ``project()`` degenerates to "u = x, v = y, z = z" (minus the origin,
        which ``tracking.PageMapper.zero_at_nozzle`` zeroes at pass start --
        see there).

        ``boresight_quat`` defaults to **None**, auto-captured from whatever
        pose the cart happens to be in at the first live sample of a pass or
        ``--pos`` run (``PageMapper.capture_boresight``, gated on ``is
        None`` by the caller so a pre-supplied one below is never
        overwritten). It is deliberately NOT the identity quaternion, which
        was the first (wrong) attempt: identity means "the reference pose is
        the world frame itself", but the sensor is mounted rotated on the
        cart (measured on the real rig: 120.1 deg about [0.553, 0.589,
        -0.590]), so the whole mounting rotation stayed inside every
        reported angle -- a flat 90 deg turn read as only ~70 deg of yaw
        change, non-linearly, with roll/pitch swinging tens of degrees, and
        since that yaw also drives the sensor->nozzle offset rotation it
        placed ink wrongly too. Pinned by ``tests/test_calibration.py``.

        Passing ``boresight_quat`` explicitly (from ``--simple-boresight``,
        see ``cli.py``) exists because blind first-sample auto-capture is
        itself unreliable in the field: whatever pose the cart happens to be
        in at that exact instant (BLE still settling, hand not yet fully
        still) silently becomes "0 deg", with no way to notice or fix it
        short of restarting the whole pass and hoping. A supplied value lets
        the operator capture once via ``--pos``, visually confirm roll/pitch/
        yaw read ~0 while genuinely flat, and only then pin that exact
        quaternion into the real print -- reusing the same "capture, look at
        the number, decide it's good" workflow the traced calibration's own
        boresight button already gives, rather than trusting an unattended
        first sample.

        The trade-off versus a traced calibration is explicit: this assumes
        the sheet is laid out square with the tracker's x/y axes, and that
        the tracker's mm are true mm (``scale_col``/``scale_row`` stay 1.0,
        since there is no known sheet size to derive a correction from). In
        exchange it needs no edge-tracing step, so it cannot inherit a bad
        edge trace -- the reason it exists (measured yaw itself is accurate
        to ~1 deg once the reference pose is right, see the README's
        "Einfacher Modus" section).
        """
        return cls(
            origin=np.zeros(3, dtype=float),
            e_col=np.array([1.0, 0.0, 0.0]),
            e_row=np.array([0.0, 1.0, 0.0]),
            boresight_quat=(np.array(boresight_quat, dtype=float)
                            if boresight_quat is not None else None),
        )

    def project(self, pos) -> "tuple[float, float, float]":
        """
        World position (mm) -> page-plane ``(u_mm, v_mm, z_mm)``.

        ``u``/``v`` are distances along ``e_col``/``e_row`` from ``origin``,
        scale-corrected. ``z`` is the signed distance off the page plane
        (along ``e_col x e_row``) -- diagnostic only (nozzle standoff), not
        used to place ink.
        """
        rel = np.asarray(pos, dtype=float) - self.origin
        u = float(np.dot(rel, self.e_col)) * self.scale_col
        v = float(np.dot(rel, self.e_row)) * self.scale_row
        z = float(np.dot(rel, np.cross(self.e_col, self.e_row)))
        return u, v, z

    def to_dict(self) -> dict:
        d = {
            "origin": self.origin.tolist(),
            "e_col": self.e_col.tolist(),
            "e_row": self.e_row.tolist(),
            "scale_col": self.scale_col,
            "scale_row": self.scale_row,
            "angle_error_deg": self.angle_error_deg,
        }
        if self.boresight_quat is not None:
            d["boresight_quat"] = self.boresight_quat.tolist()
        # Quality metrics: only written when actually present, so a
        # calibration built or loaded without them (see the dataclass
        # docstring) round-trips with no fabricated numbers rather than
        # writing out a fake 0.0/None-as-string.
        for key in _QUALITY_FIELDS:
            value = getattr(self, key)
            if value is not None:
                d[key] = value
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "PageCalibration":
        quat = d.get("boresight_quat")
        return cls(
            origin=np.array(d["origin"], dtype=float),
            e_col=np.array(d["e_col"], dtype=float),
            e_row=np.array(d["e_row"], dtype=float),
            scale_col=float(d.get("scale_col", 1.0)),
            scale_row=float(d.get("scale_row", 1.0)),
            boresight_quat=np.array(quat, dtype=float) if quat is not None else None,
            angle_error_deg=float(d.get("angle_error_deg", 0.0)),
            # .get(...) with no default -> stays None for any JSON saved
            # before this feature existed (the operator has one), or built
            # without going through calibrate_page -- see the dataclass
            # docstring for why None, not a fabricated 0.0, is required here.
            col_trace_length_mm=d.get("col_trace_length_mm"),
            row_trace_length_mm=d.get("row_trace_length_mm"),
            col_rms_residual_mm=d.get("col_rms_residual_mm"),
            row_rms_residual_mm=d.get("row_rms_residual_mm"),
            col_sample_count=d.get("col_sample_count"),
            row_sample_count=d.get("row_sample_count"),
            normal_tilt_deg=d.get("normal_tilt_deg"),
        )

    def save(self, path) -> None:
        """Write this calibration as JSON, so a print session doesn't need to
        re-trace the page edges every time (as long as paper/cart didn't move)."""
        Path(path).write_text(json.dumps(self.to_dict(), indent=2))

    @classmethod
    def load(cls, path) -> "PageCalibration":
        return cls.from_dict(json.loads(Path(path).read_text()))


def calibrate_page(col_samples: np.ndarray, row_samples: np.ndarray,
                    sheet_width_mm: Optional[float] = None,
                    sheet_height_mm: Optional[float] = None,
                    max_angle_error_deg: float = MAX_ANGLE_ERROR_DEG,
                    boresight_quat: Optional[np.ndarray] = None,
                    min_trace_length_mm: float = MIN_TRACE_LENGTH_MM,
                    max_rms_residual_mm: float = MAX_RMS_RESIDUAL_MM,
                    min_sample_count: int = MIN_SAMPLE_COUNT,
                    ) -> PageCalibration:
    """
    Fit a ``PageCalibration`` from two traced page edges.

    ``col_samples``/``row_samples`` are ``(N, 3)`` mm traces along the two
    edges (column/"u" direction and row/"v" direction, i.e. along the
    152-nozzle bar), both starting at the shared page corner.

    If ``sheet_width_mm``/``sheet_height_mm`` are given (e.g. A4 portrait ~=
    210 x 297), the scale is corrected against the sheet's known size instead
    of trusting the tracker's raw mm -- this replaces manually typing in a
    reference distance, the way the 1D ``AdvanceMapper.auto_calibrate`` does
    via ``calib_distance_mm``.

    ``boresight_quat``, if given, is stored as-is on the returned
    ``PageCalibration`` (see its docstring / ``rotation.yaw_about_normal``):
    the orientation quaternion captured with the cart held flat on the page,
    nozzle bar aligned along the traced row edge -- the reference pose cart
    yaw is measured relative to. Left ``None`` (the default) when the caller
    has not captured one yet, which keeps rotation correction off entirely
    for the resulting calibration (see ``tracking.PageMapper``) rather than
    guessing a reference pose.

    The returned calibration also carries fit-quality metrics (see
    ``AxisFitQuality``/``PageCalibration``'s own docstring): each edge's
    traced length, RMS perpendicular residual and sample count, plus the
    fitted page normal's tilt away from the tracker's z axis. These are
    diagnostic only -- computed from the SAME ``fit_axis`` output the frame
    itself is built from, never fed back into ``origin``/``e_col``/``e_row``.

    Raises ``CalibrationAngleWarning`` (not an error) if the raw traces are
    more than ``max_angle_error_deg`` away from perpendicular, and/or
    ``CalibrationQualityWarning`` (also not an error) if either traced edge
    is shorter than ``min_trace_length_mm``, has an RMS residual above
    ``max_rms_residual_mm``, or has fewer than ``min_sample_count`` samples
    (see the module-level threshold comment for the measurement these are
    based on). Either way the returned calibration is still usable -- both
    are a "worth a human look" signal, not a hard failure.
    """
    origin, e_col_raw = fit_axis(col_samples)
    _, e_row_raw = fit_axis(row_samples)
    col_quality = fit_axis_quality(col_samples, e_col_raw)
    row_quality = fit_axis_quality(row_samples, e_row_raw)

    cos_angle = float(np.clip(np.dot(e_col_raw, e_row_raw), -1.0, 1.0))
    angle_deg = math.degrees(math.acos(cos_angle))
    angle_error_deg = angle_deg - 90.0
    if abs(angle_error_deg) > max_angle_error_deg:
        warnings.warn(
            f"Traced edges are {angle_deg:.1f} deg apart (expected ~90 deg, "
            f"error {angle_error_deg:+.1f} deg) -- check the trace, or the "
            f"sheet may not be rectangular. Axes were still Gram-Schmidt "
            f"orthogonalised.",
            CalibrationAngleWarning, stacklevel=2)

    quality_issues = []
    for label, quality in (("column", col_quality), ("row", row_quality)):
        if quality.length_mm < min_trace_length_mm:
            quality_issues.append(
                f"{label} trace is only {quality.length_mm:.0f}mm long "
                f"(< {min_trace_length_mm:.0f}mm)")
        if quality.rms_residual_mm > max_rms_residual_mm:
            quality_issues.append(
                f"{label} trace RMS residual is {quality.rms_residual_mm:.2f}mm "
                f"(> {max_rms_residual_mm:.1f}mm)")
        if quality.sample_count < min_sample_count:
            quality_issues.append(
                f"{label} trace has only {quality.sample_count} samples "
                f"(< {min_sample_count})")
    if quality_issues:
        warnings.warn(
            "Calibration fit quality is low: " + "; ".join(quality_issues) +
            " -- a short, noisy, or sparse trace makes the fitted page "
            "normal less reliable, which can leak ordinary tracker tilt "
            "into apparent yaw error (see the MIN_TRACE_LENGTH_MM comment "
            "in calibration.py). Still usable, but worth re-tracing the "
            "flagged edge(s) more slowly and/or over a longer span.",
            CalibrationQualityWarning, stacklevel=2)

    # Gram-Schmidt: trust e_col as fitted, force e_row perpendicular to it.
    e_col = e_col_raw
    e_row = e_row_raw - np.dot(e_row_raw, e_col) * e_col
    row_norm = float(np.linalg.norm(e_row))
    if row_norm < 1e-9:
        raise ValueError("Row trace is parallel to the column trace; cannot "
                          "build an orthogonal page frame.")
    e_row = e_row / row_norm

    # Page-normal tilt vs. the tracker's own z axis: the ACUTE angle between
    # the two (folded into 0..90 via abs()), so a page normal that happens
    # to point "down" rather than "up" still reads as near-0 tilt for a
    # near-flat page, not near-180. Diagnostic only, like angle_error_deg --
    # a page genuinely does not have to lie flat on the tracker's own xy
    # plane for calibration to work, this just flags when it doesn't.
    normal = np.cross(e_col, e_row)
    tracker_z = np.array([0.0, 0.0, 1.0])
    normal_tilt_deg = math.degrees(math.acos(float(np.clip(abs(np.dot(normal, tracker_z)), 0.0, 1.0))))

    scale_col = 1.0
    scale_row = 1.0
    if sheet_width_mm is not None:
        measured = trace_length_mm(col_samples, e_col)
        if measured < 1e-6:
            raise ValueError("Column trace has ~zero length; cannot derive scale.")
        scale_col = sheet_width_mm / measured
    if sheet_height_mm is not None:
        measured = trace_length_mm(row_samples, e_row)
        if measured < 1e-6:
            raise ValueError("Row trace has ~zero length; cannot derive scale.")
        scale_row = sheet_height_mm / measured

    return PageCalibration(
        origin=origin, e_col=e_col, e_row=e_row,
        scale_col=scale_col, scale_row=scale_row,
        boresight_quat=(np.asarray(boresight_quat, dtype=float)
                        if boresight_quat is not None else None),
        angle_error_deg=angle_error_deg,
        col_trace_length_mm=col_quality.length_mm,
        row_trace_length_mm=row_quality.length_mm,
        col_rms_residual_mm=col_quality.rms_residual_mm,
        row_rms_residual_mm=row_quality.rms_residual_mm,
        col_sample_count=col_quality.sample_count,
        row_sample_count=row_quality.sample_count,
        normal_tilt_deg=normal_tilt_deg)
