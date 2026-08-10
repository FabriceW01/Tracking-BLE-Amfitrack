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


class CalibrationAngleWarning(UserWarning):
    """
    The two traced page edges are far from perpendicular. Raised as a
    warning rather than an error: Gram-Schmidt still produces a valid
    orthogonal frame, but a large error usually means a skewed trace or a
    non-rectangular sheet, not just sensor noise -- worth a human look
    before trusting the calibration.
    """


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
    """
    origin: np.ndarray
    e_col: np.ndarray
    e_row: np.ndarray
    scale_col: float = 1.0
    scale_row: float = 1.0
    boresight_quat: Optional[np.ndarray] = None
    angle_error_deg: float = 0.0

    @classmethod
    def simple_frame(cls) -> "PageCalibration":
        """
        The calibration-free ("simple") page frame: the tracker's own axes
        taken as the page's, with no edge tracing at all.

        ``e_col``/``e_row`` are the raw Amfitrack **x** and **y** axes, so
        ``project()`` degenerates to "u = x, v = y, z = z" (minus the origin,
        which ``tracking.PageMapper.set_origin`` zeroes at pass start -- see
        there). ``boresight_quat`` is the IDENTITY quaternion, which makes
        ``rotation.yaw_about_normal`` return exactly the cart's rotation
        about the tracker's **z** axis: with ``R(identity) == I`` the
        relative rotation ``R_rel`` reduces to ``R(quat)`` itself, and the
        page normal ``e_col x e_row`` is exactly ``z``. Verified bit-exact
        against pure z-rotations in ``tests/test_calibration.py``.

        The trade-off versus a traced calibration is explicit: this assumes
        the sheet is laid out square with the tracker's x/y axes, and that
        the tracker's mm are true mm (``scale_col``/``scale_row`` stay 1.0,
        since there is no known sheet size to derive a correction from). In
        exchange it needs no calibration step, so it cannot inherit a bad
        one -- the reason it exists (measured yaw itself is accurate to
        ~1 deg, see the README's "Einfacher Modus" section).
        """
        return cls(
            origin=np.zeros(3, dtype=float),
            e_col=np.array([1.0, 0.0, 0.0]),
            e_row=np.array([0.0, 1.0, 0.0]),
            boresight_quat=np.array([0.0, 0.0, 0.0, 1.0]),
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

    Raises ``CalibrationAngleWarning`` (not an error) if the raw traces are
    more than ``max_angle_error_deg`` away from perpendicular; the returned
    calibration is still Gram-Schmidt orthogonalised either way.
    """
    origin, e_col_raw = fit_axis(col_samples)
    _, e_row_raw = fit_axis(row_samples)

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

    # Gram-Schmidt: trust e_col as fitted, force e_row perpendicular to it.
    e_col = e_col_raw
    e_row = e_row_raw - np.dot(e_row_raw, e_col) * e_col
    row_norm = float(np.linalg.norm(e_row))
    if row_norm < 1e-9:
        raise ValueError("Row trace is parallel to the column trace; cannot "
                          "build an orthogonal page frame.")
    e_row = e_row / row_norm

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
        angle_error_deg=angle_error_deg)
