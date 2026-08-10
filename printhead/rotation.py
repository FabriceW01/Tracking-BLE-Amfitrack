"""
Cart-yaw extraction from the tracked orientation quaternion
=============================================================

Measured from a real print's logged quaternions (``pass5.csv``, relative-
rotation axis-angle decomposition against the page normal from the user's
own calibration): cart yaw about the page normal spans **75.6 deg** over a
normal freehand pass, while tilt (pitch/roll) is small by comparison
(median 2.7 deg, max 7.8 deg) -- so correcting yaw alone recovers nearly all
of the error, and pitch/roll correction is a deliberate, measured
non-goal (see the module docstrings of ``coverage.py`` / ``tracking.py``
for where the resulting error actually shows up: a 62.36mm sensor->nozzle
lever arm rotating with the cart, up to ~76mm of position error at the
measured yaw span, and the 15mm nozzle bar itself sweeping across columns
as it tilts, ~14.5mm at 75 deg).

Kept as its own module rather than folded into ``calibration.py``: this is
quaternion/rotation-matrix algebra used at PRINT time (every sample, via
``tracking.PageMapper`` and ``controller._print_freehand_pass``), not just
during the one-off page-edge-tracing calibration step that ``calibration.py``
is about -- and it is independently testable (see ``tests/test_rotation.py``)
without dragging in ``PageCalibration``/``fit_axis``/Gram-Schmidt at all.
"""

from __future__ import annotations

import math

import numpy as np


def quat_to_matrix(quat) -> np.ndarray:
    """
    Unit quaternion ``(qx, qy, qz, qw)`` (this project's component order --
    see ``tracking.AmfitrackTracker._extract_pose``) -> the 3x3 rotation
    matrix ``R`` such that ``R @ v_body`` expresses a body-frame vector in
    world coordinates.

    Normalises defensively: a raw sensor quaternion is not guaranteed to be
    exactly unit-norm, and the standard formula below silently returns a
    non-rotation (scaled) matrix if fed one that isn't.
    """
    q = np.asarray(quat, dtype=float)
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        raise ValueError("quat_to_matrix: zero-norm quaternion")
    x, y, z, w = q / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),       2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w),       2.0 * (y * z + x * w),       1.0 - 2.0 * (x * x + y * y)],
    ])


def _rotation_vector(R: np.ndarray) -> np.ndarray:
    """
    Rotation matrix -> its axis-angle "rotation vector" (axis * angle,
    radians, right-hand rule). Zero for the identity rotation (axis is
    undefined there, so the zero vector is the only sane answer).

    Standard closed-form log-map of SO(3), valid for angle in [0, pi). The
    antisymmetric part this divides by vanishes again as angle -> pi (an
    exact 180-degree rotation) -- not specially handled here: the measured
    cart yaw/tilt this module exists for tops out at 75.6 deg (see the
    module docstring), nowhere near that edge case, so adding a second
    branch for it would be untested complexity with no real caller.
    """
    cos_angle = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cos_angle)
    if angle < 1e-9:
        return np.zeros(3)
    sin_angle = math.sin(angle)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * sin_angle)
    return axis * angle


def yaw_about_normal(quat, boresight_quat, e_col, e_row) -> float:
    """
    Signed yaw (radians) of the cart about the page normal, relative to the
    boresight pose -- the orientation captured with the nozzle bar aligned
    along the traced row edge (see ``calibration.PageCalibration.
    boresight_quat`` / ``calibration.calibrate_page``'s ``boresight_quat``
    parameter).

    Method: build ``R_rel = R(quat) @ R(boresight_quat)^T`` -- the rotation
    that takes the cart from its boresight orientation to its current one,
    expressed in world/page coordinates -- convert that to its axis-angle
    rotation vector (see ``_rotation_vector``), and return the component of
    that vector along the page normal ``n = normalise(e_col x e_row)``.

    This is deliberately NOT "project some fixed cart axis onto the page
    plane and measure the angle between its boresight and current
    projections" -- that naive approach was tried first during the
    pass5.csv analysis and gave INCONSISTENT answers (143 deg vs 76 deg,
    same data) depending on which body axis was probed, because projecting
    a tilted axis onto a plane distorts angles whenever any pitch/roll is
    present alongside the yaw -- exactly the case here (see the module
    docstring: tilt is small but non-zero, median 2.7 deg). The axis-angle
    decomposition instead operates on the rotation itself, not on any
    arbitrarily chosen probe vector, so it gives one answer regardless of
    how much tilt rides along with the yaw. Do not reach for the naive
    projection version even though it looks simpler -- it was already tried
    and found to be the wrong tool for this data.

    Sign convention: matches ``tracking.PageMapper``'s offset-rotation
    formula directly (``u += col*cos(yaw) - row*sin(yaw)``,
    ``v += col*sin(yaw) + row*cos(yaw)``) with no extra negation needed --
    pinned by ``tests/test_rotation.py``'s 90-degree case together with
    ``tests/test_page_mapper.py``'s matching 90-degree offset case.
    """
    e_col = np.asarray(e_col, dtype=float)
    e_row = np.asarray(e_row, dtype=float)
    n = np.cross(e_col, e_row)
    n_norm = float(np.linalg.norm(n))
    if n_norm < 1e-9:
        raise ValueError("yaw_about_normal: e_col and e_row are parallel "
                         "(degenerate page frame)")
    n = n / n_norm

    r_rel = quat_to_matrix(quat) @ quat_to_matrix(boresight_quat).T
    return float(np.dot(_rotation_vector(r_rel), n))
