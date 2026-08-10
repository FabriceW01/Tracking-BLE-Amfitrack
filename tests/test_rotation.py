"""
Cart-yaw extraction tests (no hardware): printhead/rotation.py's
quaternion -> axis-angle -> "component about the page normal" math, checked
against hand-derived rotations rather than against any live tracker/page
calibration.

Run with:  python tests/test_rotation.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.rotation import (  # noqa: E402
    cart_rotation_angles, quat_to_matrix, yaw_about_normal,
)

IDENTITY_QUAT = (0.0, 0.0, 0.0, 1.0)
E_COL = np.array([1.0, 0.0, 0.0])
E_ROW = np.array([0.0, 1.0, 0.0])          # normal n = e_col x e_row = +Z here


def _quat_about_z(deg: float):
    """Quaternion for a rotation of ``deg`` about +Z -- coincides with the
    page normal for E_COL/E_ROW above, i.e. a pure yaw."""
    half = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _quat_about_axis(axis, deg: float):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    half = math.radians(deg) / 2.0
    s = math.sin(half)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half))


def _qmul(a, b):
    """Hamilton product, (qx, qy, qz, qw) order -- a completely independent
    (from rotation.py) way to compose two rotations, used below to build a
    combined tilt+yaw quaternion for the "naive method is wrong" test."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


# =================================================================== basics
def test_yaw_about_normal_identity_boresight_and_quat_is_zero():
    assert abs(yaw_about_normal(IDENTITY_QUAT, IDENTITY_QUAT, E_COL, E_ROW)) < 1e-9


def test_yaw_about_normal_pure_positive_yaw_recovers_the_angle():
    for deg in (10.0, 45.0, 89.0):
        got = math.degrees(yaw_about_normal(_quat_about_z(deg), IDENTITY_QUAT, E_COL, E_ROW))
        assert abs(got - deg) < 1e-6, (deg, got)


def test_yaw_about_normal_pure_negative_yaw_recovers_the_angle():
    for deg in (10.0, 45.0, 89.0):
        got = math.degrees(yaw_about_normal(_quat_about_z(-deg), IDENTITY_QUAT, E_COL, E_ROW))
        assert abs(got - (-deg)) < 1e-6, (deg, got)


def test_yaw_about_normal_relative_to_a_nonidentity_boresight():
    # The boresight itself need not be identity -- only the RELATIVE
    # rotation (current vs. boresight) should matter. Boresight at +20 deg,
    # current at +50 deg -> 30 deg of yaw since the boresight was captured.
    boresight = _quat_about_z(20.0)
    current = _quat_about_z(50.0)
    got = math.degrees(yaw_about_normal(current, boresight, E_COL, E_ROW))
    assert abs(got - 30.0) < 1e-6


# =================================================== pure tilt isolates to 0
def test_yaw_about_normal_pure_inplane_tilt_is_approximately_zero():
    # A rotation entirely about an axis PERPENDICULAR to the page normal
    # (e.g. about e_col itself) is pure pitch/roll, no yaw component at all
    # -- proves the decomposition isolates the right axis rather than
    # leaking tilt into the yaw number.
    for axis, deg in ((E_COL, 30.0), (E_ROW, -25.0), ((1.0, 1.0, 0.0), 40.0)):
        got = yaw_about_normal(_quat_about_axis(axis, deg), IDENTITY_QUAT, E_COL, E_ROW)
        assert abs(got) < 1e-9, (axis, deg, got)


# ======================================== combined tilt+yaw: naive is wrong
def _naive_projection_yaw_deg(R: np.ndarray, probe_body_axis: np.ndarray,
                              n=np.array([0.0, 0.0, 1.0])) -> float:
    """
    The rejected alternative mentioned in rotation.py's docstring: project a
    fixed CART axis onto the page plane before/after the rotation and
    measure the angle between the two projections. Reimplemented here (not
    imported -- it deliberately does not exist in rotation.py) purely to
    demonstrate why it was rejected: unlike yaw_about_normal, its answer
    depends on which body axis you happen to probe with.
    """
    def project(v):
        p = v - np.dot(v, n) * n
        norm = np.linalg.norm(p)
        return p / norm if norm > 1e-9 else p

    p0 = project(probe_body_axis)
    p1 = project(R @ probe_body_axis)
    cos_ang = float(np.clip(np.dot(p0, p1), -1.0, 1.0))
    ang = math.degrees(math.acos(cos_ang))
    sign = 1.0 if np.dot(np.cross(p0, p1), n) >= 0 else -1.0
    return sign * ang


def test_yaw_about_normal_combined_tilt_and_yaw_where_naive_projection_disagrees_with_itself():
    # 75 deg tilt about a DIAGONAL in-plane axis (not aligned with either
    # e_col or e_row), composed with a 40 deg yaw about the page normal.
    # This is exactly the failure mode described in the pass5.csv analysis
    # (naive probing gave 143 deg vs 76 deg for the same real data): probing
    # the naive projection method with e_row vs. e_col gives wildly
    # different, and both wrong, answers -- while yaw_about_normal (which
    # never probes any particular body axis) gives one single, correct
    # value, independently re-derived below via the same axis-angle math
    # rotation.py itself uses, rather than trusting its own output blindly.
    tilt_axis = (1.0, 1.0, 0.0)
    tilt_deg = 75.0
    yaw_deg = 40.0
    q_tilt = _quat_about_axis(tilt_axis, tilt_deg)
    q_yaw = _quat_about_z(yaw_deg)
    q_combo = _qmul(q_yaw, q_tilt)          # tilt first, then yaw, world-frame compose

    R = quat_to_matrix(q_combo)
    # Independent re-derivation of the expected axis-angle-about-normal
    # value (mirrors _rotation_vector's formula, but written out fresh here
    # instead of calling it, so a bug in rotation.py's own implementation
    # would not also be baked into the "expected" value).
    cos_angle = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cos_angle)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * math.sin(angle))
    expected_yaw_deg = math.degrees(np.dot(axis * angle, np.array([0.0, 0.0, 1.0])))

    got_deg = math.degrees(yaw_about_normal(q_combo, IDENTITY_QUAT, E_COL, E_ROW))
    assert abs(got_deg - expected_yaw_deg) < 1e-6, (got_deg, expected_yaw_deg)

    naive_row = _naive_projection_yaw_deg(R, E_ROW)
    naive_col = _naive_projection_yaw_deg(R, E_COL)
    # The naive probes disagree with EACH OTHER by tens of degrees --
    # proof they cannot both be right, and in fact neither is close to the
    # correct (probe-independent) value.
    assert abs(naive_row - naive_col) > 30.0, (naive_row, naive_col)
    assert abs(naive_row - got_deg) > 15.0, (naive_row, got_deg)
    assert abs(naive_col - got_deg) > 15.0, (naive_col, got_deg)


# ============================================================== quat_to_matrix
def test_quat_to_matrix_identity_is_the_identity_matrix():
    assert np.allclose(quat_to_matrix(IDENTITY_QUAT), np.eye(3))


def test_quat_to_matrix_is_a_proper_rotation():
    R = quat_to_matrix(_quat_about_axis((0.3, -0.7, 0.4), 63.0))
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-9)       # orthogonal
    assert abs(np.linalg.det(R) - 1.0) < 1e-9                # proper (no reflection)


def test_quat_to_matrix_normalises_a_non_unit_quaternion():
    # A raw sensor quaternion is not guaranteed to be exactly unit-norm --
    # quat_to_matrix must correct for that rather than silently returning a
    # scaled (non-rotation) matrix.
    unit = np.array(_quat_about_z(37.0))
    scaled = unit * 3.0
    assert np.allclose(quat_to_matrix(scaled), quat_to_matrix(unit))


def test_quat_to_matrix_rejects_a_zero_quaternion():
    try:
        quat_to_matrix((0.0, 0.0, 0.0, 0.0))
    except ValueError:
        return
    raise AssertionError("expected ValueError for a zero-norm quaternion")


# ============================================================== degenerate frame
def test_yaw_about_normal_rejects_a_degenerate_page_frame():
    try:
        yaw_about_normal(IDENTITY_QUAT, IDENTITY_QUAT, E_COL, E_COL)   # parallel
    except ValueError:
        return
    raise AssertionError("expected ValueError for parallel e_col/e_row")


# =============================================================== cart_rotation_angles
def test_cart_rotation_angles_pure_roll_about_e_col_isolates_to_roll():
    # A rotation purely about e_col (the column/travel axis) is pure "roll"
    # by this module's aircraft-style convention -- must show up entirely as
    # roll, with pitch and yaw both ~0.
    for deg in (10.0, 45.0, -30.0):
        roll, pitch, yaw = cart_rotation_angles(
            _quat_about_axis(E_COL, deg), IDENTITY_QUAT, E_COL, E_ROW)
        assert abs(math.degrees(roll) - deg) < 1e-6, (deg, roll)
        assert abs(pitch) < 1e-9, (deg, pitch)
        assert abs(yaw) < 1e-9, (deg, yaw)


def test_cart_rotation_angles_pure_pitch_about_e_row_isolates_to_pitch():
    # Symmetric case: a rotation purely about e_row (the row axis, along the
    # nozzle bar) is pure "pitch" -- must show up entirely as pitch, with
    # roll and yaw both ~0.
    for deg in (10.0, 45.0, -30.0):
        roll, pitch, yaw = cart_rotation_angles(
            _quat_about_axis(E_ROW, deg), IDENTITY_QUAT, E_COL, E_ROW)
        assert abs(roll) < 1e-9, (deg, roll)
        assert abs(math.degrees(pitch) - deg) < 1e-6, (deg, pitch)
        assert abs(yaw) < 1e-9, (deg, yaw)


def test_cart_rotation_angles_pure_roll_relative_to_a_nonidentity_boresight():
    # Same idea as yaw_about_normal's non-identity-boresight test: only the
    # RELATIVE rotation (current vs. boresight) should matter for roll too.
    boresight = _quat_about_axis(E_COL, 15.0)
    current = _quat_about_axis(E_COL, 55.0)
    roll, pitch, yaw = cart_rotation_angles(current, boresight, E_COL, E_ROW)
    assert abs(math.degrees(roll) - 40.0) < 1e-6
    assert abs(pitch) < 1e-9
    assert abs(yaw) < 1e-9


def test_cart_rotation_angles_yaw_only_matches_yaw_about_normal():
    # Cross-check required by the PR: for a yaw-only rotation, this
    # function's yaw component must be IDENTICAL to yaw_about_normal called
    # with the same inputs -- the two must never disagree.
    for deg in (10.0, 45.0, -60.0, 89.0):
        quat = _quat_about_z(deg)
        roll, pitch, yaw = cart_rotation_angles(quat, IDENTITY_QUAT, E_COL, E_ROW)
        expected_yaw = yaw_about_normal(quat, IDENTITY_QUAT, E_COL, E_ROW)
        assert yaw == expected_yaw, (deg, yaw, expected_yaw)
        assert abs(roll) < 1e-9, (deg, roll)
        assert abs(pitch) < 1e-9, (deg, pitch)


def test_cart_rotation_angles_combined_tilt_and_yaw_yaw_component_still_matches():
    # Same cross-check but with roll/pitch/yaw all simultaneously nonzero
    # (combined tilt+yaw, like the naive-projection-disagreement case above)
    # -- the yaw component must still agree with yaw_about_normal exactly.
    tilt_axis = (1.0, 1.0, 0.0)
    q_tilt = _quat_about_axis(tilt_axis, 35.0)
    q_yaw = _quat_about_z(20.0)
    q_combo = _qmul(q_yaw, q_tilt)
    roll, pitch, yaw = cart_rotation_angles(q_combo, IDENTITY_QUAT, E_COL, E_ROW)
    expected_yaw = yaw_about_normal(q_combo, IDENTITY_QUAT, E_COL, E_ROW)
    assert yaw == expected_yaw, (yaw, expected_yaw)
    # And with genuine combined tilt, roll/pitch are no longer both ~0.
    assert abs(roll) > 1e-6 or abs(pitch) > 1e-6


def test_cart_rotation_angles_identity_boresight_and_quat_is_all_zero():
    roll, pitch, yaw = cart_rotation_angles(IDENTITY_QUAT, IDENTITY_QUAT, E_COL, E_ROW)
    assert abs(roll) < 1e-9 and abs(pitch) < 1e-9 and abs(yaw) < 1e-9


def test_cart_rotation_angles_normalises_non_unit_e_col_e_row():
    # Defensive re-normalisation: scaling e_col/e_row must not change the
    # returned angles (mirrors yaw_about_normal's defensive normalisation of
    # the page normal).
    roll, pitch, yaw = cart_rotation_angles(
        _quat_about_axis(E_COL, 30.0), IDENTITY_QUAT, E_COL * 5.0, E_ROW * 0.2)
    assert abs(math.degrees(roll) - 30.0) < 1e-6
    assert abs(pitch) < 1e-9
    assert abs(yaw) < 1e-9


def test_cart_rotation_angles_rejects_a_degenerate_page_frame():
    try:
        cart_rotation_angles(IDENTITY_QUAT, IDENTITY_QUAT, E_COL, E_COL)   # parallel
    except ValueError:
        return
    raise AssertionError("expected ValueError for parallel e_col/e_row")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All rotation tests passed.")
