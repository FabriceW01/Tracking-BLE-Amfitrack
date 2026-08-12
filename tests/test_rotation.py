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


def _old_rotation_vector_yaw_deg(quat, boresight_quat=IDENTITY_QUAT,
                                 n=np.array([0.0, 0.0, 1.0])) -> float:
    """
    The REMOVED rotation-vector-projection method rotation.py used before
    the swing-twist rewrite (see its module docstring), reimplemented here
    -- not imported, it no longer exists in rotation.py -- purely so this
    test file can demonstrate, on real inline math rather than by assertion,
    exactly how and where it used to go wrong: correct for a pure rotation
    about ``n`` only up to (but not including) the 180-degree singularity,
    where the antisymmetric-part-over-``sin(angle)`` division blows up, and
    sign-flipped (wrapped by 360 deg, not just imprecise) past it, because a
    rotation MATRIX's axis-angle log-map has no memory of "which way
    around" a rotation past 180 degrees went, unlike a quaternion.
    """
    R = quat_to_matrix(quat) @ quat_to_matrix(boresight_quat).T
    cos_angle = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cos_angle)
    if angle < 1e-9:
        return 0.0
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * math.sin(angle))
    return math.degrees(np.dot(axis * angle, n))


def test_yaw_about_normal_combined_tilt_and_yaw_recovers_exactly_where_the_old_method_drifted():
    # 75 deg tilt about a DIAGONAL in-plane axis (not aligned with either
    # e_col or e_row), composed with a 40 deg yaw about the page normal,
    # the yaw applied as the OUTER (page-frame) rotation on top of the tilt
    # -- exactly the physical situation this module cares about (page yaw
    # is a page-frame quantity riding on top of whatever incidental cart
    # tilt already exists, see yaw_about_normal's own docstring for the
    # algebra of why this composition order lets swing-twist recover the
    # injected yaw EXACTLY, unlike the old rotation-vector method, which
    # only ever approximated it and visibly DRIFTS here -- this was tried
    # and measured during the pass5.csv analysis (naive probing gave 143
    # deg vs 76 deg for the same real data).
    tilt_axis = (1.0, 1.0, 0.0)
    tilt_deg = 75.0
    yaw_deg = 40.0
    q_tilt = _quat_about_axis(tilt_axis, tilt_deg)
    q_yaw = _quat_about_z(yaw_deg)
    q_combo = _qmul(q_yaw, q_tilt)          # tilt first, then yaw, world-frame compose

    # Independent re-derivation of the swing-twist formula itself (fresh
    # Hamilton-product + atan2 math, not calling rotation.py's
    # _quat_multiply/_quat_conjugate), so a bug in rotation.py's own
    # implementation would not also be baked into the "expected" value.
    qx, qy, qz, qw = q_combo
    n = np.array([0.0, 0.0, 1.0])
    expected_yaw_deg = math.degrees(2.0 * math.atan2(qx * n[0] + qy * n[1] + qz * n[2], qw))

    got_deg = math.degrees(yaw_about_normal(q_combo, IDENTITY_QUAT, E_COL, E_ROW))
    assert abs(got_deg - expected_yaw_deg) < 1e-9, (got_deg, expected_yaw_deg)
    # Recovers the TRUE injected yaw exactly, not just self-consistently.
    assert abs(got_deg - yaw_deg) < 1e-6, (got_deg, yaw_deg)

    # The OLD (removed) rotation-vector method drifts noticeably here --
    # proof this is a genuine accuracy fix for combined tilt+yaw, not only a
    # singularity patch for the 180-degree case.
    old_deg = _old_rotation_vector_yaw_deg(q_combo)
    assert abs(old_deg - yaw_deg) > 5.0, (
        "the old rotation-vector method was expected to drift noticeably "
        "from the true injected yaw here -- if it now matches, this test's "
        "premise (demonstrating swing-twist's accuracy advantage) no "
        "longer holds and should be revisited")

    R = quat_to_matrix(q_combo)
    naive_row = _naive_projection_yaw_deg(R, E_ROW)
    naive_col = _naive_projection_yaw_deg(R, E_COL)
    # The naive probes disagree with EACH OTHER by tens of degrees --
    # proof they cannot both be right, and in fact neither is close to the
    # correct (probe-independent) value.
    assert abs(naive_row - naive_col) > 30.0, (naive_row, naive_col)
    assert abs(naive_row - got_deg) > 15.0, (naive_row, got_deg)
    assert abs(naive_col - got_deg) > 15.0, (naive_col, got_deg)


# ============================ full sweep: no singularity, no early sign-flip
def test_yaw_about_normal_pure_rotation_recovers_exactly_through_a_full_sweep():
    # Sweeps right across the old method's failure zone (see rotation.py's
    # module docstring): correct only up to 135 deg, garbage (934.2 deg on
    # the operator's real calibration) at 180, sign-flipped by exactly 360
    # deg past it. Swing-twist has no equivalent singularity anywhere in
    # this range -- pin exact recovery at every one of these angles,
    # including right at the old 180-degree blow-up point.
    for deg in (0.0, 45.0, 90.0, 135.0, 179.0, 180.0, 225.0, 270.0, 315.0):
        got = math.degrees(yaw_about_normal(_quat_about_z(deg), IDENTITY_QUAT, E_COL, E_ROW))
        assert abs(got - deg) < 1e-6, (deg, got)


def test_yaw_about_normal_MUTATION_check_old_method_sign_flips_past_180():
    # Inlines the removed rotation-vector method and shows it disagrees
    # with the true injected angle at 225/270 degrees (by exactly 360 deg,
    # i.e. sign-flipped, not merely imprecise) -- proof the 225/270 cases in
    # the full-sweep test above are actually exercising the swing-twist
    # replacement, not coincidentally passing some other way.
    for deg, old_expected in ((225.0, -135.0), (270.0, -90.0)):
        old_deg = _old_rotation_vector_yaw_deg(_quat_about_z(deg))
        assert abs(old_deg - old_expected) < 1e-6, (deg, old_deg)  # matches the module docstring's table
        assert abs(old_deg - deg) > 90.0, (deg, old_deg)           # nowhere near the true angle

        new_deg = math.degrees(yaw_about_normal(_quat_about_z(deg), IDENTITY_QUAT, E_COL, E_ROW))
        assert abs(new_deg - deg) < 1e-6, (deg, new_deg)           # swing-twist: exact, no flip


# ==================================================== defensive normalisation
def test_yaw_about_normal_normalises_non_unit_quat_and_boresight():
    # Real sensor quaternions aren't guaranteed exactly unit-norm -- the
    # operator's own real boresight measures 1.00002. yaw_about_normal must
    # correct for that itself now that it no longer routes through
    # quat_to_matrix (which used to normalise implicitly on its behalf).
    unit_quat = np.array(_quat_about_z(37.0))
    unit_bore = np.array(_quat_about_z(12.0))
    got = yaw_about_normal(unit_quat * 1.00002, unit_bore * 0.99998, E_COL, E_ROW)
    expected = yaw_about_normal(unit_quat, unit_bore, E_COL, E_ROW)
    assert abs(got - expected) < 1e-6, (got, expected)


def test_yaw_about_normal_rejects_a_zero_norm_quat():
    for bad_quat, bad_bore in (((0.0, 0.0, 0.0, 0.0), IDENTITY_QUAT),
                               (IDENTITY_QUAT, (0.0, 0.0, 0.0, 0.0))):
        try:
            yaw_about_normal(bad_quat, bad_bore, E_COL, E_ROW)
        except ValueError:
            continue
        raise AssertionError(f"expected ValueError for {bad_quat=} {bad_bore=}")


# ======================================================= quaternion double cover
def test_yaw_about_normal_double_cover_shifts_the_READOUT_by_exactly_360():
    # Pins the one property the swing-twist rewrite gave up, so it is a
    # known, measured trade rather than a surprise later: q and -q are the
    # SAME physical orientation, and the old matrix-based method was immune
    # by construction (R(q) == R(-q)), but reading the quaternion's
    # components directly is not. The shift is exactly 360 deg at EVERY
    # angle -- not just near a full turn -- which is what makes it
    # recognisable on hardware if it ever happens (a 360 deg jump with the
    # cart standing still). See yaw_about_normal's docstring for the
    # one-line fix, and for why it is deliberately NOT applied pre-emptively.
    for deg in (0.0, 30.0, 90.0, 135.0, 180.0, 225.0, 315.0):
        q = np.array(_quat_about_z(deg))
        plus = math.degrees(yaw_about_normal(q, IDENTITY_QUAT, E_COL, E_ROW))
        minus = math.degrees(yaw_about_normal(-q, IDENTITY_QUAT, E_COL, E_ROW))
        assert abs(abs(plus - minus) - 360.0) < 1e-6, (deg, plus, minus)


def test_yaw_about_normal_double_cover_cannot_affect_the_PRINT_correction():
    # The counter-check that makes the trade above acceptable: everything
    # downstream (tracking.PageMapper.project, coverage.CoverageEngine)
    # consumes only sin/cos of this yaw, both exactly 360-deg periodic, so a
    # sign flip is provably invisible to where the ink actually lands.
    # Verified across a full sweep rather than at a couple of angles.
    for deg in np.arange(0.0, 360.0, 7.0):
        q = np.array(_quat_about_z(float(deg)))
        a = yaw_about_normal(q, IDENTITY_QUAT, E_COL, E_ROW)
        b = yaw_about_normal(-q, IDENTITY_QUAT, E_COL, E_ROW)
        assert abs(math.cos(a) - math.cos(b)) < 1e-9, (deg, a, b)
        assert abs(math.sin(a) - math.sin(b)) < 1e-9, (deg, a, b)


def test_cart_rotation_angles_roll_pitch_are_double_cover_INVARIANT():
    # Roll/pitch are read off the SWING quaternion (twist divided out), so
    # unlike yaw they keep full double-cover invariance -- worth pinning,
    # because it means the live tilt diagnostic stays trustworthy even in
    # the scenario the yaw readout would not.
    for deg in (0.0, 90.0, 180.0, 270.0):
        q = np.array(_qmul(_quat_about_z(deg), _quat_about_axis((1.0, 1.0, 0.0), 5.0)))
        roll_a, pitch_a, _ = cart_rotation_angles(q, IDENTITY_QUAT, E_COL, E_ROW)
        roll_b, pitch_b, _ = cart_rotation_angles(-q, IDENTITY_QUAT, E_COL, E_ROW)
        assert abs(roll_a - roll_b) < 1e-9, (deg, roll_a, roll_b)
        assert abs(pitch_a - pitch_b) < 1e-9, (deg, pitch_a, pitch_b)


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


def test_cart_rotation_angles_roll_pitch_stay_small_when_yaw_exceeds_180():
    # Regression for reading roll/pitch off the SWING quaternion (twist
    # removed) rather than directly off the full, un-factored relative
    # rotation -- see cart_rotation_angles's own docstring for why the
    # latter cross-talks once yaw alone pushes the relative rotation's
    # scalar part negative. A small (10 deg) real tilt combined with a
    # yaw well past 180 (250 deg) must still report a SMALL roll/pitch --
    # not the wildly wrong hundreds-of-degrees values the naive "same
    # formula straight off quat_rel" approach gives for this exact case
    # (reimplemented inline below, not imported, to demonstrate the
    # contrast on real numbers rather than by assertion alone).
    tilt = _quat_about_axis((1.0, 1.0, 0.0), 10.0)
    yaw = _quat_about_z(250.0)
    combo = _qmul(yaw, tilt)

    roll, pitch, yaw_out = cart_rotation_angles(combo, IDENTITY_QUAT, E_COL, E_ROW)
    assert abs(math.degrees(yaw_out) - 250.0) < 1e-6, yaw_out
    assert abs(math.degrees(roll)) < 20.0, math.degrees(roll)      # small tilt -> small roll
    assert abs(math.degrees(pitch)) < 20.0, math.degrees(pitch)    # small tilt -> small pitch

    # The naive (no swing-removal) formula applied directly to quat_rel:
    qx, qy, qz, qw = combo
    naive_roll_deg = math.degrees(2.0 * math.atan2(qx, qw))
    naive_pitch_deg = math.degrees(2.0 * math.atan2(qy, qw))
    assert abs(naive_roll_deg) > 90.0, naive_roll_deg      # hundreds of degrees off
    assert abs(naive_pitch_deg) > 90.0, naive_pitch_deg


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
