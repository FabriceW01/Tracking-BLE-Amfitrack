"""
Page-plane calibration tests (no hardware): synthetic (x, y, z) mm traces
standing in for a cart tracing two page edges.

Run with:  python tests/test_calibration.py
"""

import math
import os
import sys
import tempfile
import warnings

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.calibration import (                                  # noqa: E402
    CalibrationAngleWarning, PageCalibration, calibrate_page, fit_axis,
    trace_length_mm,
)
from printhead.rotation import cart_rotation_angles, yaw_about_normal  # noqa: E402


def _noisy_line(origin, direction, length_mm, n=40, noise_mm=0.05, seed=0):
    """n samples from origin to origin + length_mm*direction, plus small
    isotropic noise -- stands in for a hand-traced edge."""
    rng = np.random.default_rng(seed)
    t = np.linspace(0.0, length_mm, n)
    pts = origin + np.outer(t, direction)
    pts = pts + rng.normal(0.0, noise_mm, pts.shape)
    return pts


# =================================================================== fit_axis
def test_fit_axis_recovers_a_known_direction():
    true_dir = np.array([0.6, 0.8, 0.0])            # already unit-norm (3-4-5)
    samples = _noisy_line(np.array([10.0, 20.0, 0.0]), true_dir, 200.0)
    origin, direction = fit_axis(samples)
    assert np.allclose(origin, samples[0])
    assert np.dot(direction, true_dir) > 0.999       # same direction, tiny noise
    assert abs(np.linalg.norm(direction) - 1.0) < 1e-9


def test_fit_axis_orientation_follows_first_to_last():
    true_dir = np.array([1.0, 0.0, 0.0])
    forward = _noisy_line(np.zeros(3), true_dir, 100.0, seed=1)
    backward = forward[::-1]
    _, dir_fwd = fit_axis(forward)
    _, dir_bwd = fit_axis(backward)
    assert np.dot(dir_fwd, true_dir) > 0.99
    assert np.dot(dir_bwd, true_dir) < -0.99          # reversed trace -> reversed axis


def test_fit_axis_rejects_too_few_or_malformed_samples():
    for bad in (np.zeros((1, 3)), np.zeros((5, 2)), np.zeros((0, 3))):
        try:
            fit_axis(bad)
        except ValueError:
            continue
        raise AssertionError(f"expected ValueError for shape {bad.shape}")


def test_fit_axis_rejects_a_degenerate_point_cloud():
    coincident = np.tile([5.0, 5.0, 5.0], (10, 1))    # every sample identical
    try:
        fit_axis(coincident)
    except ValueError:
        return
    raise AssertionError("expected ValueError for all-coincident samples")


def test_trace_length_mm_matches_known_span():
    direction = np.array([0.0, 1.0, 0.0])
    samples = _noisy_line(np.array([0.0, 0.0, 0.0]), direction, 210.0, noise_mm=0.01)
    length = trace_length_mm(samples, direction)
    assert abs(length - 210.0) < 1.0                  # noise is tiny relative to span


# ============================================================== calibrate_page
def _page_traces(width_mm=210.0, height_mm=297.0, corner=(0.0, 0.0, 0.0),
                  col_dir=(1.0, 0.0, 0.0), row_dir=(0.0, 1.0, 0.0),
                  noise_mm=0.02, seed=0):
    corner = np.asarray(corner, dtype=float)
    col_dir = np.asarray(col_dir, dtype=float) / np.linalg.norm(col_dir)
    row_dir = np.asarray(row_dir, dtype=float) / np.linalg.norm(row_dir)
    col_samples = _noisy_line(corner, col_dir, width_mm, noise_mm=noise_mm, seed=seed)
    row_samples = _noisy_line(corner, row_dir, height_mm, noise_mm=noise_mm, seed=seed + 1)
    return col_samples, row_samples


def test_calibrate_page_orthogonalizes_perpendicular_traces():
    col_samples, row_samples = _page_traces()
    with warnings.catch_warnings():
        warnings.simplefilter("error", CalibrationAngleWarning)
        cal = calibrate_page(col_samples, row_samples)      # must not warn
    assert abs(np.dot(cal.e_col, cal.e_row)) < 1e-9
    assert abs(np.linalg.norm(cal.e_col) - 1.0) < 1e-9
    assert abs(np.linalg.norm(cal.e_row) - 1.0) < 1e-9


def test_calibrate_page_warns_on_skewed_edges():
    # row edge tilted 30 deg off perpendicular from the column edge (rotated
    # *towards* col_dir=(1,0,0), within the col/row plane, so the angle
    # between the two axes actually changes) -> 30 deg angle error, above
    # the default 15 deg threshold.
    tilt = np.radians(30.0)
    skewed_row_dir = (np.sin(tilt), np.cos(tilt), 0.0)
    col_samples, row_samples = _page_traces(row_dir=skewed_row_dir)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        cal = calibrate_page(col_samples, row_samples)
    assert any(issubclass(w.category, CalibrationAngleWarning) for w in caught)
    # still orthogonalised despite the warning
    assert abs(np.dot(cal.e_col, cal.e_row)) < 1e-9


def test_calibrate_page_does_not_warn_on_near_perpendicular_edges():
    # ~3 deg off -- within the default 15 deg tolerance.
    tilt = np.radians(3.0)
    row_dir = (np.sin(tilt), np.cos(tilt), 0.0)
    col_samples, row_samples = _page_traces(row_dir=row_dir)
    with warnings.catch_warnings():
        warnings.simplefilter("error", CalibrationAngleWarning)
        calibrate_page(col_samples, row_samples)   # would raise if it warned


def test_calibrate_page_scale_from_known_sheet_size():
    # Column trace's *raw* length is 200mm, but the real sheet edge is 210mm
    # (e.g. a slightly mis-scaled tracker) -- scale_col should correct for it.
    col_samples, row_samples = _page_traces(width_mm=200.0, height_mm=297.0,
                                             noise_mm=0.0)
    cal = calibrate_page(col_samples, row_samples,
                          sheet_width_mm=210.0, sheet_height_mm=297.0)
    assert abs(cal.scale_col - 210.0 / 200.0) < 1e-6
    assert abs(cal.scale_row - 1.0) < 1e-6
    u, v, _ = cal.project(col_samples[-1])
    assert abs(u - 210.0) < 0.5                       # corrected, not raw 200mm


def test_calibrate_page_defaults_to_unit_scale_without_sheet_size():
    col_samples, row_samples = _page_traces()
    cal = calibrate_page(col_samples, row_samples)
    assert cal.scale_col == 1.0
    assert cal.scale_row == 1.0


def test_calibrate_page_defaults_boresight_quat_to_none():
    # No procedure existed to capture it before this feature -- every saved
    # calibration up to now, and any calibrate_page() call that doesn't pass
    # boresight_quat explicitly, must keep producing one with no boresight,
    # so rotation correction stays off rather than silently guessing a
    # reference pose (see tracking.PageMapper).
    col_samples, row_samples = _page_traces()
    cal = calibrate_page(col_samples, row_samples)
    assert cal.boresight_quat is None


def test_calibrate_page_accepts_and_stores_a_boresight_quat():
    col_samples, row_samples = _page_traces()
    quat = np.array([0.0, 0.0, 0.1305, 0.9914])       # ~15 deg about Z, arbitrary
    cal = calibrate_page(col_samples, row_samples, boresight_quat=quat)
    assert np.allclose(cal.boresight_quat, quat)


def test_calibrate_page_boresight_quat_survives_a_save_load_roundtrip():
    col_samples, row_samples = _page_traces()
    quat = [0.0, 0.0, 0.1305, 0.9914]                 # plain list, not ndarray
    cal = calibrate_page(col_samples, row_samples, boresight_quat=quat)
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "cal.json")
        cal.save(path)
        loaded = PageCalibration.load(path)
    assert np.allclose(loaded.boresight_quat, quat)


def test_calibrate_page_rejects_parallel_traces():
    col_samples, row_samples = _page_traces(row_dir=(1.0, 0.0, 0.0), noise_mm=0.0)
    with warnings.catch_warnings():
        # parallel traces are also 90 deg away from perpendicular, so this
        # would otherwise also raise CalibrationAngleWarning -- expected and
        # secondary to the ValueError this test actually checks for.
        warnings.simplefilter("ignore", CalibrationAngleWarning)
        try:
            calibrate_page(col_samples, row_samples)
        except ValueError:
            return
    raise AssertionError("expected ValueError for parallel col/row traces")


# ==================================================================== project
def test_project_returns_origin_at_zero():
    col_samples, row_samples = _page_traces(noise_mm=0.0)
    cal = calibrate_page(col_samples, row_samples)
    u, v, z = cal.project(cal.origin)
    assert abs(u) < 1e-9 and abs(v) < 1e-9 and abs(z) < 1e-9


def test_project_recovers_a_known_corner_noise_free():
    # Perfectly perpendicular, noise-free traces -> project() should recover
    # the true (u, v) of a point built directly from e_col/e_row.
    col_samples, row_samples = _page_traces(noise_mm=0.0)
    cal = calibrate_page(col_samples, row_samples)
    true_u, true_v = 123.4, 56.7
    point = cal.origin + true_u * cal.e_col + true_v * cal.e_row
    u, v, z = cal.project(point)
    assert abs(u - true_u) < 1e-6
    assert abs(v - true_v) < 1e-6
    assert abs(z) < 1e-6


def test_project_reports_nonzero_z_off_the_page_plane():
    col_samples, row_samples = _page_traces(noise_mm=0.0)
    cal = calibrate_page(col_samples, row_samples)
    normal = np.cross(cal.e_col, cal.e_row)
    lifted = cal.origin + 15.0 * normal              # 15mm standoff
    _, _, z = cal.project(lifted)
    assert abs(z - 15.0) < 1e-6


# ============================================================== save / load
def test_save_and_load_roundtrip():
    col_samples, row_samples = _page_traces()
    cal = calibrate_page(col_samples, row_samples,
                          sheet_width_mm=210.0, sheet_height_mm=297.0)
    cal.boresight_quat = np.array([0.1, -0.2, 0.3, 0.9])

    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "page_calibration.json")
        cal.save(path)
        loaded = PageCalibration.load(path)

    assert np.allclose(loaded.origin, cal.origin)
    assert np.allclose(loaded.e_col, cal.e_col)
    assert np.allclose(loaded.e_row, cal.e_row)
    assert abs(loaded.scale_col - cal.scale_col) < 1e-12
    assert abs(loaded.scale_row - cal.scale_row) < 1e-12
    assert np.allclose(loaded.boresight_quat, cal.boresight_quat)
    assert abs(loaded.angle_error_deg - cal.angle_error_deg) < 1e-9


def test_save_and_load_roundtrip_without_boresight_quat():
    col_samples, row_samples = _page_traces()
    cal = calibrate_page(col_samples, row_samples)
    assert cal.boresight_quat is None

    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "page_calibration.json")
        cal.save(path)
        loaded = PageCalibration.load(path)

    assert loaded.boresight_quat is None


# ===================================================== simple (uncalibrated)
def _quat_about(axis: str, deg: float) -> np.ndarray:
    h = math.radians(deg) / 2.0
    s, c = math.sin(h), math.cos(h)
    return {"x": np.array([s, 0.0, 0.0, c]),
            "y": np.array([0.0, s, 0.0, c]),
            "z": np.array([0.0, 0.0, s, c])}[axis]


def _quat_mul(a, b):
    """Hamilton product in (qx, qy, qz, qw) order -- `a` applied after `b`."""
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                     w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def test_simple_frame_projects_tracker_axes_unchanged():
    # The whole promise of --page-frame simple: u is the tracker's x, v its
    # y, z its z -- no rotation, no scaling, no traced calibration.
    cal = PageCalibration.simple_frame()
    assert np.allclose(cal.e_col, [1, 0, 0])
    assert np.allclose(cal.e_row, [0, 1, 0])
    assert cal.scale_col == 1.0 and cal.scale_row == 1.0
    for pos in ([0, 0, 0], [123.0, 45.0, 7.5], [-10.0, 3.25, -2.0]):
        assert np.allclose(cal.project(pos), pos), pos


def test_simple_frame_starts_without_a_boresight():
    # Must be None, NOT identity. Identity means "the reference pose is the
    # world frame", but the sensor is mounted rotated on the cart (measured:
    # 120 deg), so identity leaves that whole mounting rotation inside every
    # reported angle. The reference is captured at START instead --
    # PageMapper.capture_boresight.
    cal = PageCalibration.simple_frame()
    assert cal.boresight_quat is None


def test_simple_frame_yaw_is_the_turn_from_the_captured_reference():
    # REGRESSION (real-rig numbers): with the measured 120 deg mounting pose
    # as the reference, turning the cart FLAT about tracker z must report
    # exactly that turn. The earlier identity-boresight version reported a
    # 90 deg turn as ~70 deg of yaw change, non-linearly -- and since yaw
    # also drives the sensor->nozzle offset rotation, it misplaced ink too.
    q_mount = np.array([0.479, 0.510, -0.511, 0.499])
    q_mount = q_mount / np.linalg.norm(q_mount)
    cal = PageCalibration.simple_frame()
    for deg in (-30.0, 0.0, 15.0, 45.0, 90.0):
        q_now = _quat_mul(_quat_about("z", deg), q_mount)   # flat turn
        yaw = yaw_about_normal(q_now, q_mount, cal.e_col, cal.e_row)
        assert abs(math.degrees(yaw) - deg) < 1e-6, (deg, math.degrees(yaw))


def test_simple_frame_identity_boresight_would_be_wrong():
    # Pins WHY the identity shortcut was abandoned, so it cannot quietly come
    # back -- against the real mounting pose it still gets things wrong,
    # just not the SAME thing it used to (see the note below, added when
    # rotation.py moved to a swing-twist decomposition).
    q_mount = np.array([0.479, 0.510, -0.511, 0.499])
    q_mount = q_mount / np.linalg.norm(q_mount)
    cal = PageCalibration.simple_frame()
    identity = np.array([0.0, 0.0, 0.0, 1.0])

    # NOTE (post swing-twist): this test used to compare the DIFFERENCE
    # between two flat-turn readings (turn by 0 deg, then by 90) and show
    # identity boresight got that difference wrong by tens of degrees, with
    # the old rotation-vector method. Swing-twist changed that specific
    # symptom: composing an ADDITIONAL world-frame twist about n onto ANY
    # starting orientation adds exactly that twist to the readout (see
    # rotation.yaw_about_normal's own docstring for the algebra), so the
    # DIFFERENCE between two such readings is now invariant to whichever
    # boresight is used, INCLUDING identity -- a genuine, if narrow,
    # positive side effect of the fix (see tests/test_rotation.py's
    # combined-tilt-and-yaw test for the general form of this property).
    at_90 = yaw_about_normal(_quat_mul(_quat_about("z", 90.0), q_mount),
                             identity, cal.e_col, cal.e_row)
    at_0 = yaw_about_normal(q_mount, identity, cal.e_col, cal.e_row)
    assert abs(abs(math.degrees(at_90 - at_0)) - 90.0) < 1e-6, (
        "if this no longer holds, the swing-twist invariance described "
        "above changed -- revisit this comment")

    # What identity boresight still gets wrong: the ABSOLUTE yaw at the
    # reference pose itself. A correctly-boresighted flat cart reads 0 deg
    # yaw at the pose it was boresighted from (see
    # test_simple_frame_yaw_is_the_turn_from_the_captured_reference); with
    # identity boresight, that same physically-flat pose reads far from 0
    # -- exactly the practical problem simple_frame's docstring describes
    # (a pass assumes ~0 tilt/yaw at its captured reference).
    assert abs(math.degrees(at_0)) > 15.0, (
        f"identity boresight unexpectedly reports the reference pose as "
        f"near-zero yaw ({math.degrees(at_0):.1f} deg) -- if this now "
        f"holds, revisit this comment too")

    # And roll/pitch (the live tilt diagnostic) are wrong even more
    # dramatically: with no boresight to subtract the ~120 deg real mounting
    # tilt, that whole tilt shows up as "roll"/"pitch" on every sample,
    # rather than the near-zero reading a genuinely flat cart should give.
    roll, pitch, _ = cart_rotation_angles(q_mount, identity, cal.e_col, cal.e_row)
    assert abs(math.degrees(roll)) > 45.0 or abs(math.degrees(pitch)) > 45.0, (
        (math.degrees(roll), math.degrees(pitch)))


def test_simple_frame_accepts_a_pinned_boresight():
    q = [-0.5, -0.5, -0.51, 0.49]
    cal = PageCalibration.simple_frame(boresight_quat=q)
    assert np.allclose(cal.boresight_quat, q)


def test_simple_frame_pinned_boresight_makes_its_own_pose_read_as_zero():
    # The whole point of --simple-boresight: with the pinned quat used as
    # both the current sample AND the reference, yaw/roll/pitch must be
    # exactly zero -- this is what "capture, verify, pin" checks before
    # trusting a value.
    q = np.array([-0.5, -0.5, -0.51, 0.49])
    cal = PageCalibration.simple_frame(boresight_quat=q)
    yaw = yaw_about_normal(q, cal.boresight_quat, cal.e_col, cal.e_row)
    assert abs(math.degrees(yaw)) < 1e-9


def test_simple_frame_is_independent_between_calls():
    # Returned fresh each call: the controller mutates .origin at pass start
    # (PageMapper.zero_at_nozzle), which must not leak into the next pass or
    # into any other caller via a shared array.
    a, b = PageCalibration.simple_frame(), PageCalibration.simple_frame()
    a.origin += 5.0
    assert np.allclose(b.origin, [0.0, 0.0, 0.0])
    assert np.allclose(a.e_col, b.e_col)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All calibration tests passed.")
