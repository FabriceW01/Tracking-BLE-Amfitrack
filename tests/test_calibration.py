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
    CalibrationAngleWarning, CalibrationQualityWarning, PageCalibration,
    calibrate_page, fit_axis, fit_axis_quality, trace_length_mm,
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


# ============================================================= fit_axis_quality
def test_fit_axis_quality_reports_length_count_and_near_zero_rms_when_clean():
    direction = np.array([1.0, 0.0, 0.0])
    samples = _noisy_line(np.zeros(3), direction, 210.0, n=50, noise_mm=0.0)
    _, fitted_dir = fit_axis(samples)
    q = fit_axis_quality(samples, fitted_dir)
    assert abs(q.length_mm - 210.0) < 1e-6
    assert q.sample_count == 50
    assert q.rms_residual_mm < 1e-9                    # noise-free -> perfectly on the line


def test_fit_axis_quality_rms_residual_reflects_injected_noise():
    # Noise is injected PERPENDICULAR to the fitted direction here (the
    # direction itself is along x, noise along y/z only), so the RMS
    # residual should land close to the injected noise std, not near zero.
    direction = np.array([1.0, 0.0, 0.0])
    rng = np.random.default_rng(0)
    t = np.linspace(0.0, 200.0, 100)
    samples = np.zeros((100, 3))
    samples[:, 0] = t
    samples[:, 1] = rng.normal(0.0, 0.8, 100)
    samples[:, 2] = rng.normal(0.0, 0.8, 100)
    _, fitted_dir = fit_axis(samples)
    q = fit_axis_quality(samples, fitted_dir)
    # RMS of two independent N(0, 0.8) components combined -> sqrt(2)*0.8 ~= 1.13.
    # Tight band deliberately: the original +-0.45mm band here was wide
    # enough to pass with the first version of fit_axis_quality, which
    # measured residuals from samples[0] instead of the fitted line and so
    # over-reported by a measured 1.33x (1.47 here, still inside that band).
    # A metric that feeds a threshold has to be pinned tightly enough to
    # catch a systematic scale error, not just "roughly the right size".
    assert abs(q.rms_residual_mm - math.sqrt(2.0) * 0.8) < 0.1, q.rms_residual_mm


def test_fit_axis_quality_rms_residual_is_independent_of_sample_ORDER():
    # REGRESSION: measuring residuals from samples[0] made this metric
    # depend on which sample happened to arrive first. On one fixed 60-point
    # trace, merely rotating the sample order moved the reported RMS between
    # 0.49 and 1.04mm -- straddling MAX_RMS_RESIDUAL_MM (1.0), so the same
    # physical trace passed or failed the quality check purely on ordering.
    # Measuring from the centroid (the point fit_axis's PCA line actually
    # passes through) is order-invariant by construction; this pins that.
    rng = np.random.default_rng(7)
    n = 60
    samples = np.zeros((n, 3))
    samples[:, 0] = np.linspace(0.0, 200.0, n)
    samples[:, 1] = rng.normal(0.0, 0.4, n)
    samples[:, 2] = rng.normal(0.0, 0.4, n)
    _, direction = fit_axis(samples)

    values = [fit_axis_quality(np.roll(samples, -k, axis=0), direction).rms_residual_mm
              for k in range(n)]
    assert max(values) - min(values) < 1e-9, (min(values), max(values))


def test_fit_axis_quality_rms_residual_is_not_dominated_by_one_outlier():
    # REGRESSION, same root cause as the ordering test above: with residuals
    # measured from samples[0], a single outlier landing FIRST reported
    # 4.97mm RMS on a trace whose true scatter was 0.81mm -- a 6x false
    # alarm that would have told the operator to re-trace a perfectly good
    # edge. From the centroid, one outlier among 60 samples can only move
    # the RMS a little.
    rng = np.random.default_rng(7)
    n = 60
    samples = np.zeros((n, 3))
    samples[:, 0] = np.linspace(0.0, 200.0, n)
    samples[:, 1] = rng.normal(0.0, 0.4, n)
    samples[:, 2] = rng.normal(0.0, 0.4, n)
    clean_rms = fit_axis_quality(samples, fit_axis(samples)[1]).rms_residual_mm

    samples[0, 1] += 5.0                      # one gross outlier, placed FIRST
    _, direction = fit_axis(samples)
    outlier_rms = fit_axis_quality(samples, direction).rms_residual_mm
    assert outlier_rms < 2.0 * clean_rms, (clean_rms, outlier_rms)


def test_fit_axis_quality_does_not_change_with_more_noise_free_samples():
    # Sample count is reported, not baked into length/rms: doubling the
    # sample density along the same clean line should leave length/rms
    # essentially unchanged, only sample_count should differ.
    direction = np.array([0.0, 1.0, 0.0])
    sparse = _noisy_line(np.zeros(3), direction, 100.0, n=10, noise_mm=0.0)
    dense = _noisy_line(np.zeros(3), direction, 100.0, n=100, noise_mm=0.0)
    q_sparse = fit_axis_quality(sparse, direction)
    q_dense = fit_axis_quality(dense, direction)
    assert abs(q_sparse.length_mm - q_dense.length_mm) < 1e-6
    assert q_sparse.sample_count == 10
    assert q_dense.sample_count == 100


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


# ================================================== fit-quality metrics + warnings
def test_calibrate_page_populates_quality_metrics():
    col_samples, row_samples = _page_traces()          # 210mm x 297mm, 40 samples each
    with warnings.catch_warnings():
        warnings.simplefilter("error", CalibrationQualityWarning)
        cal = calibrate_page(col_samples, row_samples)  # must not warn -- good trace
    assert abs(cal.col_trace_length_mm - 210.0) < 1.0
    assert abs(cal.row_trace_length_mm - 297.0) < 1.0
    assert cal.col_sample_count == 40
    assert cal.row_sample_count == 40
    assert cal.col_rms_residual_mm < 0.1               # noise_mm=0.02 default
    assert cal.row_rms_residual_mm < 0.1
    # e_col~=(1,0,0), e_row~=(0,1,0) -> normal is close to the tracker's own
    # z axis (not bit-exact: noise_mm=0.02 default noise nudges the fit).
    assert abs(cal.normal_tilt_deg) < 0.1, cal.normal_tilt_deg


def test_calibrate_page_warns_on_short_trace():
    # 30mm column edge -- under the 50mm MIN_TRACE_LENGTH_MM threshold.
    col_samples, row_samples = _page_traces(width_mm=30.0, noise_mm=0.02)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        cal = calibrate_page(col_samples, row_samples)
    assert cal.col_trace_length_mm < 50.0
    quality_warnings = [w for w in caught if issubclass(w.category, CalibrationQualityWarning)]
    assert quality_warnings, caught
    assert "column" in str(quality_warnings[0].message)
    assert "mm long" in str(quality_warnings[0].message)


def test_calibrate_page_warns_on_noisy_trace():
    # 2mm RMS-scale noise on the row edge -- well above the 1mm MAX_RMS_RESIDUAL_MM.
    col_samples, row_samples = _page_traces(noise_mm=0.02)
    _, noisy_row_samples = _page_traces(noise_mm=2.0)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        cal = calibrate_page(col_samples, noisy_row_samples)
    assert cal.row_rms_residual_mm > 1.0
    quality_warnings = [w for w in caught if issubclass(w.category, CalibrationQualityWarning)]
    assert quality_warnings, caught
    assert "row" in str(quality_warnings[0].message)
    assert "RMS residual" in str(quality_warnings[0].message)


def test_calibrate_page_warns_on_few_samples():
    # Only 8 samples per edge -- under the 20-sample MIN_SAMPLE_COUNT.
    col_samples = _noisy_line(np.zeros(3), np.array([1.0, 0.0, 0.0]), 210.0, n=8, noise_mm=0.02)
    row_samples = _noisy_line(np.zeros(3), np.array([0.0, 1.0, 0.0]), 297.0, n=8, noise_mm=0.02, seed=1)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        cal = calibrate_page(col_samples, row_samples)
    assert cal.col_sample_count == 8 and cal.row_sample_count == 8
    quality_warnings = [w for w in caught if issubclass(w.category, CalibrationQualityWarning)]
    assert quality_warnings, caught
    assert "samples" in str(quality_warnings[0].message)


def test_calibrate_page_normal_tilt_reflects_a_tilted_page():
    # Tilt the ROW edge 20 deg out of the xy plane (nonzero z component) --
    # the fitted page normal should tilt away from tracker z by roughly the
    # same amount (not exactly: e_col stays in-plane, only e_row tilts, and
    # Gram-Schmidt only forces perpendicularity, not planarity).
    tilt = np.radians(20.0)
    row_dir = (0.0, np.cos(tilt), np.sin(tilt))
    col_samples, row_samples = _page_traces(row_dir=row_dir, noise_mm=0.0)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", CalibrationAngleWarning)
        cal = calibrate_page(col_samples, row_samples)
    assert cal.normal_tilt_deg > 5.0, cal.normal_tilt_deg


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


def test_save_and_load_roundtrip_includes_quality_metrics():
    col_samples, row_samples = _page_traces()
    cal = calibrate_page(col_samples, row_samples)
    assert cal.col_trace_length_mm is not None          # calibrate_page always fills these

    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "page_calibration.json")
        cal.save(path)
        loaded = PageCalibration.load(path)

    assert abs(loaded.col_trace_length_mm - cal.col_trace_length_mm) < 1e-9
    assert abs(loaded.row_trace_length_mm - cal.row_trace_length_mm) < 1e-9
    assert abs(loaded.col_rms_residual_mm - cal.col_rms_residual_mm) < 1e-9
    assert abs(loaded.row_rms_residual_mm - cal.row_rms_residual_mm) < 1e-9
    assert loaded.col_sample_count == cal.col_sample_count
    assert loaded.row_sample_count == cal.row_sample_count
    assert abs(loaded.normal_tilt_deg - cal.normal_tilt_deg) < 1e-9


def test_quality_metrics_default_to_none_when_built_directly():
    # A calibration built directly (not via calibrate_page) -- e.g. every
    # PageCalibration(...) call elsewhere in this file -- has no measured
    # quality. None, not a fabricated 0.0, is what must come back: 0.0 would
    # read as "measured, and perfect", which is simply false here.
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    assert cal.col_trace_length_mm is None
    assert cal.row_trace_length_mm is None
    assert cal.col_rms_residual_mm is None
    assert cal.row_rms_residual_mm is None
    assert cal.col_sample_count is None
    assert cal.row_sample_count is None
    assert cal.normal_tilt_deg is None


def test_to_dict_omits_absent_quality_fields():
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    d = cal.to_dict()
    for key in ("col_trace_length_mm", "row_trace_length_mm",
               "col_rms_residual_mm", "row_rms_residual_mm",
               "col_sample_count", "row_sample_count", "normal_tilt_deg"):
        assert key not in d, key


def test_from_dict_loads_a_pre_feature_json_with_no_quality_fields():
    # REGRESSION: the operator has a real saved page_calibration.json from
    # before this feature existed -- from_dict must load it fine, with the
    # quality fields defaulting to None rather than raising a KeyError or
    # fabricating numbers. Built as a plain dict here (not via to_dict) to
    # simulate exactly that older file on disk.
    old_style = {
        "origin": [0.0, 0.0, 0.0],
        "e_col": [1.0, 0.0, 0.0],
        "e_row": [0.0, 1.0, 0.0],
        "scale_col": 1.0,
        "scale_row": 1.0,
        "angle_error_deg": 0.3,
    }
    cal = PageCalibration.from_dict(old_style)
    assert cal.col_trace_length_mm is None
    assert cal.normal_tilt_deg is None
    assert abs(cal.angle_error_deg - 0.3) < 1e-9        # unaffected fields still load


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
