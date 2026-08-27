"""
Straightness / tracking-precision analysis tests (no hardware).

Covers printhead/straightness.py, the offline analysis of a --mode page
--profile-csv pass run along a ruler.

The valuable tests here are the ones with a KNOWN analytic answer -- a
synthetic path whose deviation is constructed, so the reported number can be
checked against arithmetic rather than against whatever the code happens to
produce. Three of those in particular:

  * a perfectly straight diagonal must report ~0 deviation at ANY angle
    (the whole reason the fit is total-least-squares and not OLS),
  * a straight vertical line must work at all (OLS would divide by zero),
  * a constructed sine/parabola bow must come back with the bow amplitude
    that was put in.

Run with:  python tests/test_straightness.py
"""

import math
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead import straightness as S                                # noqa: E402
from printhead.geometry import (                                       # noqa: E402
    NOZZLE_PITCH_MM, SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM,
)


# ============================================================ line fitting
def test_fit_recovers_a_horizontal_line():
    u = np.linspace(0, 100, 50)
    v = np.full_like(u, 7.0)
    fit = S.fit_line_tls(u, v)
    assert fit is not None
    assert abs(fit["angle_deg"]) < 1e-6, fit["angle_deg"]
    _, perp = S.project_to_line(u, v, fit)
    assert np.max(np.abs(perp)) < 1e-9


def test_fit_recovers_a_45_degree_line_with_zero_deviation():
    # The OLS-vs-TLS case: a diagonal is still a perfect line, and the
    # perpendicular deviation must be ~0 regardless of the angle.
    t = np.linspace(0, 100, 60)
    u, v = t, t
    fit = S.fit_line_tls(u, v)
    assert abs(fit["angle_deg"] - 45.0) < 1e-6, fit["angle_deg"]
    _, perp = S.project_to_line(u, v, fit)
    assert np.max(np.abs(perp)) < 1e-9


def test_fit_handles_a_vertical_line_that_ols_could_not():
    # v = m*u + c has infinite slope here; TLS has no preferred axis.
    v = np.linspace(0, 80, 40)
    u = np.full_like(v, -3.0)
    fit = S.fit_line_tls(u, v)
    assert fit is not None
    assert abs(abs(fit["angle_deg"]) - 90.0) < 1e-6, fit["angle_deg"]
    along, perp = S.project_to_line(u, v, fit)
    assert np.max(np.abs(perp)) < 1e-9
    assert abs((along.max() - along.min()) - 80.0) < 1e-9


def test_angle_is_direction_independent():
    # The same ruler driven both ways must report the same angle: the line
    # is undirected, so a sign flip in the fitted direction is meaningless.
    t = np.linspace(0, 50, 30)
    f1 = S.fit_line_tls(t, 0.5 * t)
    f2 = S.fit_line_tls(t[::-1], 0.5 * t[::-1])
    assert abs(f1["angle_deg"] - f2["angle_deg"]) < 1e-9


def test_fit_returns_none_for_degenerate_input():
    assert S.fit_line_tls([], []) is None
    assert S.fit_line_tls([1.0], [2.0]) is None
    # All points identical -> no direction exists.
    assert S.fit_line_tls([5.0] * 10, [5.0] * 10) is None


def test_known_offset_point_gives_its_exact_perpendicular_distance():
    # 20 points on the u axis plus one pushed exactly 0.5mm off it. The
    # fitted line tilts slightly to accommodate it, so assert the max
    # deviation is close to (but at most) the constructed 0.5mm.
    u = np.linspace(0, 100, 21)
    v = np.zeros_like(u)
    v[10] = 0.5                      # the middle point, so the tilt is minimal
    fit = S.fit_line_tls(u, v)
    _, perp = S.project_to_line(u, v, fit)
    assert 0.45 < np.max(np.abs(perp)) <= 0.5 + 1e-9, np.max(np.abs(perp))


# ==================================================================== bow
def test_bow_recovers_a_constructed_parabola_amplitude():
    # A path bowed by a known amount: perp = A * (1 - (2s/L - 1)^2), i.e. a
    # parabola peaking at A in the middle and 0 at both ends.
    L, A = 100.0, 0.4
    s = np.linspace(0, L, 101)
    u = s
    v = A * (1.0 - (2.0 * s / L - 1.0) ** 2)
    fit = S.fit_line_tls(u, v)
    along, perp = S.project_to_line(u, v, fit)
    bow = S.fit_bow(along, perp)
    assert bow is not None
    # The fitted quadratic reproduces the constructed shape, so its
    # peak-to-peak across the span is the full amplitude A.
    assert abs(bow["bow_mm"] - A) < 0.02, bow["bow_mm"]
    # A pure parabola is entirely systematic: nothing left over.
    assert bow["random_rms_mm"] < 1e-6, bow["random_rms_mm"]
    assert bow["systematic_rms_mm"] > 0.05


def test_bow_calls_pure_noise_random_not_systematic():
    rng = np.random.default_rng(12345)
    u = np.linspace(0, 100, 400)
    v = rng.normal(0.0, 0.05, u.size)
    fit = S.fit_line_tls(u, v)
    along, perp = S.project_to_line(u, v, fit)
    bow = S.fit_bow(along, perp)
    # Zero-mean noise has no quadratic shape to find, so the random part
    # must dominate -- this is the discriminator the verdict relies on.
    assert bow["random_rms_mm"] > 5.0 * bow["systematic_rms_mm"], bow


def test_bow_needs_three_points():
    assert S.fit_bow([0.0, 1.0], [0.0, 0.0]) is None


# ================================================================ binning
def test_binned_profile_splits_the_span_and_counts_every_sample():
    along = np.linspace(0.0, 100.0, 101)
    perp = np.zeros_like(along)
    bins = S.binned_profile(along, perp, n_bins=10)
    assert len(bins) == 10
    assert sum(b["count"] for b in bins) == along.size   # nothing dropped
    assert abs(bins[0]["start_mm"] - 0.0) < 1e-9
    assert abs(bins[-1]["end_mm"] - 100.0) < 1e-9


def test_binned_profile_localises_a_deviation_to_the_right_bin():
    # Deviation only in the last fifth of the travel: the report must show
    # it *there* and show ~0 everywhere else. This is the "wie groß ist die
    # Abweichung je nach Position" question in its purest form.
    along = np.linspace(0.0, 100.0, 201)
    perp = np.where(along > 80.0, 0.3, 0.0)
    bins = S.binned_profile(along, perp, n_bins=10)
    assert abs(bins[0]["mean_mm"]) < 1e-9
    assert abs(bins[4]["mean_mm"]) < 1e-9
    assert bins[9]["mean_mm"] > 0.25, bins[9]


def test_binned_profile_keeps_empty_bins_visible():
    # A gap in the travel must stay a gap, not silently close up.
    along = np.concatenate([np.linspace(0, 10, 20), np.linspace(90, 100, 20)])
    perp = np.zeros_like(along)
    bins = S.binned_profile(along, perp, n_bins=10)
    assert any(b["count"] == 0 for b in bins)
    for b in bins:
        if b["count"] == 0:
            assert b["mean_mm"] is None and b["rms_mm"] is None


# =============================================================== rotation
def test_relative_rotation_is_zero_for_a_constant_orientation():
    n = 20
    q = np.zeros((n, 4))
    q[:, 3] = 1.0                       # identity quaternion throughout
    rot = S.relative_rotation_deg(q[:, 0], q[:, 1], q[:, 2], q[:, 3])
    assert rot is not None
    assert np.nanmax(np.abs(rot)) < 1e-9


def test_relative_rotation_recovers_a_known_angle():
    # 10 degrees about z, built directly as a quaternion.
    half = math.radians(10.0) / 2.0
    q = [(0.0, 0.0, 0.0, 1.0), (0.0, 0.0, math.sin(half), math.cos(half))]
    arr = np.array(q)
    rot = S.relative_rotation_deg(arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3])
    assert abs(rot[0]) < 1e-9
    assert abs(rot[1] - 10.0) < 1e-6, rot[1]


def test_relative_rotation_folds_the_quaternion_sign():
    # q and -q are the same rotation; a sign flip must not read as a swing.
    arr = np.array([(0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0, -1.0)])
    rot = S.relative_rotation_deg(arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3])
    assert abs(rot[1]) < 1e-9, rot[1]


def test_relative_rotation_ignores_blank_quaternions():
    arr = np.array([(0.0, 0.0, 0.0, 1.0),
                    (np.nan, np.nan, np.nan, np.nan),
                    (0.0, 0.0, 0.0, 1.0)])
    rot = S.relative_rotation_deg(arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3])
    assert math.isnan(rot[1])           # not silently "no rotation"
    assert abs(rot[2]) < 1e-9


def test_relative_rotation_returns_none_when_nothing_is_usable():
    nan4 = [np.nan, np.nan]
    assert S.relative_rotation_deg(nan4, nan4, nan4, nan4) is None


def test_lever_arm_is_the_arc_length_of_the_measured_offset():
    # The report's central warning rests on this conversion: rotating the
    # cart by an angle swings the nozzle-referenced point by the arc length
    # r * theta of the sensor->bar offset. Derived from the constant rather
    # than pinned to today's millimetres -- that offset is a measured value
    # and has already been re-measured once (62.36 -> 45.5mm), which must
    # flow through to the report instead of failing this test.
    got = S.lever_arm_mm(1.0)
    expected = abs(SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM) * math.radians(1.0)
    assert abs(got - expected) < 1e-12
    # Still a real magnitude check, just expressed relative to the offset:
    # one degree is always ~1.75% of the lever arm, whatever it measures.
    assert abs(got / abs(SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM) - 0.017453) < 1e-5
    # Doubling the angle doubles the swing (linear, not trigonometric).
    assert abs(S.lever_arm_mm(2.0) - 2 * got) < 1e-12


# ============================================================ CSV reading
def _write_csv(text):
    fh = tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False)
    fh.write(text)
    fh.close()
    return fh.name


def test_read_profile_csv_parses_page_mode_columns():
    path = _write_csv(
        "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw\n"
        "0.1,0,0,0.000,0.000,10.00,50.0,5.000,6.000,7.000,0.0000,0.0000,0.0000,1.0000\n"
        "0.2,1,1,1.000,0.100,10.00,50.0,6.000,6.100,7.000,0.0000,0.0000,0.0000,1.0000\n")
    try:
        data = S.read_profile_csv(path)
        assert np.allclose(data["u_mm"], [0.0, 1.0])
        assert np.allclose(data["v_mm"], [0.0, 0.1])
        assert np.allclose(data["qw"], [1.0, 1.0])
    finally:
        os.unlink(path)


def test_read_profile_csv_turns_blank_quaternions_into_nan():
    path = _write_csv(
        "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw\n"
        "0.1,0,0,0.000,0.000,10.00,50.0,5.000,6.000,7.000,,,,\n")
    try:
        data = S.read_profile_csv(path)
        assert math.isnan(data["qx"][0]) and math.isnan(data["qw"][0])
    finally:
        os.unlink(path)


def test_read_profile_csv_rejects_a_line_mode_file_with_a_real_diagnosis():
    path = _write_csv(
        "t_s,column,advance_mm,write_latency_ms,speed_mm_s,x,y,z\n"
        "0.1,0,0.000,3.100,10.00,5.000,6.000,7.000\n")
    try:
        S.read_profile_csv(path)
    except ValueError as exc:
        msg = str(exc).lower()
        assert "line-mode" in msg and "--mode page" in msg, msg
        return
    finally:
        os.unlink(path)
    raise AssertionError("expected a ValueError for a line-mode CSV")


def test_read_profile_csv_exposes_the_raw_sensor_columns():
    # The reader floats every column it finds, so the new raw-sensor columns
    # have to arrive as usable arrays -- that is what makes a profile CSV
    # answer "what did the tracker say" alongside "where did the ink go",
    # instead of needing a separate --pos run.
    path = _write_csv(
        "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw\n"
        "0.1,0,0,0.000,0.000,10.00,50.0,5.000,6.000,7.000,0.0,0.0,0.0,1.0\n"
        "0.2,1,1,1.000,0.100,10.00,50.0,6.000,6.100,7.000,0.0,0.0,0.0,1.0\n")
    try:
        data = S.read_profile_csv(path)
        assert np.allclose(data["x"], [5.0, 6.0])
        assert np.allclose(data["y"], [6.0, 6.1])
        assert np.allclose(data["z"], [7.0, 7.0])
        # ...without disturbing what was already there.
        assert np.allclose(data["u_mm"], [0.0, 1.0])
        assert np.allclose(data["qw"], [1.0, 1.0])
    finally:
        os.unlink(path)


# ========================================================= analyze / report
def _page_csv(u, v, quats=None):
    lines = ["t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw"]
    for i, (uu, vv) in enumerate(zip(u, v)):
        if quats is None:
            q = "0.0000,0.0000,0.0000,1.0000"
        else:
            qx, qy, qz, qw = quats[i]
            q = f"{qx:.4f},{qy:.4f},{qz:.4f},{qw:.4f}"
        lines.append(f"{i * 0.01:.4f},0,0,{uu:.3f},{vv:.3f},10.00,50.0,"
                     f"{uu:.3f},{vv:.3f},0.000,{q}")
    return _write_csv("\n".join(lines) + "\n")


def test_analyze_reports_near_zero_deviation_for_a_perfect_ruler_run():
    u = np.linspace(0, 150, 200)
    v = np.zeros_like(u)
    path = _page_csv(u, v)
    try:
        res = S.analyze(S.read_profile_csv(path))
        assert "error" not in res
        assert res["rms_mm"] < 1e-9
        assert res["enough_travel"] is True
        assert abs(res["travel_mm"] - 150.0) < 1e-6
        assert "sehr gerade" in S.format_report(res)
    finally:
        os.unlink(path)


def test_analyze_flags_a_too_short_run_before_calling_it_straight():
    # A 10mm dab is trivially straight and must NOT read as an all-clear.
    u = np.linspace(0, 10, 50)
    path = _page_csv(u, np.zeros_like(u))
    try:
        res = S.analyze(S.read_profile_csv(path))
        assert res["enough_travel"] is False
        report = S.format_report(res)
        assert "zu kurze Strecke" in report, report
        assert "sehr gerade" not in report
    finally:
        os.unlink(path)


def test_analyze_recovers_a_constructed_deviation_magnitude():
    # A known 0.25mm step in the second half of a 200mm run. The expected
    # RMS is NOT the naive 0.125mm (half the samples either side of the
    # mean): the fitted line TILTS to follow the step, and that tilt
    # absorbs part of the deviation. Derived rather than observed, so this
    # test pins real arithmetic instead of whatever the code emits --
    #
    #   centre u at the step: s in [-100, +100], v_centred = -+0.125
    #   slope = cov(s, v)/var(s) = (0.125 * E|s|) / (100^2/3)
    #         = 6.25 / 3333.33 = 0.001875
    #   RMS^2 = var(v) - slope^2 * var(s)
    #         = 0.125^2 - 0.001875^2 * 3333.33
    #         = 0.015625 - 0.01171875 = 0.00390625
    #   RMS   = 0.0625 mm   (exactly half the naive figure)
    u = np.linspace(0, 200, 400)
    v = np.where(u > 100.0, 0.25, 0.0)
    path = _page_csv(u, v)
    try:
        res = S.analyze(S.read_profile_csv(path))
        assert abs(res["rms_mm"] - 0.0625) < 1e-3, res["rms_mm"]
        # The position breakdown must localise the step to where it
        # actually is -- the MIDDLE of the run, not the ends. With the
        # tilt included the residual is a sawtooth,
        #   residual(s) = v_centred(s) - 0.001875 * s
        # running +0.0625 -> -0.125 across the first half, then jumping to
        # +0.125 -> -0.0625 across the second. So the extreme bins are the
        # two straddling the step (index 4 and 5 of 10), with opposite
        # signs and the largest magnitudes anywhere in the run.
        bins = res["bins"]
        means = [b["mean_mm"] for b in bins]
        assert means[4] < 0 < means[5], means
        assert means[4] == min(means) and means[5] == max(means), means
    finally:
        os.unlink(path)


def test_binned_means_are_flat_when_the_path_is_genuinely_straight():
    # Counterpart to the test above: no constructed defect -> every bin
    # must report ~0, so a real report showing structure means something.
    u = np.linspace(0, 200, 400)
    path = _page_csv(u, np.zeros_like(u))
    try:
        res = S.analyze(S.read_profile_csv(path))
        for b in res["bins"]:
            assert abs(b["mean_mm"]) < 1e-9, b
    finally:
        os.unlink(path)


def test_analyze_reports_rotation_and_its_lever_arm_cost():
    # Cart rotates a full 2 degrees over the run -> the report must say
    # that alone is worth >2mm of apparent deviation.
    n = 200
    u = np.linspace(0, 150, n)
    quats = []
    for i in range(n):
        half = math.radians(2.0 * i / (n - 1)) / 2.0
        quats.append((0.0, 0.0, math.sin(half), math.cos(half)))
    path = _page_csv(u, np.zeros_like(u), quats=quats)
    try:
        res = S.analyze(S.read_profile_csv(path))
        rot = res["rotation"]
        assert rot is not None
        assert abs(rot["span_deg"] - 2.0) < 0.01, rot["span_deg"]
        # Derived from the measured offset, not pinned to a millimetre
        # figure: the point is that the reported lever-arm cost matches the
        # rotation actually found, whatever the offset currently measures.
        assert abs(rot["lever_arm_mm"] - S.lever_arm_mm(2.0)) < 0.01, rot
        # ... and that it is large enough to matter -- many nozzle rows of
        # apparent deviation from 2 degrees of hand twist.
        assert rot["lever_arm_mm"] / NOZZLE_PITCH_MM > 5.0, rot["lever_arm_mm"]
        report = S.format_report(res)
        assert "Wagen-Drehung" in report
        assert "gedreht" in report      # the verdict's rotation warning fired
    finally:
        os.unlink(path)


def test_analyze_says_so_when_the_csv_has_no_orientation():
    u = np.linspace(0, 150, 100)
    lines = ["t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,x,y,z,qx,qy,qz,qw"]
    for i, uu in enumerate(u):
        lines.append(f"{i * 0.01:.4f},0,0,{uu:.3f},0.000,10.00,50.0,"
                     f"{uu:.3f},0.000,0.000,,,,")
    path = _write_csv("\n".join(lines) + "\n")
    try:
        res = S.analyze(S.read_profile_csv(path))
        assert res["rotation"] is None
        assert "keine Orientierung im CSV" in S.format_report(res)
    finally:
        os.unlink(path)


def test_analyze_errors_cleanly_on_a_single_sample():
    path = _page_csv([1.0], [2.0])
    try:
        res = S.analyze(S.read_profile_csv(path))
        assert "error" in res
        assert "[straightness]" in S.format_report(res)
    finally:
        os.unlink(path)


def test_report_expresses_deviation_in_nozzle_rows_too():
    # A deviation's print impact is measured in nozzle rows, so the report
    # must carry that unit next to the millimetres.
    u = np.linspace(0, 200, 300)
    v = np.where(u > 100.0, 10 * NOZZLE_PITCH_MM, 0.0)
    path = _page_csv(u, v)
    try:
        report = S.format_report(S.analyze(S.read_profile_csv(path)))
        assert "Düsenreihen" in report
    finally:
        os.unlink(path)


def test_analyze_csv_reports_a_missing_file_without_raising():
    out = S.analyze_csv("/definitely/not/here.csv")
    assert out.startswith("[straightness]")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All straightness tests passed.")
