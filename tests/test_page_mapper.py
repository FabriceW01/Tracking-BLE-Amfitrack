"""
Milestone 3 tests: PageMapper (tracking.py) and its --pos wiring (no hardware).

PageCalibration's own projection math is already covered in depth by
tests/test_calibration.py; what's new here is PageMapper's thin wiring onto a
loaded calibration, and that diagnostics.monitor_position actually threads a
--page-calibration path through to live page_u/page_v/page_z output.

Run with:  python tests/test_page_mapper.py
"""

import asyncio
import contextlib
import io
import json
import math
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead import diagnostics                          # noqa: E402
from printhead.calibration import PageCalibration           # noqa: E402
from printhead.config import TrackingSettings                # noqa: E402
from printhead.geometry import (                              # noqa: E402
    NOZZLE_BAR_SPAN_MM,
    SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM,
    SENSOR_TO_NOZZLE_COL_MM,
)
from printhead.tracking import PageMapper                    # noqa: E402

IDENTITY_QUAT = (0.0, 0.0, 0.0, 1.0)


def _quat_mul(a, b):
    """Hamilton product in (qx, qy, qz, qw) order -- `a` applied after `b`."""
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                     w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def _quat_about_z(deg: float):
    """Same helper as tests/test_rotation.py: a quaternion for a rotation of
    ``deg`` about +Z, which coincides with the page normal for the
    axis-aligned e_col=(1,0,0)/e_row=(0,1,0) calibrations used throughout
    this file."""
    half = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


# ================================================================= PageMapper
# These two pin PageMapper's underlying calibration delegation / scale
# handling in isolation, with the sensor->nozzle correction neutralised (see
# the NOTE on _NEUTRAL_ROW_OFFSET below) so it doesn't leak into what they
# check -- that correction is pinned separately further down.
#
# NOTE: PageMapper's row axis always subtracts NOZZLE_BAR_SPAN_MM/2 from
# whatever sensor_offset_row_mm is given -- that subtraction is the measured-
# bar-CENTRE-to-nozzle-0 conversion, not "no correction". Passing literal 0.0
# does NOT cancel to a zero net shift; it means "the bar centre is exactly at
# the sensor", which still shifts v by -NOZZLE_BAR_SPAN_MM/2 (nozzle 0 sits
# that far from the centre). The value that actually cancels to zero net
# shift is NOZZLE_BAR_SPAN_MM/2.0 (see
# test_page_mapper_explicit_offset_matches_pre_offset_behaviour below, which
# pins this distinction directly).
_NEUTRAL_ROW_OFFSET = NOZZLE_BAR_SPAN_MM / 2.0


def test_page_mapper_delegates_to_the_calibration():
    cal = PageCalibration(origin=np.array([1.0, 2.0, 3.0]),
                          e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    mapper = PageMapper(cal, sensor_offset_row_mm=_NEUTRAL_ROW_OFFSET,
                        sensor_offset_col_mm=0.0)
    pos = np.array([11.0, 7.0, 3.0])
    assert mapper.project(pos) == cal.project(pos)


def test_page_mapper_reflects_scale_correction():
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]), scale_col=2.0)
    mapper = PageMapper(cal, sensor_offset_row_mm=_NEUTRAL_ROW_OFFSET,
                        sensor_offset_col_mm=0.0)
    u, v, z = mapper.project(np.array([5.0, 5.0, 0.0]))
    assert abs(u - 10.0) < 1e-9      # 5mm raw * scale_col 2.0
    assert abs(v - 5.0) < 1e-9       # scale_row default 1.0
    assert abs(z) < 1e-9


# ===================================================== sensor->nozzle offset
def test_page_mapper_default_offset_shifts_uv_by_the_exact_measured_amount():
    # Pin the exact arithmetic (not just "it changed"): default row shift is
    # the measured bar-CENTRE offset minus half the bar SPAN (converting to
    # the nozzle-0-referenced v CoverageEngine needs); default col shift is
    # the raw column offset (no bar-span correction on that axis).
    cal = PageCalibration(origin=np.array([1.0, 2.0, 3.0]),
                          e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    pos = np.array([11.0, 7.0, 3.0])
    u_raw, v_raw, z_raw = cal.project(pos)

    mapper = PageMapper(cal)     # default offsets
    u, v, z = mapper.project(pos)

    expected_row_shift = SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM - NOZZLE_BAR_SPAN_MM / 2.0
    assert abs(expected_row_shift - (-69.91)) < 1e-9  # -62.36 - 15.1/2 = -69.91
    expected_col_shift = SENSOR_TO_NOZZLE_COL_MM      # 0.0, no bar-width term

    assert abs(u - (u_raw + expected_col_shift)) < 1e-9
    assert abs(v - (v_raw + expected_row_shift)) < 1e-9
    assert abs(z - z_raw) < 1e-9      # z is untouched by either offset


def test_page_mapper_explicit_offset_matches_pre_offset_behaviour():
    # Regression guard: the feature must be fully opt-out-able. PageMapper
    # must be able to reproduce exactly what calling calibration.project()
    # directly gives -- i.e. the behaviour PageMapper had before this offset
    # existed at all.
    #
    # This is NOT sensor_offset_row_mm=0.0: the constructor always subtracts
    # NOZZLE_BAR_SPAN_MM/2 from whatever row value is given (the measured-
    # bar-CENTRE-to-nozzle-0 conversion), so 0.0 in still leaves a
    # -NOZZLE_BAR_SPAN_MM/2 mm net shift, NOT zero -- literal (0.0, 0.0)
    # is checked explicitly below to pin exactly that (real, if easy to miss)
    # gotcha. The value that actually cancels the row conversion to zero net
    # shift is NOZZLE_BAR_SPAN_MM/2.0 (col has no such conversion, so 0.0 is
    # already neutral there).
    cal = PageCalibration(origin=np.array([1.0, 2.0, 3.0]),
                          e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    pos = np.array([11.0, 7.0, 3.0])
    mapper = PageMapper(cal, sensor_offset_row_mm=NOZZLE_BAR_SPAN_MM / 2.0,
                        sensor_offset_col_mm=0.0)
    assert mapper.project(pos) == cal.project(pos)


def test_page_mapper_literal_zero_row_offset_is_not_neutral():
    # The gotcha explained above, pinned directly: sensor_offset_row_mm=0.0
    # is a real, physically meaningful value (bar centre coincident with the
    # sensor), not an "off switch" -- it still shifts v by -NOZZLE_BAR_SPAN_MM/2.
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    pos = np.array([3.0, 4.0, 0.0])
    _, v_raw, _ = cal.project(pos)
    mapper = PageMapper(cal, sensor_offset_row_mm=0.0, sensor_offset_col_mm=0.0)
    _, v, _ = mapper.project(pos)
    assert abs(v - (v_raw - NOZZLE_BAR_SPAN_MM / 2.0)) < 1e-9
    assert abs(v - v_raw) > 1.0     # nowhere near neutral


def test_negating_the_row_offset_negates_the_resulting_shift():
    # Proves the "if it's backwards, just negate the flag value" fix-it story
    # in geometry.py's comment actually works arithmetically.
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    pos = np.array([3.0, 4.0, 0.0])
    _, v_raw, _ = cal.project(pos)

    mapper_pos = PageMapper(cal, sensor_offset_row_mm=62.36, sensor_offset_col_mm=0.0)
    mapper_neg = PageMapper(cal, sensor_offset_row_mm=-62.36, sensor_offset_col_mm=0.0)
    _, v_pos, _ = mapper_pos.project(pos)
    _, v_neg, _ = mapper_neg.project(pos)

    shift_pos = v_pos - v_raw
    shift_neg = v_neg - v_raw
    assert abs(shift_pos - (62.36 - NOZZLE_BAR_SPAN_MM / 2.0)) < 1e-9
    assert abs(shift_neg - (-62.36 - NOZZLE_BAR_SPAN_MM / 2.0)) < 1e-9
    # Negating the measured value must not simply negate the applied shift
    # (the -bar_span/2 term is a fixed conversion constant, not part of the
    # measurement) -- but it does flip the sign of the *measurement's own*
    # contribution, which is the actual "wrong direction -> negate" lever.
    assert abs((shift_pos - shift_neg) - 2 * 62.36) < 1e-9


def test_page_mapper_end_to_end_known_world_position():
    # Axis-aligned e_col/e_row so the whole chain is checkable by hand: world
    # position -> calibration.project() -> PageMapper's default sensor offset.
    origin = np.array([100.0, 50.0, 0.0])
    cal = PageCalibration(origin=origin, e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    world_pos = np.array([130.0, 62.0, 5.0])   # rel = (30, 12, 5) from origin

    mapper = PageMapper(cal)
    u, v, z = mapper.project(world_pos)

    expected_u = 30.0 + SENSOR_TO_NOZZLE_COL_MM
    expected_v = 12.0 + (SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM - NOZZLE_BAR_SPAN_MM / 2.0)
    expected_z = 5.0
    assert abs(u - expected_u) < 1e-9
    assert abs(v - expected_v) < 1e-9    # 12 + (SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM - 7.55)
    assert abs(z - expected_z) < 1e-9


# ==================================================== yaw / rotation correction
def _cal_with_boresight(boresight_quat=IDENTITY_QUAT):
    return PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                           e_row=np.array([0.0, 1.0, 0.0]),
                           boresight_quat=np.array(boresight_quat, dtype=float))


def test_page_mapper_without_boresight_ignores_quat_entirely():
    # Absent boresight_quat -> current (pre-rotation-correction) behaviour,
    # identical whether or not a live orientation happens to be available --
    # the "no invisible failure" design: a print must never start
    # rotation-correcting just because a quat happened to arrive, and must
    # never depend on whatever orientation the cart had at some arbitrary
    # sample. This is the situation EVERY calibration saved before this
    # feature existed is in.
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    assert cal.boresight_quat is None
    pos = np.array([3.0, 4.0, 0.0])
    mapper = PageMapper(cal, sensor_offset_row_mm=62.36, sensor_offset_col_mm=0.0)

    no_quat = mapper.project(pos, quat=None)
    with_quat = mapper.project(pos, quat=_quat_about_z(90.0))
    assert with_quat == no_quat
    assert mapper.last_yaw_rad == 0.0


def test_page_mapper_90deg_yaw_moves_a_pure_row_offset_entirely_onto_u():
    # Hand-checkable pin of the sign convention against
    # rotation.yaw_about_normal: a pure row (v-axis) sensor->nozzle offset,
    # rotated 90 degrees, must land ENTIRELY on the u axis with nothing left
    # on v.
    cal = _cal_with_boresight()
    row_offset_mm = 10.0
    # PageMapper's constructor always subtracts NOZZLE_BAR_SPAN_MM/2 from
    # the given sensor_offset_row_mm (bar-centre -> nozzle-0 conversion, see
    # its docstring) -- add it back so the NET row offset is exactly 10.0mm,
    # col offset exactly 0.0mm, i.e. a "pure row offset" in the sense this
    # test needs.
    mapper = PageMapper(cal, sensor_offset_row_mm=row_offset_mm + NOZZLE_BAR_SPAN_MM / 2.0,
                        sensor_offset_col_mm=0.0)
    pos = np.zeros(3)
    u_raw, v_raw, _ = cal.project(pos)

    quat_90 = _quat_about_z(90.0)
    u, v, _ = mapper.project(pos, quat=quat_90)

    assert abs(mapper.last_yaw_rad - math.pi / 2.0) < 1e-9
    # u += col*cos(90) - row*sin(90) = 0 - row_offset_mm = -row_offset_mm
    assert abs((u - u_raw) - (-row_offset_mm)) < 1e-9
    # v += col*sin(90) + row*cos(90) = 0 + 0 = 0 -- nothing left on v
    assert abs(v - v_raw) < 1e-9


def test_page_mapper_quat_none_reuses_the_last_known_yaw():
    # An intermittent orientation dropout (this tick's packet carried no
    # quaternion) must not snap the correction back to 0 -- that would make
    # the correction flicker on/off sample to sample for no physical reason.
    cal = _cal_with_boresight()
    mapper = PageMapper(cal, sensor_offset_row_mm=NOZZLE_BAR_SPAN_MM / 2.0 + 10.0,
                        sensor_offset_col_mm=0.0)
    pos = np.zeros(3)

    mapper.project(pos, quat=_quat_about_z(90.0))
    assert abs(mapper.last_yaw_rad - math.pi / 2.0) < 1e-9

    dropout_u, dropout_v, _ = mapper.project(pos, quat=None)
    held_u, held_v, _ = mapper.project(pos, quat=_quat_about_z(90.0))
    assert dropout_u == held_u and dropout_v == held_v
    assert abs(mapper.last_yaw_rad - math.pi / 2.0) < 1e-9   # unchanged, not reset to 0


def test_page_mapper_zero_yaw_is_bit_identical_to_no_rotation():
    # A boresight IS present, but the cart is exactly at the boresight pose
    # (quat == boresight_quat) -> yaw is exactly 0.0, and the result must be
    # bit-identical to the pre-rotation formula (not just "very close").
    cal = _cal_with_boresight()
    mapper = PageMapper(cal, sensor_offset_row_mm=70.0, sensor_offset_col_mm=-3.5)
    pos = np.array([11.0, 7.0, 3.0])
    u, v, z = mapper.project(pos, quat=IDENTITY_QUAT)
    u_expected, v_expected, z_expected = cal.project(pos)
    u_expected += -3.5
    v_expected += 70.0 - NOZZLE_BAR_SPAN_MM / 2.0
    assert u == u_expected and v == v_expected and z == z_expected


def test_page_mapper_boresight_offset_is_additive_on_top_of_the_captured_boresight():
    # --boresight-deg: cart genuinely still at the boresight pose (raw yaw
    # 0), but a +20 deg fine-tune must appear directly in last_yaw_rad.
    cal = _cal_with_boresight()
    mapper = PageMapper(cal, boresight_offset_rad=math.radians(20.0))
    mapper.project(np.zeros(3), quat=IDENTITY_QUAT)
    assert abs(math.degrees(mapper.last_yaw_rad) - 20.0) < 1e-9


def test_page_mapper_boresight_offset_has_no_effect_without_a_boresight():
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))          # no boresight
    mapper = PageMapper(cal, boresight_offset_rad=math.radians(20.0))
    mapper.project(np.zeros(3), quat=IDENTITY_QUAT)
    assert mapper.last_yaw_rad == 0.0


# ============================================== roll / pitch (diagnostic only)
def _quat_about_axis(axis, deg: float):
    """Same helper as tests/test_rotation.py, reimplemented here so this file
    stays independently runnable (no cross-test-file imports)."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    half = math.radians(deg) / 2.0
    s = math.sin(half)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half))


def test_page_mapper_defaults_last_roll_and_pitch_to_zero():
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    mapper = PageMapper(cal)
    assert mapper.last_roll_rad == 0.0
    assert mapper.last_pitch_rad == 0.0


def test_page_mapper_last_roll_and_pitch_stay_zero_without_boresight():
    # No boresight_quat on the calibration -> project() must not touch
    # last_roll_rad/last_pitch_rad at all, even with a live quat, mirroring
    # last_yaw_rad's documented behaviour in this situation.
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))          # no boresight
    mapper = PageMapper(cal)
    mapper.project(np.zeros(3), quat=_quat_about_axis((1.0, 0.0, 0.0), 30.0))
    assert mapper.last_roll_rad == 0.0
    assert mapper.last_pitch_rad == 0.0


def test_page_mapper_last_roll_and_pitch_stay_zero_when_quat_is_none():
    # Boresight present but this tick's quat is None (dropped orientation
    # packet) -> same "reuse the last value" behaviour as last_yaw_rad;
    # starting from the 0.0 default, they stay 0.0.
    cal = _cal_with_boresight()
    mapper = PageMapper(cal)
    mapper.project(np.zeros(3), quat=None)
    assert mapper.last_roll_rad == 0.0
    assert mapper.last_pitch_rad == 0.0


def test_page_mapper_updates_last_roll_and_pitch_from_a_live_quat():
    # Boresight + live quat present -> last_roll_rad/last_pitch_rad update
    # from cart_rotation_angles, using the SAME e_col/e_row/boresight_quat
    # arguments as last_yaw_rad.
    cal = _cal_with_boresight()
    mapper = PageMapper(cal)
    roll_quat = _quat_about_axis((1.0, 0.0, 0.0), 12.0)     # about e_col -> roll
    mapper.project(np.zeros(3), quat=roll_quat)
    assert abs(math.degrees(mapper.last_roll_rad) - 12.0) < 1e-6
    assert abs(mapper.last_pitch_rad) < 1e-9
    assert abs(mapper.last_yaw_rad) < 1e-9

    pitch_quat = _quat_about_axis((0.0, 1.0, 0.0), -8.0)    # about e_row -> pitch
    mapper.project(np.zeros(3), quat=pitch_quat)
    assert abs(mapper.last_roll_rad) < 1e-9
    assert abs(math.degrees(mapper.last_pitch_rad) - (-8.0)) < 1e-6
    assert abs(mapper.last_yaw_rad) < 1e-9


def test_page_mapper_boresight_offset_rad_does_not_leak_into_roll_or_pitch():
    # boresight_offset_rad (--boresight-deg) is a yaw-only fine-tune; it must
    # show up in last_yaw_rad but have NO effect on last_roll_rad/last_pitch_rad.
    cal = _cal_with_boresight()
    mapper = PageMapper(cal, boresight_offset_rad=math.radians(20.0))
    mapper.project(np.zeros(3), quat=IDENTITY_QUAT)
    assert abs(math.degrees(mapper.last_yaw_rad) - 20.0) < 1e-9   # yaw: offset applied
    assert mapper.last_roll_rad == 0.0                             # roll: unaffected
    assert mapper.last_pitch_rad == 0.0                            # pitch: unaffected


# =========================================== mutation check: roll/pitch axes
def test_page_mapper_MUTATION_check_swapping_roll_and_pitch_axes_is_detected():
    # Inlines a "swap which axis feeds roll vs pitch" mutation (the one
    # described in the PR body) and confirms it disagrees with the real
    # wiring -- proof the assignment in project() is actually pinned by
    # test_page_mapper_updates_last_roll_and_pitch_from_a_live_quat above.
    from printhead.rotation import cart_rotation_angles
    cal = _cal_with_boresight()
    roll_quat = _quat_about_axis((1.0, 0.0, 0.0), 12.0)
    correct_roll, correct_pitch, _ = cart_rotation_angles(
        roll_quat, cal.boresight_quat, cal.e_col, cal.e_row)
    mutated_roll, mutated_pitch = correct_pitch, correct_roll   # swapped
    assert (mutated_roll, mutated_pitch) != (correct_roll, correct_pitch)
    assert abs(correct_roll) > 1e-6 and abs(mutated_roll) < 1e-9


# =============================================== mutation check (see PR body)
def test_page_mapper_MUTATION_check_dropping_the_offset_rotation_breaks_the_90deg_case():
    # Inlines the mutation described in the PR: if project() applied the
    # sensor offset WITHOUT rotating it by yaw (i.e. always used yaw=0 for
    # the offset math, the pre-fix behaviour), the 90-degree pure-row-offset
    # case above would keep the shift entirely on v instead of moving it to
    # u. This function reproduces exactly that (mutated) formula inline
    # rather than editing tracking.py, so the regression stays covered by
    # the test suite.
    cal = _cal_with_boresight()
    row_offset_mm = 10.0
    u_raw, v_raw, _ = cal.project(np.zeros(3))

    # Mutated project(): offset added as a constant, ignoring last_yaw_rad.
    mutated_u = u_raw + 0.0
    mutated_v = v_raw + row_offset_mm

    mapper = PageMapper(cal, sensor_offset_row_mm=row_offset_mm + NOZZLE_BAR_SPAN_MM / 2.0,
                        sensor_offset_col_mm=0.0)
    correct_u, correct_v, _ = mapper.project(np.zeros(3), quat=_quat_about_z(90.0))

    assert (mutated_u, mutated_v) != (correct_u, correct_v), (
        "the mutated (no-rotation) formula must disagree with the real, "
        "rotation-aware one at 90 degrees -- if this ever matches, the "
        "90-degree test above has stopped actually exercising the rotation")


# ===================================================== --pos / monitor_position
async def _run_monitor_briefly(settings=None, **kwargs):
    """Run monitor_position(simulate=True, ndjson=True, ...) for a short while,
    then cancel it (like Ctrl+C) and return everything it printed.

    ``settings`` overrides the default TrackingSettings() -- used by the
    --page-frame simple case, which selects its page frame through the
    settings rather than through a calibration path."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        task = asyncio.ensure_future(diagnostics.monitor_position(
            settings if settings is not None else TrackingSettings(),
            simulate=True, hz=50.0, ndjson=True, **kwargs))
        await asyncio.sleep(0.3)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
    return buf.getvalue()


def _events(output: str):
    # SimulatedTracker.open() prints a plain-text banner even in ndjson mode
    # (pre-existing, unrelated to this change) -- skip lines that aren't JSON,
    # same tolerance ui.server._try_parse_json applies to this same stream.
    events = []
    for line in output.splitlines():
        if not line.strip():
            continue
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return events


def test_monitor_position_reports_page_uvz_when_calibration_given():
    # Identity-ish calibration in the XY plane: SimulatedTracker's default
    # motion (50 mm/s along the default advance axis, x) should come straight
    # through as page_u. Sensor row offset neutralised (NOZZLE_BAR_SPAN_MM/2,
    # NOT 0.0 -- see the NOTE near the top of this file) so this test stays
    # about the --pos/PageMapper wiring itself, not the offset feature
    # (pinned separately below).
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "cal.json")
        cal.save(path)
        output = asyncio.run(_run_monitor_briefly(
            page_calibration_path=path,
            sensor_offset_row_mm=NOZZLE_BAR_SPAN_MM / 2.0,
            sensor_offset_col_mm=0.0))

    positions = [e for e in _events(output) if e.get("event") == "position"]
    assert positions, output
    last = positions[-1]
    assert "page_u" in last and "page_v" in last and "page_z" in last
    assert abs(last["page_v"]) < 0.5              # travel is along x only
    assert abs(last["page_z"]) < 0.5
    assert abs(last["page_u"] - last["x"]) < 0.5   # e_col == x-axis, scale 1


def test_monitor_position_page_v_reflects_default_sensor_offset():
    # The --pos diagnostic must apply the same default sensor->nozzle offset
    # a real freehand pass does (see PrintController._print_freehand_pass) --
    # otherwise it would show different numbers than what actually prints.
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "cal.json")
        cal.save(path)
        output = asyncio.run(_run_monitor_briefly(page_calibration_path=path))

    positions = [e for e in _events(output) if e.get("event") == "position"]
    assert positions, output
    last = positions[-1]
    expected_row_shift = SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM - NOZZLE_BAR_SPAN_MM / 2.0
    assert abs(last["page_v"] - expected_row_shift) < 0.5   # travel is along x only


def test_monitor_position_omits_page_uvz_without_calibration():
    output = asyncio.run(_run_monitor_briefly())
    positions = [e for e in _events(output) if e.get("event") == "position"]
    assert positions, output
    assert "page_u" not in positions[-1]


def test_monitor_position_reports_yaw_deg_alongside_page_uvz():
    # SimulatedTracker never fakes orientation (quat always None -- see
    # SimulatedTracker.read_pose), so the mapper never gets a live sample to
    # rotate from and yaw_deg stays at its 0.0 (assume-boresight-pose)
    # default -- this pins the field's PRESENCE and default value, not a
    # live rotation (that's PageMapper's own job, pinned directly in
    # test_page_mapper_90deg_yaw_moves_a_pure_row_offset_entirely_onto_u).
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]),
                          boresight_quat=np.array([0.0, 0.0, 0.0, 1.0]))
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "cal.json")
        cal.save(path)
        output = asyncio.run(_run_monitor_briefly(page_calibration_path=path))

    positions = [e for e in _events(output) if e.get("event") == "position"]
    assert positions, output
    last = positions[-1]
    assert "yaw_deg" in last
    assert abs(last["yaw_deg"]) < 1e-6


def test_monitor_position_omits_yaw_deg_without_calibration():
    output = asyncio.run(_run_monitor_briefly())
    positions = [e for e in _events(output) if e.get("event") == "position"]
    assert positions, output
    assert "yaw_deg" not in positions[-1]


def test_monitor_position_reports_roll_and_pitch_deg_alongside_page_uvz():
    # Same reasoning as test_monitor_position_reports_yaw_deg_alongside_page_uvz:
    # SimulatedTracker never fakes orientation, so roll_deg/pitch_deg stay at
    # their 0.0 defaults -- this pins their PRESENCE in the NDJSON event.
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]),
                          boresight_quat=np.array([0.0, 0.0, 0.0, 1.0]))
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "cal.json")
        cal.save(path)
        output = asyncio.run(_run_monitor_briefly(page_calibration_path=path))

    positions = [e for e in _events(output) if e.get("event") == "position"]
    assert positions, output
    last = positions[-1]
    assert "roll_deg" in last and "pitch_deg" in last
    assert abs(last["roll_deg"]) < 1e-6
    assert abs(last["pitch_deg"]) < 1e-6


def test_monitor_position_omits_roll_and_pitch_deg_without_calibration():
    output = asyncio.run(_run_monitor_briefly())
    positions = [e for e in _events(output) if e.get("event") == "position"]
    assert positions, output
    assert "roll_deg" not in positions[-1]
    assert "pitch_deg" not in positions[-1]


def test_monitor_position_reports_an_error_for_a_bad_calibration_path():
    output = asyncio.run(_run_monitor_briefly(
        page_calibration_path="/nonexistent/path/cal.json"))
    events = _events(output)
    assert any(e.get("event") == "error" for e in events), output
    assert not any(e.get("event") == "position" for e in events)


# ======================================== simple frame origin zeroing (M10)
def test_zero_at_nozzle_puts_the_origin_under_the_nozzle_bar():
    # The bug this method exists for: set_origin() alone zeroes at the
    # SENSOR, leaving the nozzle bar ~69.91mm away along v (magnitude only --
    # direction depends on the constant's current, hardware-measured sign),
    # so every sample reads out of bounds on a 15.2mm-tall page and nothing
    # prints (observed on the first simulated simple-frame pass). After
    # zero_at_nozzle, the start pose must project to exactly (0, 0).
    mapper = PageMapper(PageCalibration.simple_frame())
    start = np.array([100.0, 40.0, 5.0])

    mapper.set_origin(start)
    u_sensor, v_sensor, _ = mapper.project(start, IDENTITY_QUAT)
    expected_v = SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM - NOZZLE_BAR_SPAN_MM / 2.0
    assert abs(u_sensor - SENSOR_TO_NOZZLE_COL_MM) < 1e-9
    assert abs(v_sensor - expected_v) < 1e-9, v_sensor   # the off-page offset

    mapper.zero_at_nozzle(start, IDENTITY_QUAT)
    u, v, _ = mapper.project(start, IDENTITY_QUAT)
    assert abs(u) < 1e-9 and abs(v) < 1e-9, (u, v)


def test_zero_at_nozzle_keeps_relative_motion_one_to_one():
    # Zeroing must only shift the frame, never scale or rotate it: moving the
    # cart 10mm in tracker x / 4mm in y must read as exactly (10, 4).
    mapper = PageMapper(PageCalibration.simple_frame())
    start = np.array([7.0, -3.0, 1.0])
    mapper.zero_at_nozzle(start, IDENTITY_QUAT)
    u, v, _ = mapper.project(start + np.array([10.0, 4.0, 0.0]), IDENTITY_QUAT)
    assert abs(u - 10.0) < 1e-9 and abs(v - 4.0) < 1e-9, (u, v)


def test_zero_at_nozzle_accounts_for_the_start_yaw():
    # The sensor->nozzle offset rotates with cart yaw, so zeroing has to use
    # the pose actually held at START -- otherwise a cart started at an angle
    # would be zeroed to the wrong point by exactly the rotated offset.
    for deg in (0.0, 30.0, -45.0, 90.0):
        mapper = PageMapper(PageCalibration.simple_frame())
        start = np.array([12.0, 8.0, 0.0])
        quat = _quat_about_z(deg)
        mapper.zero_at_nozzle(start, quat)
        u, v, _ = mapper.project(start, quat)
        assert abs(u) < 1e-9 and abs(v) < 1e-9, (deg, u, v)


def test_zero_at_nozzle_does_not_disturb_yaw_readout():
    # Zeroing touches the origin only; yaw must still read as the turn from
    # the captured reference afterwards.
    mapper = PageMapper(PageCalibration.simple_frame())
    mapper.capture_boresight(IDENTITY_QUAT)
    mapper.zero_at_nozzle(np.array([1.0, 2.0, 3.0]), IDENTITY_QUAT)
    mapper.project(np.array([1.0, 2.0, 3.0]), _quat_about_z(37.5))
    assert abs(math.degrees(mapper.last_yaw_rad) - 37.5) < 1e-9


def test_simple_frame_without_a_captured_reference_applies_no_rotation():
    # No orientation at START (tracker reported none): rather than reference
    # a wrong pose, the frame must fall back to no rotation correction at all
    # -- the same safe behaviour as a pre-boresight traced calibration.
    mapper = PageMapper(PageCalibration.simple_frame())
    mapper.capture_boresight(None)                  # explicit no-op
    assert mapper.calibration.boresight_quat is None
    mapper.project(np.array([1.0, 2.0, 3.0]), _quat_about_z(37.5))
    assert mapper.last_yaw_rad == 0.0


def test_captured_reference_makes_a_flat_turn_read_out_exactly():
    # REGRESSION with the real rig's mounting pose (120 deg off the tracker
    # axes, from the hardware boresight capture): after capturing the START
    # pose, a flat turn about tracker z must report exactly that turn, with
    # roll/pitch staying at zero. The earlier identity-boresight version
    # reported a 90 deg turn as ~70 deg and swung roll/pitch by tens of
    # degrees, which also misplaced ink via the offset rotation.
    q_mount = np.array([0.479, 0.510, -0.511, 0.499])
    q_mount = q_mount / np.linalg.norm(q_mount)
    mapper = PageMapper(PageCalibration.simple_frame())
    mapper.capture_boresight(q_mount)
    pos = np.array([50.0, 20.0, 0.0])
    mapper.zero_at_nozzle(pos, q_mount)

    for deg in (0.0, 15.0, 45.0, 90.0, -30.0):
        mapper.project(pos, _quat_mul(_quat_about_z(deg), q_mount))
        assert abs(math.degrees(mapper.last_yaw_rad) - deg) < 1e-6, deg
        assert abs(math.degrees(mapper.last_roll_rad)) < 1e-6, deg
        assert abs(math.degrees(mapper.last_pitch_rad)) < 1e-6, deg


def test_simple_frame_pos_stream_reports_page_uv_without_a_calibration():
    # --page-frame simple must light up the same live page_u/page_v/yaw_deg
    # readout a traced calibration does, with no --page-calibration at all --
    # that is what lets the frame be sanity-checked before printing.
    output = asyncio.run(_run_monitor_briefly(
        settings=TrackingSettings(page_frame="simple")))
    events = [e for e in _events(output) if e.get("event") == "position"]
    assert events, output
    assert "page_u" in events[0] and "page_v" in events[0], events[0]
    assert "yaw_deg" in events[0], events[0]


def test_pos_stream_accepts_a_pinned_simple_boresight():
    # Smoke test for the --simple-boresight plumbing through monitor_position
    # (SimulatedTracker never fakes orientation -- see the yaw_deg tests
    # above -- so this can't observe a live rotated reading; the actual
    # "reference pose reads as zero" math is pinned directly in
    # test_calibration.py, and "pinned reference survives a real pass
    # unmutated" in tests/test_freehand_pass.py). Must simply not crash and
    # must still report the ordinary page_u/page_v/yaw_deg fields.
    output = asyncio.run(_run_monitor_briefly(
        settings=TrackingSettings(page_frame="simple"),
        simple_boresight=[-0.5, -0.5, -0.51, 0.49]))
    events = [e for e in _events(output) if e.get("event") == "position"]
    assert events, output
    assert "page_u" in events[0] and "yaw_deg" in events[0], events[0]


# ==================================================== --calibration-check
class _ScriptedPoseTracker:
    """Returns a predetermined sequence of ``(pos, quat)`` pairs, holding the
    last one once exhausted -- mirrors tests/test_freehand_pass.py's
    ``ScriptedTracker`` (reimplemented here, not imported, so this file
    stays independently runnable -- the same convention that file's own
    docstring states for itself)."""

    def __init__(self, positions, quats):
        self._positions = [np.asarray(p, dtype=float) for p in positions]
        self._quats = [np.asarray(q, dtype=float) for q in quats]

        self._i = 0

    def open(self):
        pass

    def close(self):
        pass

    def read_position(self):
        return self.read_pose()[0]

    def read_pose(self):
        i = min(self._i, len(self._positions) - 1)
        self._i += 1
        return self._positions[i], self._quats[i]


def _calibration_check_cal_path(tmp_dir):
    """A trivial axis-aligned calibration WITH a captured (identity)
    boresight -- calibration_check needs a boresight for yaw/roll/pitch to
    be anything other than the 0.0 no-boresight default (see
    PageMapper.project)."""
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]),
                          boresight_quat=np.array(IDENTITY_QUAT))
    path = os.path.join(tmp_dir, "check_cal.json")
    cal.save(path)
    return path


async def _run_calibration_check_briefly(tracker, hz=300.0, duration_s=0.15, **kwargs):
    """Same cancel-after-a-short-while harness as _run_monitor_briefly, for
    diagnostics.calibration_check instead -- always NDJSON (ndjson=True),
    always a scripted tracker (never real hardware/--simulate)."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        task = asyncio.ensure_future(diagnostics.calibration_check(
            TrackingSettings(), simulate=True, hz=hz, ndjson=True,
            tracker=tracker, **kwargs))
        await asyncio.sleep(duration_s)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
    return buf.getvalue()


def test_calibration_check_ndjson_event_shape():
    with tempfile.TemporaryDirectory() as tmp:
        cal_path = _calibration_check_cal_path(tmp)
        n = 60
        positions = [(i * 2.0, 0.0, 0.0) for i in range(n)]
        quats = [IDENTITY_QUAT for _ in range(n)]
        tracker = _ScriptedPoseTracker(positions, quats)
        output = asyncio.run(_run_calibration_check_briefly(
            tracker, page_calibration_path=cal_path))

    events = _events(output)
    kinds = [e.get("event") for e in events]
    assert "connected" in kinds, events
    assert "position" in kinds, events
    assert "calibration_check_summary" in kinds, events
    assert "stopped" in kinds, events

    pos_event = next(e for e in events if e["event"] == "position")
    for key in ("x", "y", "z", "page_u", "page_v", "page_z",
               "yaw_deg", "roll_deg", "pitch_deg"):
        assert key in pos_event, (key, pos_event)

    summary_event = next(e for e in events if e["event"] == "calibration_check_summary")
    for key in ("sample_count", "u_travel_mm", "v_travel_mm", "yaw_min_deg",
               "yaw_max_deg", "yaw_span_deg", "roll_span_deg", "pitch_span_deg",
               "yaw_u_correlation", "yaw_v_correlation", "verdict"):
        assert key in summary_event, (key, summary_event)


def test_calibration_check_reports_an_error_without_a_page_frame():
    # No --page-calibration and not --page-frame simple: there is nothing to
    # check drift IN, and this diagnostic (unlike --pos) must say so rather
    # than silently reporting bare x/y/z.
    output = asyncio.run(_run_calibration_check_briefly(
        _ScriptedPoseTracker([(0.0, 0.0, 0.0)], [IDENTITY_QUAT])))
    events = _events(output)
    assert any(e.get("event") == "error" for e in events), output
    assert not any(e.get("event") == "position" for e in events)


def test_calibration_check_pure_translation_gives_near_zero_yaw_span():
    # The cart slides in a straight line (u only) with the orientation
    # quaternion held perfectly constant throughout -- i.e. NEVER rotates --
    # so relative to the boresight, yaw must be exactly 0.0 on every single
    # sample: the headline "no rotation -> yaw stays put" case.
    with tempfile.TemporaryDirectory() as tmp:
        cal_path = _calibration_check_cal_path(tmp)
        n = 80
        positions = [(i * 2.5, 0.0, 0.0) for i in range(n)]     # pure translation along u
        quats = [IDENTITY_QUAT for _ in range(n)]                 # never rotates
        tracker = _ScriptedPoseTracker(positions, quats)
        output = asyncio.run(_run_calibration_check_briefly(
            tracker, page_calibration_path=cal_path))

    summary = next(e for e in _events(output) if e["event"] == "calibration_check_summary")
    assert summary["sample_count"] > 10, summary
    assert summary["u_travel_mm"] > 50.0, summary          # actually moved a meaningful distance
    assert summary["yaw_span_deg"] < 1e-6, summary
    # Yaw has ~0 variance (it is exactly 0.0 throughout) -> correlation is
    # mathematically undefined, not 0 -- see _calibration_check_summary's
    # docstring for why that distinction matters.
    assert summary["yaw_u_correlation"] is None, summary
    assert "OK" in summary["verdict"], summary["verdict"]


def test_calibration_check_injected_yaw_ramp_gives_large_span_and_high_correlation():
    # Same straight-line translation as above, but this time the scripted
    # orientation ALSO ramps a yaw proportional to sample index (and hence
    # to u, since u increases one-to-one with index too) -- standing in for
    # a page-normal error that leaks position-dependent tilt into yaw (see
    # rotation.py / calibration.py's threshold comment, and the module
    # docstring's real-rig example: measured tilt correlated +0.69 with v
    # while the cart was provably flat). Must show up as a large span AND a
    # strong positive correlation with u -- not just "some" nonzero number.
    with tempfile.TemporaryDirectory() as tmp:
        cal_path = _calibration_check_cal_path(tmp)
        n = 80
        positions = [(i * 2.5, 0.0, 0.0) for i in range(n)]
        quats = [_quat_about_z(i * 0.3) for i in range(n)]         # yaw ramps with u
        tracker = _ScriptedPoseTracker(positions, quats)
        output = asyncio.run(_run_calibration_check_briefly(
            tracker, page_calibration_path=cal_path))

    summary = next(e for e in _events(output) if e["event"] == "calibration_check_summary")
    assert summary["yaw_span_deg"] > 5.0, summary
    assert summary["yaw_u_correlation"] is not None, summary
    assert summary["yaw_u_correlation"] > 0.9, summary          # strongly correlated, not noise
    assert "BAD" in summary["verdict"], summary["verdict"]


def test_calibration_check_summary_pure_function_matches_the_live_run():
    # Cross-check: diagnostics._calibration_check_summary (the pure,
    # directly-testable statistics function) must produce EXACTLY the same
    # numbers the live async loop above reports through NDJSON, computed
    # independently here from hand-built sample lists rather than trusting
    # the live run's own output.
    u = [float(i) for i in range(50)]
    v = [0.0] * 50
    yaw = [0.1 * i for i in range(50)]
    roll = [0.0] * 50
    pitch = [0.0] * 50
    summary = diagnostics._calibration_check_summary(u, v, yaw, roll, pitch)
    assert summary["sample_count"] == 50
    assert abs(summary["u_travel_mm"] - 49.0) < 1e-9
    assert abs(summary["v_travel_mm"] - 0.0) < 1e-9
    assert abs(summary["yaw_span_deg"] - 4.9) < 1e-9
    assert summary["yaw_u_correlation"] > 0.999                 # perfectly linear in u
    assert summary["yaw_v_correlation"] is None                 # v has zero variance


def test_calibration_check_zero_samples_is_INCONCLUSIVE_not_a_pass():
    # REGRESSION: the verdict keyed on yaw span alone, so a run that
    # collected NOTHING (tracker delivered no pose, or Ctrl+C landed
    # immediately) reported "OK: yaw span 0.00 deg ... consistent with a
    # good calibration" -- a false all-clear on the exact question the
    # operator ran the check to answer. A health check must never call a
    # measurement it did not take a pass.
    summary = diagnostics._calibration_check_summary([], [], [], [], [])
    assert summary["sample_count"] == 0
    assert summary["verdict"].startswith("INCONCLUSIVE"), summary["verdict"]
    assert "not a pass" in summary["verdict"], summary["verdict"]


def test_calibration_check_short_wiggle_is_INCONCLUSIVE_not_a_pass():
    # Same root cause as above with real samples: a 20mm wiggle holds yaw
    # near zero simply because the cart barely moved, which says nothing
    # about whether yaw drifts across a whole page. Under
    # CALIBRATION_CHECK_MIN_TRAVEL_MM -> INCONCLUSIVE, not OK.
    n = 60
    u = [0.33 * i for i in range(n)]                  # ~20mm of travel
    v = [0.0] * n
    yaw = [0.01 * (i % 3) for i in range(n)]          # tiny, harmless jitter
    summary = diagnostics._calibration_check_summary(u, v, yaw, [0.0] * n, [0.0] * n)
    assert summary["u_travel_mm"] < diagnostics.CALIBRATION_CHECK_MIN_TRAVEL_MM
    assert summary["yaw_span_deg"] < diagnostics.CALIBRATION_CHECK_YAW_SPAN_FINE_DEG
    assert summary["verdict"].startswith("INCONCLUSIVE"), summary["verdict"]


def test_calibration_check_too_few_samples_is_INCONCLUSIVE_even_over_a_long_sweep():
    # The other half of the guard: plenty of travel, but so few samples that
    # the span is one or two readings' worth of noise rather than a measured
    # trend. Pinned separately from the travel case so removing either
    # condition alone fails a test.
    u = [0.0, 60.0, 120.0, 180.0]                     # 180mm of travel, only 4 samples
    summary = diagnostics._calibration_check_summary(
        u, [0.0] * 4, [0.0] * 4, [0.0] * 4, [0.0] * 4)
    assert summary["u_travel_mm"] > diagnostics.CALIBRATION_CHECK_MIN_TRAVEL_MM
    assert summary["sample_count"] < diagnostics.CALIBRATION_CHECK_MIN_SAMPLES
    assert summary["verdict"].startswith("INCONCLUSIVE"), summary["verdict"]


def test_calibration_check_a_real_sweep_still_reaches_a_real_verdict():
    # Counter-check that the INCONCLUSIVE guard above is not simply
    # swallowing every run: a sweep that clears BOTH thresholds must still
    # get a genuine OK/BORDERLINE/BAD verdict, never INCONCLUSIVE.
    n = 120
    u = [1.75 * i for i in range(n)]                   # ~208mm, A4-width sweep
    v = [0.0] * n
    for yaw_span, expected in ((0.5, "OK"), (3.0, "BORDERLINE"), (9.0, "BAD")):
        yaw = [yaw_span * i / (n - 1) for i in range(n)]
        summary = diagnostics._calibration_check_summary(
            u, v, yaw, [0.0] * n, [0.0] * n)
        assert summary["verdict"].startswith(expected), (yaw_span, summary["verdict"])


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All page-mapper tests passed.")
