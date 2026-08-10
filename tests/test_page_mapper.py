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
    NOZZLE_BAR_WIDTH_MM,
    SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM,
    SENSOR_TO_NOZZLE_COL_MM,
)
from printhead.tracking import PageMapper                    # noqa: E402

IDENTITY_QUAT = (0.0, 0.0, 0.0, 1.0)


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
# NOTE: PageMapper's row axis always subtracts NOZZLE_BAR_WIDTH_MM/2 from
# whatever sensor_offset_row_mm is given -- that subtraction is the measured-
# bar-CENTRE-to-nozzle-0 conversion, not "no correction". Passing literal 0.0
# does NOT cancel to a zero net shift; it means "the bar centre is exactly at
# the sensor", which still shifts v by -NOZZLE_BAR_WIDTH_MM/2 (nozzle 0 sits
# that far from the centre). The value that actually cancels to zero net
# shift is NOZZLE_BAR_WIDTH_MM/2.0 (see
# test_page_mapper_explicit_offset_matches_pre_offset_behaviour below, which
# pins this distinction directly).
_NEUTRAL_ROW_OFFSET = NOZZLE_BAR_WIDTH_MM / 2.0


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
    # the measured bar-CENTRE offset minus half the bar width (converting to
    # the nozzle-0-referenced v CoverageEngine needs); default col shift is
    # the raw column offset (no bar-width correction on that axis).
    cal = PageCalibration(origin=np.array([1.0, 2.0, 3.0]),
                          e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    pos = np.array([11.0, 7.0, 3.0])
    u_raw, v_raw, z_raw = cal.project(pos)

    mapper = PageMapper(cal)     # default offsets
    u, v, z = mapper.project(pos)

    expected_row_shift = SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM - NOZZLE_BAR_WIDTH_MM / 2.0
    assert abs(expected_row_shift - 54.86) < 1e-9    # 62.36 - 15.0/2 = 54.86
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
    # NOZZLE_BAR_WIDTH_MM/2 from whatever row value is given (the measured-
    # bar-CENTRE-to-nozzle-0 conversion), so 0.0 in still leaves a
    # -NOZZLE_BAR_WIDTH_MM/2 mm net shift, NOT zero -- literal (0.0, 0.0)
    # is checked explicitly below to pin exactly that (real, if easy to miss)
    # gotcha. The value that actually cancels the row conversion to zero net
    # shift is NOZZLE_BAR_WIDTH_MM/2.0 (col has no such conversion, so 0.0 is
    # already neutral there).
    cal = PageCalibration(origin=np.array([1.0, 2.0, 3.0]),
                          e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    pos = np.array([11.0, 7.0, 3.0])
    mapper = PageMapper(cal, sensor_offset_row_mm=NOZZLE_BAR_WIDTH_MM / 2.0,
                        sensor_offset_col_mm=0.0)
    assert mapper.project(pos) == cal.project(pos)


def test_page_mapper_literal_zero_row_offset_is_not_neutral():
    # The gotcha explained above, pinned directly: sensor_offset_row_mm=0.0
    # is a real, physically meaningful value (bar centre coincident with the
    # sensor), not an "off switch" -- it still shifts v by -NOZZLE_BAR_WIDTH_MM/2.
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    pos = np.array([3.0, 4.0, 0.0])
    _, v_raw, _ = cal.project(pos)
    mapper = PageMapper(cal, sensor_offset_row_mm=0.0, sensor_offset_col_mm=0.0)
    _, v, _ = mapper.project(pos)
    assert abs(v - (v_raw - NOZZLE_BAR_WIDTH_MM / 2.0)) < 1e-9
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
    assert abs(shift_pos - (62.36 - NOZZLE_BAR_WIDTH_MM / 2.0)) < 1e-9
    assert abs(shift_neg - (-62.36 - NOZZLE_BAR_WIDTH_MM / 2.0)) < 1e-9
    # Negating the measured value must not simply negate the applied shift
    # (the -bar_width/2 term is a fixed conversion constant, not part of the
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
    expected_v = 12.0 + (SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM - NOZZLE_BAR_WIDTH_MM / 2.0)
    expected_z = 5.0
    assert abs(u - expected_u) < 1e-9
    assert abs(v - expected_v) < 1e-9    # 12 + 54.86 = 66.86
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
    # PageMapper's constructor always subtracts NOZZLE_BAR_WIDTH_MM/2 from
    # the given sensor_offset_row_mm (bar-centre -> nozzle-0 conversion, see
    # its docstring) -- add it back so the NET row offset is exactly 10.0mm,
    # col offset exactly 0.0mm, i.e. a "pure row offset" in the sense this
    # test needs.
    mapper = PageMapper(cal, sensor_offset_row_mm=row_offset_mm + NOZZLE_BAR_WIDTH_MM / 2.0,
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
    mapper = PageMapper(cal, sensor_offset_row_mm=NOZZLE_BAR_WIDTH_MM / 2.0 + 10.0,
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
    v_expected += 70.0 - NOZZLE_BAR_WIDTH_MM / 2.0
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

    mapper = PageMapper(cal, sensor_offset_row_mm=row_offset_mm + NOZZLE_BAR_WIDTH_MM / 2.0,
                        sensor_offset_col_mm=0.0)
    correct_u, correct_v, _ = mapper.project(np.zeros(3), quat=_quat_about_z(90.0))

    assert (mutated_u, mutated_v) != (correct_u, correct_v), (
        "the mutated (no-rotation) formula must disagree with the real, "
        "rotation-aware one at 90 degrees -- if this ever matches, the "
        "90-degree test above has stopped actually exercising the rotation")


# ===================================================== --pos / monitor_position
async def _run_monitor_briefly(**kwargs):
    """Run monitor_position(simulate=True, ndjson=True, ...) for a short while,
    then cancel it (like Ctrl+C) and return everything it printed."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        task = asyncio.ensure_future(diagnostics.monitor_position(
            TrackingSettings(), simulate=True, hz=50.0, ndjson=True, **kwargs))
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
    # through as page_u. Sensor row offset neutralised (NOZZLE_BAR_WIDTH_MM/2,
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
            sensor_offset_row_mm=NOZZLE_BAR_WIDTH_MM / 2.0,
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
    expected_row_shift = SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM - NOZZLE_BAR_WIDTH_MM / 2.0
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


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All page-mapper tests passed.")
