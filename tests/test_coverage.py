"""
Freehand coverage engine tests (no hardware): mirrors the scenarios the
firmware's tests/test_column_fifo.c already proved out for the 1D dose
pipeline, generalised to CoverageEngine's per-nozzle, per-pixel model.

Run with:  python tests/test_coverage.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.coverage import CoverageEngine, DEFAULT_DOSE_HOLD_S     # noqa: E402
from printhead.geometry import (                                       # noqa: E402
    NOZZLE_BAR_WIDTH_MM, NOZZLE_PITCH_MM, NUM_NOZZLES, ROW_BYTES,
)

DOSE_HOLD_S = 0.1


def _unpack(pattern: bytes) -> np.ndarray:
    return np.unpackbits(np.frombuffer(pattern, dtype=np.uint8), bitorder="little")


# ================================================== measured dose-hold tuning
def test_default_dose_hold_tracks_the_firmware_pattern_stride():
    # DEFAULT_DOSE_HOLD_S is derived from -- and MUST stay in sync with --
    # the firmware's PATTERN_STRIDE (src/ble_dose.h in the firmware repo):
    # DOSE_HOLD_S ~= 3 * PATTERN_STRIDE * 450e-6 (3 = BLE_DROPS_PER_COLUMN,
    # 450us = the firmware print loop tick, PATTERN_STRIDE = 3). If someone
    # changes DEFAULT_DOSE_HOLD_S here without changing PATTERN_STRIDE (and
    # re-flashing) to match, or vice versa, the ~3-drop-per-pixel target this
    # pair was tuned to breaks silently on hardware -- this test is the loud
    # failure meant to catch that on the client side.
    #
    # CORRECTION: an earlier pick (PATTERN_STRIDE=4, DOSE_HOLD_S=0.0054 s)
    # also hit this ~3-drop target but landed just ABOVE the 5.00 ms poll
    # interval of the default --poll-hz 200, which requires a third sample
    # to land on the same column to complete a dose -- measured coverage
    # collapsed from 100% (at 4.90 ms) to 31% (at 5.40 ms). The additional
    # constraint that adds, on top of the 3-drop target: DOSE_HOLD_S must
    # stay below 1/poll_hz (the poll interval), or two consecutive samples
    # are not enough to complete a dose. 0.00405 s is 19% below the 5.00 ms
    # default poll interval.
    firmware_pattern_stride = 3
    drops_per_column = 3          # BLE_DROPS_PER_COLUMN, line mode's dose target
    tick_s = 450e-6                # firmware print loop tick
    expected = drops_per_column * firmware_pattern_stride * tick_s
    assert abs(DEFAULT_DOSE_HOLD_S - expected) < 1e-6, (
        f"DEFAULT_DOSE_HOLD_S={DEFAULT_DOSE_HOLD_S} no longer matches "
        f"3 * PATTERN_STRIDE({firmware_pattern_stride}) * 450us = {expected} "
        "-- update the firmware's PATTERN_STRIDE (src/ble_dose.h) to match, "
        "or this comment/test, and re-flash the firmware")

    # The additional poll-interval constraint above, pinned directly: with
    # the default poll_hz=200 (5.00 ms interval), the hold must stay below
    # it or coverage collapses (see coverage.py's DEFAULT_DOSE_HOLD_S
    # comment for the measured cliff).
    default_poll_hz = 200.0
    assert DEFAULT_DOSE_HOLD_S < 1.0 / default_poll_hz, (
        f"DEFAULT_DOSE_HOLD_S={DEFAULT_DOSE_HOLD_S} is not below the "
        f"{1.0 / default_poll_hz}s poll interval at poll_hz={default_poll_hz} "
        "-- this is the quantization cliff that made the previous 0.0054s "
        "value collapse coverage to ~31%")


def test_realistic_median_dwell_completes_with_the_new_default():
    # 0.2 mm column at the measured median hand speed (17.3 mm/s) dwells for
    # about 11.6 ms -- comfortably above the new 4.05 ms default, so the
    # pixel must be marked printed.
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DEFAULT_DOSE_HOLD_S)
    median_dwell_s = 0.2 / 17.3

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=median_dwell_s)
    assert eng.printed[0, 0]


def test_short_dwell_above_49mms_stays_unprinted_with_the_new_default():
    # A 3 ms dwell corresponds to roughly 0.2/0.003 ~= 67 mm/s, well above
    # the ~49 mm/s point (0.2 / 0.00405) where the new default hold no
    # longer fits inside one column's crossing time -- the pixel must stay
    # open for a later pass rather than being marked printed early.
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DEFAULT_DOSE_HOLD_S)
    short_dwell_s = 0.003

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=short_dwell_s)
    assert not eng.printed[0, 0]


# ============================================================== basic dosing
def test_single_pass_completes_after_dose_hold_elapses():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)               # first touch
    assert not eng.printed[0, 0]                       # not yet dosed
    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.05)  # dwelled long enough
    assert eng.printed[0, 0]


def test_nozzle_stops_firing_once_its_pixel_is_printed():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.05)
    assert _unpack(pattern)[0] == 1          # fires through the completing sample
    pattern, changed = eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.1)
    assert _unpack(pattern)[0] == 0          # then stops
    assert changed


def test_revisit_does_not_refire_an_already_printed_pixel():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.05)   # (0,0) printed
    assert eng.printed[0, 0]

    eng.step(u_mm=3.0, v_mm=0.0, t=0.5)                  # move away
    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.5 + DOSE_HOLD_S + 0.5)  # revisit, dwell plenty
    assert _unpack(pattern)[0] == 0                      # never refires
    assert eng.printed[0, 0]                             # stays printed


def test_loop_with_a_longer_second_pass_achieves_coverage():
    # First pass is too fast (dwell < threshold) -> not printed. A slower
    # "loop back" pass completes it. Models "keep going in circles until it
    # takes" rather than a single continuous dwell.
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S * 0.5)     # left too early
    assert not eng.printed[0, 0]

    eng.step(u_mm=3.0, v_mm=0.0, t=1.0)                   # loop away
    eng.step(u_mm=0.0, v_mm=0.0, t=2.0)                   # loop back
    eng.step(u_mm=0.0, v_mm=0.0, t=2.0 + DOSE_HOLD_S + 0.05)  # this time it dwells
    assert eng.printed[0, 0]


def test_completion_depends_on_wall_clock_dwell_not_sample_count():
    # Many samples packed into a tiny time window must not fool the engine
    # into completing the dose early -- only elapsed time matters.
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    for i in range(50):
        eng.step(u_mm=0.0, v_mm=0.0, t=0.001 * i)          # 50 samples, 49ms span
    assert not eng.printed[0, 0]

    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.2)       # real dwell time passes
    assert eng.printed[0, 0]


def test_dose_does_not_accumulate_across_different_pixels():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)                      # pixel (0,0)
    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S * 0.5)         # 50% dwelled
    eng.step(u_mm=1.0, v_mm=0.0, t=DOSE_HOLD_S * 0.5)         # pixel (0,1)
    eng.step(u_mm=1.0, v_mm=0.0, t=DOSE_HOLD_S * 1.0)         # 50% dwelled

    assert not eng.printed[0, 0]
    assert not eng.printed[0, 1]


def test_ink_not_requested_never_fires_or_prints():
    ink = np.zeros((10, 5), dtype=bool)          # nothing wanted anywhere
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    pattern, changed = eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    assert not _unpack(pattern).any()
    assert changed                                # first call always reports a change
    pattern, changed = eng.step(u_mm=0.0, v_mm=0.0, t=1.0)
    assert not changed                            # still all-zero -> no change this time
    eng.step(u_mm=0.0, v_mm=0.0, t=10.0)
    assert not eng.printed[0, 0]


# ============================================================== bar geometry
def test_multiple_nozzles_fire_together_when_all_wanted():
    ink = np.ones((NUM_NOZZLES, 1), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)
    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    assert _unpack(pattern)[:NUM_NOZZLES].all()


def test_tall_image_needs_vertical_travel_to_reach_all_rows():
    height = NUM_NOZZLES + 60
    ink = np.ones((height, 1), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    # base_row=0 -> nozzle bar spans rows [0, NUM_NOZZLES). Dwell long enough.
    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.05)
    assert eng.printed[0, 0]
    assert eng.printed[NUM_NOZZLES - 1, 0]
    assert not eng.printed[NUM_NOZZLES, 0]           # just out of the bar's reach
    assert not eng.printed[height - 1, 0]            # far out of reach

    # Shift vertically so the bar's top edge now reaches the last row.
    v_shift_mm = (height - NUM_NOZZLES) * NOZZLE_PITCH_MM
    eng.step(u_mm=0.0, v_mm=v_shift_mm, t=1.0)
    eng.step(u_mm=0.0, v_mm=v_shift_mm, t=1.0 + DOSE_HOLD_S + 0.05)
    assert eng.printed[height - 1, 0]


def test_out_of_bounds_position_does_not_crash_or_print_anything():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)

    # Far off the image in both u and v.
    pattern, _ = eng.step(u_mm=1000.0, v_mm=1000.0, t=0.0)
    eng.step(u_mm=1000.0, v_mm=1000.0, t=10.0)
    assert not _unpack(pattern).any()
    assert not eng.printed.any()

    # Negative column.
    eng.step(u_mm=-5.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=-5.0, v_mm=0.0, t=10.0)
    assert not eng.printed.any()


# ================================================================= bookkeeping
def test_pattern_is_row_bytes_long():
    ink = np.ones((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)
    pattern, _ = eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    assert len(pattern) == ROW_BYTES


def test_done_property_tracks_full_coverage():
    ink = np.ones((3, 1), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)
    assert not eng.done

    eng.step(u_mm=0.0, v_mm=0.0, t=0.0)
    eng.step(u_mm=0.0, v_mm=0.0, t=DOSE_HOLD_S + 0.05)
    assert eng.done                 # bar (152 nozzles) covers all 3 rows at once


def test_done_is_true_for_a_blank_target():
    ink = np.zeros((10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=DOSE_HOLD_S)
    assert eng.done                 # nothing wanted -> trivially done


# ============================================== yaw / cart-rotation correction
def test_step_default_yaw_rad_reduces_exactly_to_the_pre_rotation_behaviour():
    # Regression guard (every test above depends on this): calling step()
    # without yaw_rad at all must place every nozzle at the same column and
    # at row = base_row + p, exactly as before this feature existed.
    ink = np.ones((NUM_NOZZLES + 10, 5), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=0.0)
    eng.step(u_mm=2.0, v_mm=0.0, t=0.0)
    assert eng.printed[0:NUM_NOZZLES, 2].all()
    assert not eng.printed[:, [0, 1, 3, 4]].any()


def test_step_zero_yaw_is_bit_identical_across_a_sweep_of_v_offsets_incl_rounding_boundaries():
    # Floating-point drift guard: recomputing row_p as
    # round((v_mm + p*NOZZLE_PITCH_MM) / NOZZLE_PITCH_MM) instead of the
    # exact integer base_row + p measurably drifts by one nozzle right at
    # rounding boundaries (empirically confirmed while building this fix --
    # about half of exact .5-boundary v_mm values disagree). Sweep including
    # those boundaries and require BIT-IDENTICAL row placement between
    # explicit yaw_rad=0.0 and the pre-rotation base_row + p formula.
    height = NUM_NOZZLES + 5
    for k in range(-50, 50):
        v_mm = (k + 0.5) * NOZZLE_PITCH_MM        # exact rounding boundary
        ink = np.ones((height, 3), dtype=bool)
        eng = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=0.0)
        eng.step(u_mm=1.0, v_mm=v_mm, t=0.0, yaw_rad=0.0)
        base_row = int(round(v_mm / NOZZLE_PITCH_MM))
        expected_rows = {r for r in range(base_row, base_row + NUM_NOZZLES) if 0 <= r < height}
        actual_rows = set(np.nonzero(eng.printed[:, 1])[0].tolist())
        assert actual_rows == expected_rows, (v_mm, base_row, actual_rows, expected_rows)


def test_step_nonzero_yaw_spreads_nozzles_across_columns_by_the_expected_amount():
    mm_per_column = 0.5
    width_cols = 200
    height_rows = 300
    ink = np.ones((height_rows, width_cols), dtype=bool)
    eng = CoverageEngine(ink, mm_per_column=mm_per_column, dose_hold_s=0.0)
    yaw = math.radians(45.0)

    eng.step(u_mm=50.0, v_mm=5.0, t=0.0, yaw_rad=yaw)

    touched_cols = np.nonzero(eng.printed.any(axis=0))[0]
    assert touched_cols.size > 1, "a nonzero yaw must spread ink across more than one column"
    spread_cols = int(touched_cols.max() - touched_cols.min())

    # bar_length == (NUM_NOZZLES - 1) * NOZZLE_PITCH_MM == NOZZLE_BAR_WIDTH_MM
    # exactly (see geometry.py) -- the u-extent across the whole bar is
    # bar_length * sin(yaw_rad), converted to columns.
    expected_spread_cols = NOZZLE_BAR_WIDTH_MM * math.sin(yaw) / mm_per_column
    assert abs(spread_cols - expected_spread_cols) <= 1.0, (spread_cols, expected_spread_cols)


def test_step_yaw_sign_flips_the_column_spread_direction():
    mm_per_column = 0.5
    ink = np.ones((300, 200), dtype=bool)
    yaw = math.radians(30.0)

    eng_pos = CoverageEngine(ink, mm_per_column=mm_per_column, dose_hold_s=0.0)
    eng_pos.step(u_mm=50.0, v_mm=5.0, t=0.0, yaw_rad=yaw)
    cols_pos = np.nonzero(eng_pos.printed.any(axis=0))[0]

    eng_neg = CoverageEngine(ink, mm_per_column=mm_per_column, dose_hold_s=0.0)
    eng_neg.step(u_mm=50.0, v_mm=5.0, t=0.0, yaw_rad=-yaw)
    cols_neg = np.nonzero(eng_neg.printed.any(axis=0))[0]

    # Opposite yaw signs must spread to opposite sides of nozzle 0's column
    # (u_mm=50 -> col 100): +yaw spreads to higher columns, -yaw to lower.
    assert cols_pos.max() > 100 and cols_neg.min() < 100
    assert cols_pos.min() >= 100 and cols_neg.max() <= 100


# =============================================== mutation check (see PR body)
def test_step_MUTATION_check_ignoring_yaw_rad_breaks_the_spread_test():
    # Inlines the mutation described in the PR: a step() that accepts
    # yaw_rad but never uses it (drops back to the single shared `col`)
    # would place every nozzle in the same column regardless of yaw --
    # reproduced here directly rather than by editing coverage.py, so the
    # regression stays covered by the test suite.
    mm_per_column = 0.5
    ink = np.ones((300, 200), dtype=bool)
    yaw = math.radians(45.0)
    u_mm, v_mm = 50.0, 5.0

    col_ignoring_yaw = int(round(u_mm / mm_per_column))
    mutated_touched_cols = {col_ignoring_yaw}          # every nozzle, same column

    eng = CoverageEngine(ink, mm_per_column=mm_per_column, dose_hold_s=0.0)
    eng.step(u_mm=u_mm, v_mm=v_mm, t=0.0, yaw_rad=yaw)
    real_touched_cols = set(np.nonzero(eng.printed.any(axis=0))[0].tolist())

    assert real_touched_cols != mutated_touched_cols, (
        "the mutated (yaw-ignoring) engine must disagree with the real one "
        "-- if this ever matches, the spread test above has stopped "
        "actually exercising yaw_rad")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All coverage tests passed.")
