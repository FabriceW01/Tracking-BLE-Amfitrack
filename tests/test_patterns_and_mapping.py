"""
Pattern generation and nozzle-remapping tests (no hardware).

Run with:  python tests/test_patterns_and_mapping.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead import cli, patterns                         # noqa: E402
from printhead.geometry import IMAGE_HEIGHT, NOZZLE_PITCH_MM  # noqa: E402
from printhead.nozzle_map import parse_order, remap_rows     # noqa: E402
from printhead.rendering import frames_from_ink              # noqa: E402

# All six generators that must honour an explicit rows= override (Change 2):
# the five PATTERNS presets plus ruler_pattern, which isn't in that dict
# because --calibrate reaches it directly rather than through --pattern NAME.
_ALL_GENERATORS = list(patterns.PATTERNS.values()) + [patterns.ruler_pattern]


# ============================================================================
# ruler_pattern
# ============================================================================
def test_ruler_pattern_ticks():
    mm_per_column = 0.1                      # 10 columns/mm -> easy to reason about
    ink = patterns.ruler_pattern(30.0, mm_per_column, major_every_mm=10.0,
                                 minor_every_mm=1.0)
    assert ink.shape == (IMAGE_HEIGHT, 300)
    mid = IMAGE_HEIGHT // 2

    assert ink[mid, :].all(), "baseline row must be continuous across the width"

    # Major ticks (every 10mm = every 100 columns) span the full height.
    for col in (0, 100, 200):
        assert ink[:, col].all(), f"col {col} should be a full-height major tick"

    # A minor-only column (e.g. 1mm = col 10) is not full height, but is taller
    # than just the baseline.
    minor_col = ink[:, 10]
    assert minor_col[mid]
    assert not minor_col.all()
    assert minor_col.sum() > 1

    # A column between ticks (e.g. col 5, half a mm) is baseline-only.
    plain_col = ink[:, 5]
    assert plain_col.sum() == 1 and plain_col[mid]


# ============================================================================
# preset patterns
# ============================================================================
def test_pattern_shapes_and_framing():
    mm_per_column = 0.2
    for name, fn in patterns.PATTERNS.items():
        ink = fn(20.0, mm_per_column, square_mm=5.0, square_rows=10)
        assert ink.dtype == bool, name
        assert ink.shape[0] == IMAGE_HEIGHT, name
        assert ink.shape[1] > 0, name
        frames = frames_from_ink(ink)          # must not raise
        assert len(frames) == ink.shape[1], name


def test_checkerboard_alternates():
    ink = patterns.checkerboard_pattern(10.0, 1.0, square_mm=2.0, square_rows=3)
    # Adjacent tiles across a column boundary (col 1 -> col 2) must differ.
    assert bool(ink[0, 1]) != bool(ink[0, 2])
    # Adjacent tiles across a row boundary (row 2 -> row 3) must differ.
    assert bool(ink[2, 0]) != bool(ink[3, 0])


def test_solid_and_stripes_are_nonempty():
    solid = patterns.solid_pattern(10.0, 1.0)
    assert solid.all()
    h_stripes = patterns.h_stripes_pattern(10.0, 1.0, square_rows=10)
    assert h_stripes.any() and not h_stripes.all()
    v_stripes = patterns.v_stripes_pattern(10.0, 1.0, square_mm=2.0)
    assert v_stripes.any() and not v_stripes.all()


# ============================================================================
# nozzle_map
# ============================================================================
def test_parse_order_valid_and_invalid():
    assert parse_order("2,3,4,1,5", 5) == [1, 2, 3, 0, 4]

    for bad, block in [("2,3,4,1", 5), ("2,3,4,1,1", 5), ("a,b,c", 3)]:
        try:
            parse_order(bad, block)
            assert False, f"expected ValueError for {bad!r}"
        except ValueError:
            pass


def test_remap_rows_permutation():
    # 10 rows, 1 column, each row uniquely identified by its own boolean marker
    # so a permutation is easy to detect by which row ends up where.
    ink = np.zeros((10, 1), dtype=bool)
    order = parse_order("2,3,4,1,5", 5)        # -> [1, 2, 3, 0, 4]

    for src_row in range(10):
        probe = np.zeros((10, 1), dtype=bool)
        probe[src_row, 0] = True
        out = remap_rows(probe, block_size=5, order=order)
        # Find where the marker ended up.
        (dst_row,) = np.nonzero(out[:, 0])[0]
        block, i = divmod(src_row, 5)
        expected_dst = None
        # new[block*5 + k] = old[block*5 + order[k]]; find k such that order[k] == i
        for k, src in enumerate(order):
            if src == i:
                expected_dst = block * 5 + k
        assert dst_row == expected_dst, (src_row, dst_row, expected_dst)


def test_remap_rows_partial_trailing_block_unchanged():
    order = parse_order("2,3,4,1,5", 5)
    ink = np.eye(7, dtype=bool)                # height 7, block_size 5 -> 2 leftover rows
    out = remap_rows(ink, block_size=5, order=order)
    # Rows 5 and 6 (the trailing partial block) are left untouched (identity).
    assert np.array_equal(out[5], ink[5])
    assert np.array_equal(out[6], ink[6])
    # The full first block was remapped (not identity).
    assert not np.array_equal(out[0:5], ink[0:5])


# ============================================================================
# CLI validation
# ============================================================================
def test_cli_requires_a_content_source():
    try:
        cli.parse_args([])
        assert False, "expected SystemExit"
    except SystemExit:
        pass


def test_cli_rejects_multiple_content_sources():
    try:
        cli.parse_args(["Hi", "--calibrate"])
        assert False, "expected SystemExit"
    except SystemExit:
        pass


def test_cli_requires_nozzle_order_with_block_size():
    try:
        cli.parse_args(["Hi", "--dry-run", "--nozzle-block-size", "5"])
        assert False, "expected SystemExit"
    except SystemExit:
        pass


def test_cli_accepts_calibrate_and_pattern():
    # --mode line: this test is about --calibrate/--pattern acceptance, not
    # mode selection, and neither call supplies a --page-calibration.
    args = cli.parse_args(["--calibrate", "--dry-run", "--mode", "line",
                           "--pattern-length-mm", "10"])
    assert args.calibrate and args.text is None
    args = cli.parse_args(["--pattern", "solid", "--dry-run", "--mode", "line"])
    assert args.pattern == "solid"


def test_cli_page_calibration_flag_defaults_to_none_and_parses():
    args = cli.parse_args(["--pos", "--simulate"])
    assert args.page_calibration is None
    args = cli.parse_args(["--pos", "--simulate", "--page-calibration", "cal.json"])
    assert args.page_calibration == "cal.json"


def test_cli_mode_defaults_to_page():
    # Hardware testing has moved to page/freehand as the primary workflow;
    # --mode omitted must now resolve to "page" (used to be "line").
    args = cli.parse_args(["Hi", "--dry-run", "--page-calibration", "cal.json"])
    assert args.mode == "page"


def test_cli_default_page_mode_requires_page_calibration():
    # Same rule that already applied to explicit --mode page (see
    # test_cli_rejects_nozzle_block_remap_in_page_mode's sibling checks
    # elsewhere) now engages by default: omitting --mode with tracking on
    # and no --page-calibration must still fail with a clear argparse error,
    # not silently do the wrong thing.
    try:
        cli.parse_args(["Hi", "--dry-run"])
        assert False, "expected SystemExit: default --mode page requires --page-calibration"
    except SystemExit:
        pass


def test_cli_mode_line_still_opts_out_without_calibration():
    # --mode line must still work with no calibration required, exactly as
    # before this round's default change.
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line"])
    assert args.mode == "line"


def test_cli_poll_hz_defaults_to_500_and_round_trips():
    # Hardware testing found 200 Hz too coarse for page/freehand precision;
    # the default is now 500 Hz, and an explicit value must still round-trip.
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line"])
    assert args.poll_hz == 500.0
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line", "--poll-hz", "200"])
    assert args.poll_hz == 200.0


def test_cli_page_frame_defaults_to_calibrated():
    args = cli.parse_args(["Hi", "--dry-run", "--page-calibration", "cal.json"])
    assert args.page_frame == "calibrated"


def test_cli_page_frame_simple_needs_no_calibration():
    # The whole point of --page-frame simple: page mode without the
    # --page-calibration requirement that would otherwise SystemExit here.
    args = cli.parse_args(["Hi", "--dry-run", "--page-frame", "simple"])
    assert args.page_frame == "simple" and args.mode == "page"
    assert args.page_calibration is None


def test_cli_page_frame_simple_conflicts_with_page_calibration():
    # Two different page frames asked for at once -- refuse rather than
    # silently honouring one (see parse_args).
    try:
        cli.parse_args(["Hi", "--dry-run", "--page-frame", "simple",
                        "--page-calibration", "cal.json"])
        assert False, "expected SystemExit for simple + --page-calibration"
    except SystemExit:
        pass


def test_cli_build_page_calibration_synthesises_the_simple_frame():
    # build_page_calibration must produce the frame itself for simple (no
    # file is read), and the identity boresight has to survive -- without it
    # PageMapper disables rotation correction entirely.
    args = cli.parse_args(["Hi", "--dry-run", "--page-frame", "simple"])
    cal = cli.build_page_calibration(args)
    assert cal is not None
    assert np.allclose(cal.e_col, [1, 0, 0]) and np.allclose(cal.e_row, [0, 1, 0])
    assert cal.boresight_quat is not None


def test_cli_page_frame_reaches_tracking_settings():
    args = cli.parse_args(["Hi", "--dry-run", "--page-frame", "simple"])
    assert cli.build_tracking(args).page_frame == "simple"


def test_cli_rejects_nozzle_block_remap_in_page_mode():
    # Page mode's nozzle-to-row alignment slides with vertical travel (see
    # nozzle_map.py's docstring), so the block permutation -- indexed by fixed
    # image row -- is only correct at multiples of the block size. Rejected
    # outright rather than silently producing a wrong print most of the time.
    try:
        cli.parse_args(["Hi", "--mode", "page", "--page-calibration", "cal.json",
                        "--nozzle-block-size", "5", "--nozzle-order", "2,3,4,1,5",
                        "--dry-run"])
        assert False, "expected SystemExit"
    except SystemExit:
        pass


def test_cli_still_accepts_nozzle_block_remap_in_line_mode():
    # The guard above must not be over-broad: the same flags are fine in line
    # mode (no longer the default -- --mode page is -- so requested
    # explicitly), where the remap is geometrically correct.
    args = cli.parse_args(["Hi", "--mode", "line", "--nozzle-block-size", "5",
                           "--nozzle-order", "2,3,4,1,5", "--dry-run"])
    assert args.nozzle_block_size == 5
    assert args.nozzle_order == "2,3,4,1,5"


# ============================================================================
# --pattern-height-mm / rows= (page mode's taller-than-IMAGE_HEIGHT patterns)
# ============================================================================
def test_all_generators_honour_rows_override():
    # Page mode's whole point is a target image that is NOT capped at
    # IMAGE_HEIGHT rows -- every generator must actually use an explicit
    # rows=, not silently keep producing IMAGE_HEIGHT regardless.
    mm_per_column = 0.2
    for fn in _ALL_GENERATORS:
        ink = fn(20.0, mm_per_column, rows=300)
        assert ink.shape[0] == 300, fn.__name__


def test_all_generators_default_rows_to_image_height():
    # Backward compatibility: line/time mode (and any caller that doesn't
    # pass rows=) must keep getting exactly IMAGE_HEIGHT rows, matching
    # rendering.render_text and frames_from_ink()'s fixed-frame packing.
    mm_per_column = 0.2
    for fn in _ALL_GENERATORS:
        ink = fn(20.0, mm_per_column)
        assert ink.shape[0] == IMAGE_HEIGHT, fn.__name__


def test_ruler_pattern_rows_scales_baseline_position():
    # The baseline must track rows, not stay pinned to the old fixed
    # IMAGE_HEIGHT // 2 -- proves the parameter is actually wired in, not
    # just accepted and ignored.
    n = 300
    ink = patterns.ruler_pattern(10.0, 1.0, rows=n)
    assert ink.shape[0] == n
    mid = n // 2
    assert ink[mid, :].all(), "baseline row must be continuous across the width"

    old_mid = IMAGE_HEIGHT // 2
    assert old_mid != mid
    assert not ink[old_mid, :].all(), (
        "baseline still sitting at the old fixed IMAGE_HEIGHT // 2 position "
        "-- rows= is not actually driving it")


def test_diagonal_pattern_rows_reaches_top_and_bottom():
    # diagonal_pattern's y formula uses (rows - 1); with a taller image it
    # must still span the full height (top row at x % period == 0, bottom
    # row at x % period == period - 1) and stay in bounds while doing so.
    n = 300
    ink = patterns.diagonal_pattern(60.0, 1.0, square_mm=20.0, rows=n)
    assert ink.shape[0] == n
    assert ink[0, :].any(), "diagonal never reaches the top row of the taller image"
    assert ink[n - 1, :].any() or ink[n - 2, :].any(), (
        "diagonal never reaches the bottom of the taller image")


def test_cli_pattern_height_mm_requires_page_mode():
    args = cli.parse_args(["--pattern", "checkerboard", "--dry-run", "--mode", "page",
                           "--page-calibration", "cal.json", "--pattern-height-mm", "100"])
    assert args.pattern_height_mm == 100.0

    try:
        # --mode line (explicit): must still be rejected specifically because
        # --pattern-height-mm needs page mode -- not for the unrelated reason
        # that page mode (now the default) would need --page-calibration too.
        cli.parse_args(["--pattern", "checkerboard", "--dry-run", "--mode", "line",
                        "--pattern-height-mm", "100"])
        assert False, "expected SystemExit: --pattern-height-mm needs --mode page"
    except SystemExit:
        pass


def test_cli_pattern_square_height_mm_overrides_square_rows():
    # square_height_mm must actually change the produced ink's row-banding
    # period, not just be accepted into the namespace and dropped.
    square_rows_equiv = 4
    square_height_mm = NOZZLE_PITCH_MM * square_rows_equiv
    args = cli.parse_args(["--pattern", "checkerboard", "--dry-run", "--mode", "line",
                           "--pattern-length-mm", "4", "--mm-per-column", "1.0",
                           "--pattern-square-mm", "2",
                           "--pattern-square-height-mm", str(square_height_mm)])
    ink, _label = cli.build_ink(args, args.mm_per_column)

    # Column 0's tile band is XOR'd with cols[0] == 0, so it reflects the row
    # band directly: the first row it flips at IS the effective square_rows.
    col0 = ink[:, 0]
    flips = np.nonzero(np.diff(col0.astype(int)))[0]
    assert flips.size > 0, "no banding at all -- override did not take effect"
    first_flip_row = flips[0] + 1
    assert first_flip_row == square_rows_equiv, (
        f"expected banding period {square_rows_equiv}, got {first_flip_row}")


def test_cli_advance_axis_defaults_to_x():
    # Guards Change 1 against a silent regression back to "y".
    # --mode line: advance_axis is a line-mode-only concept; this test isn't
    # about mode selection and avoids needing --page-calibration.
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line"])
    assert args.advance_axis == "x"


if __name__ == "__main__":
    tests = [v for k, v in list(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"All {len(tests)} pattern/mapping tests passed.")
