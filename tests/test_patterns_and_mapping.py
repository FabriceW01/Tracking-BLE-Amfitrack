"""
Pattern generation and nozzle-remapping tests (no hardware).

Run with:  python tests/test_patterns_and_mapping.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead import cli, patterns                       # noqa: E402
from printhead.geometry import IMAGE_HEIGHT                # noqa: E402
from printhead.nozzle_map import parse_order, remap_rows   # noqa: E402
from printhead.rendering import frames_from_ink            # noqa: E402


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
    args = cli.parse_args(["--calibrate", "--dry-run", "--pattern-length-mm", "10"])
    assert args.calibrate and args.text is None
    args = cli.parse_args(["--pattern", "solid", "--dry-run"])
    assert args.pattern == "solid"


if __name__ == "__main__":
    tests = [v for k, v in list(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"All {len(tests)} pattern/mapping tests passed.")
