"""
Send-recorder / reconstruction tests (no hardware).

Run with:  python tests/test_recording.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.geometry import IMAGE_HEIGHT                          # noqa: E402
from printhead.recording import SendRecorder, _decode, render_coverage  # noqa: E402
from printhead.rendering import frames_from_ink                      # noqa: E402


def _frame_with_row(row):
    ink = np.zeros((IMAGE_HEIGHT, 1), dtype=bool)
    ink[row, 0] = True
    return frames_from_ink(ink)[0]


def test_decode_roundtrips_a_frame():
    for row in (0, 7, 8, IMAGE_HEIGHT - 1):
        col = _decode(_frame_with_row(row))
        assert col[row] and col.sum() == 1, row


def test_burst_is_laid_out_side_by_side():
    r = SendRecorder(mm_per_column=0.2)
    fa = _frame_with_row(0)
    fb = _frame_with_row(1)
    fc = _frame_with_row(2)
    # fa and fb sent at the SAME position (advance 1.0 -> x=5): a gap-fill burst.
    r.record(1.0, fa)
    r.record(1.0, fb)
    # fc sent later at advance 2.0 -> x=10.
    r.record(2.0, fc)

    recon = r.reconstruct()
    # The firmware queues both burst columns: neither is lost, fb spills to x=6.
    assert recon[0, 5] and not recon[1, 5]
    assert recon[1, 6] and not recon[0, 6]
    # Each column occupies exactly one slot -- no smearing across the gap.
    assert not recon[:, 7:10].any()
    assert recon[2, 10]


def test_undersampled_feed_leaves_gaps():
    # Client only managed a column every 3rd position: the firmware prints each
    # once, so the gaps in between stay empty instead of being smeared over.
    r = SendRecorder(mm_per_column=0.2)
    for c in range(0, 12, 3):
        r.record(c * 0.2, _frame_with_row(c))
    recon = r.reconstruct()
    for c in range(0, 12, 3):
        assert recon[c, c], c
    assert not recon[:, 1:3].any()
    assert not recon[:, 4:6].any()


def test_blank_consumes_a_slot_without_ink():
    r = SendRecorder(mm_per_column=0.2)
    r.record(0.0, _frame_with_row(0))
    r.record(0.0, bytes(len(_frame_with_row(0))))    # blank, same position
    r.record(0.0, _frame_with_row(1))
    recon = r.reconstruct()
    assert recon[0, 0]
    assert not recon[:, 1].any()                     # the blank's slot
    assert recon[1, 2]


def test_clean_stream_matches_intended():
    # One frame per column at evenly spaced positions -> reconstruction equals
    # the intended columns (no compression).
    rng = np.random.default_rng(3)
    ink = rng.random((IMAGE_HEIGHT, 40)) < 0.4
    frames = frames_from_ink(ink)
    r = SendRecorder(mm_per_column=0.2)
    for c, f in enumerate(frames):
        r.record(c * 0.2, f)                 # exactly one column per position
    recon = r.reconstruct()
    assert np.array_equal(recon[:, :40], ink)


def test_render_writes_png_and_empty_is_false():
    r = SendRecorder(0.2)
    assert r.render("/tmp/should_not_exist_recording.png") is False

    ink = np.zeros((IMAGE_HEIGHT, 5), dtype=bool)
    ink[0, :] = True
    for c in range(5):
        r.record(c * 0.2, frames_from_ink(ink)[c])
    path = os.path.join(os.environ.get("TMPDIR", "/tmp"), "printhead_rec_test.png")
    try:
        assert r.render(path, ink) is True
        from PIL import Image
        w, h = Image.open(path).size
        # two stacked panels (intended + sent) + labels -> taller than one panel
        assert h > IMAGE_HEIGHT and w >= 5
    finally:
        if os.path.exists(path):
            os.remove(path)


# ============================================================= render_coverage
def test_render_coverage_returns_false_for_nothing_printed():
    ink = np.ones((10, 5), dtype=bool)
    printed = np.zeros((10, 5), dtype=bool)
    assert render_coverage(printed, ink, "/tmp/should_not_exist_coverage.png") is False


def test_render_coverage_writes_a_taller_than_line_mode_png():
    # A page-mode image can be taller than IMAGE_HEIGHT (152) -- unlike
    # SendRecorder, render_coverage must not assume that cap.
    h, w = 200, 6
    ink = np.zeros((h, w), dtype=bool)
    ink[5:15, 1:4] = True
    printed = np.zeros((h, w), dtype=bool)
    printed[5:15, 1:3] = True                     # column 3 missed on purpose

    path = os.path.join(os.environ.get("TMPDIR", "/tmp"), "printhead_cov_test.png")
    try:
        assert render_coverage(printed, ink, path) is True
        from PIL import Image
        pw, ph = Image.open(path).size
        # three stacked panels (intended + covered + missed) + labels
        assert ph > 3 * h and pw >= w
    finally:
        if os.path.exists(path):
            os.remove(path)


# ==================================================== render_coverage: path
def test_render_coverage_without_paths_matches_the_pre_path_tracking_size():
    # Backward compatibility: a caller that never passes sensor_path/
    # nozzle_path (there was no such thing before this feature) must get
    # byte-for-byte the same 3-panel layout as before -- no 4th panel, same
    # height -- so this stays a safe default for any other caller.
    h, w = 100, 8
    ink = np.zeros((h, w), dtype=bool)
    ink[5:15, 1:4] = True
    printed = ink.copy()
    path = os.path.join(os.environ.get("TMPDIR", "/tmp"), "printhead_cov_nopaths.png")
    try:
        assert render_coverage(printed, ink, path) is True
        from PIL import Image
        with Image.open(path) as im:
            h_no_paths = im.size[1]
        assert render_coverage(printed, ink, path, sensor_path=[], nozzle_path=[]) is True
        with Image.open(path) as im:
            h_empty_paths = im.size[1]
        # Empty lists are falsy same as None -- pushIf-style "nothing to draw"
        # -- must not add a 4th panel either.
        assert h_no_paths == h_empty_paths
    finally:
        if os.path.exists(path):
            os.remove(path)


def test_render_coverage_with_paths_adds_a_taller_coloured_panel():
    h, w = 100, 8
    ink = np.zeros((h, w), dtype=bool)
    ink[5:15, 1:4] = True
    printed = ink.copy()
    sensor_path = [(10, 1), (10, 2), (10, 3)]
    nozzle_path = [(12, 1), (12, 2), (12, 3)]
    path = os.path.join(os.environ.get("TMPDIR", "/tmp"), "printhead_cov_paths.png")
    try:
        assert render_coverage(printed, ink, path,
                               sensor_path=sensor_path, nozzle_path=nozzle_path) is True
        from PIL import Image
        with Image.open(path) as im:
            w_out, h_out = im.size
            assert im.mode == "RGB"
        # 3 grayscale panels + 1 path panel, each h + label + gap
        assert h_out > 4 * h
        assert w_out >= w

        assert render_coverage(printed, ink, path) is True   # no paths
        with Image.open(path) as im:
            h_no_paths = im.size[1]
        assert h_out > h_no_paths, "the path panel must add real height"
    finally:
        if os.path.exists(path):
            os.remove(path)


def test_render_coverage_path_panel_draws_the_expected_colours():
    h, w = 60, 60
    ink = np.zeros((h, w), dtype=bool)
    ink[10:20, 10:20] = True
    printed = ink.copy()
    sensor_path = [(5, 5), (5, 55)]      # horizontal line near the top
    nozzle_path = [(50, 5), (50, 55)]    # horizontal line near the bottom
    path = os.path.join(os.environ.get("TMPDIR", "/tmp"), "printhead_cov_colours.png")
    try:
        assert render_coverage(printed, ink, path,
                               sensor_path=sensor_path, nozzle_path=nozzle_path) is True
        from PIL import Image
        im = Image.open(path).convert("RGB")
        arr = np.array(im)
        # path panel is the last (4th) block; scan its full width for each colour.
        blue = np.any(np.all(arr == (30, 100, 220), axis=-1))
        orange = np.any(np.all(arr == (230, 90, 20), axis=-1))
        green = np.any(np.all(arr == (30, 160, 60), axis=-1))
        dark = np.any(np.all(arr == (40, 40, 40), axis=-1))
        assert blue, "sensor path (blue) must be drawn"
        assert orange, "nozzle path (orange) must be drawn"
        assert green, "start marker (green) must be drawn"
        assert dark, "end marker (dark) must be drawn"
    finally:
        if os.path.exists(path):
            os.remove(path)


def test_render_coverage_path_panel_tolerates_out_of_bounds_points():
    # A path leaving/re-entering the page (or, for the sensor, simply never
    # being over it -- see controller._print_freehand_pass) must not crash;
    # PIL clips drawing operations outside the canvas automatically.
    h, w = 40, 40
    ink = np.ones((h, w), dtype=bool)
    printed = ink.copy()
    wild_path = [(-5000, -5000), (10, 10), (99999, 88888), (5, 5)]
    path = os.path.join(os.environ.get("TMPDIR", "/tmp"), "printhead_cov_wild.png")
    try:
        assert render_coverage(printed, ink, path,
                               sensor_path=wild_path, nozzle_path=wild_path) is True
    finally:
        if os.path.exists(path):
            os.remove(path)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All recording tests passed.")
