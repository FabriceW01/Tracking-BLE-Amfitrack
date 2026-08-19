"""
Send-recorder / reconstruction tests (no hardware).

Run with:  python tests/test_recording.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.geometry import IMAGE_HEIGHT                          # noqa: E402
from printhead.recording import (                                     # noqa: E402
    SendRecorder, _decode, _marker_indices, render_coverage,
)
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


# =============================================================== _marker_indices
def test_marker_indices_empty_for_no_sample_times():
    assert _marker_indices([], 2.0) == []
    assert _marker_indices(None, 2.0) == []


def test_marker_indices_single_marker_for_a_short_pass():
    # Duration well under one interval -- only marker 1, at index 0.
    assert _marker_indices([0.0, 0.3, 0.6], 2.0) == [(0, 1)]


def test_marker_indices_regular_samples_land_on_the_boundary():
    times = [i * 0.5 for i in range(21)]           # 0.0 .. 10.0s, 0.5s apart
    markers = _marker_indices(times, 2.0)
    assert markers == [(0, 1), (4, 2), (8, 3), (12, 4), (16, 5), (20, 6)]
    for idx, number in markers:
        assert abs(times[idx] - (number - 1) * 2.0) < 1e-9


def test_marker_indices_picks_the_nearest_sample_off_the_exact_boundary():
    # No sample lands exactly on t=2.0 -- 1.9 is 0.1 away, 2.3 is 0.3 away,
    # so the marker must land on the 1.9 sample.
    times = [0.0, 1.9, 2.3, 4.1]
    markers = _marker_indices(times, 2.0)
    assert (1, 2) in markers, markers


def test_marker_indices_never_repeats_an_index():
    # Interval finer than the sample rate -- several targets can be nearest
    # to the SAME sample; that sample must only appear once.
    times = [0.0, 5.0]
    markers = _marker_indices(times, 0.5)
    indices = [idx for idx, _ in markers]
    assert len(indices) == len(set(indices)), markers


# =========================================================== render_coverage: scale
def test_render_coverage_default_scale_is_larger_than_unscaled():
    h, w = 40, 30
    ink = np.ones((h, w), dtype=bool)
    printed = ink.copy()
    path_scaled = os.path.join(os.environ.get("TMPDIR", "/tmp"), "printhead_cov_scaled.png")
    path_unscaled = os.path.join(os.environ.get("TMPDIR", "/tmp"), "printhead_cov_unscaled.png")
    try:
        assert render_coverage(printed, ink, path_scaled) is True         # default scale
        assert render_coverage(printed, ink, path_unscaled, scale=1) is True
        from PIL import Image
        with Image.open(path_scaled) as im:
            w_scaled, h_scaled = im.size
        with Image.open(path_unscaled) as im:
            w_unscaled, h_unscaled = im.size
        assert w_scaled > w_unscaled
        assert h_scaled > h_unscaled
    finally:
        for p in (path_scaled, path_unscaled):
            if os.path.exists(p):
                os.remove(p)


def test_render_coverage_scale_1_matches_pre_scaling_dimensions():
    # Regression: scale=1 must reproduce the exact pixel dimensions the
    # function had before scaling existed (w == ink width, h > 3*ink height
    # from labels/gaps) -- same shape assertion the original test used.
    h, w = 200, 6
    ink = np.zeros((h, w), dtype=bool)
    ink[5:15, 1:4] = True
    printed = np.zeros((h, w), dtype=bool)
    printed[5:15, 1:3] = True
    path = os.path.join(os.environ.get("TMPDIR", "/tmp"), "printhead_cov_scale1.png")
    try:
        assert render_coverage(printed, ink, path, scale=1) is True
        from PIL import Image
        pw, ph = Image.open(path).size
        assert ph > 3 * h and pw == w
    finally:
        if os.path.exists(path):
            os.remove(path)


# ============================================= render_coverage: timestamped markers
def test_render_coverage_with_sample_times_draws_numbered_markers_not_plain_dots():
    h, w = 60, 60
    ink = np.ones((h, w), dtype=bool)
    printed = ink.copy()
    sensor_path = [(5, 5), (5, 30), (5, 55)]
    nozzle_path = [(50, 5), (50, 30), (50, 55)]
    sample_times = [0.0, 2.0, 4.0]
    path = os.path.join(os.environ.get("TMPDIR", "/tmp"), "printhead_cov_timed.png")
    try:
        assert render_coverage(printed, ink, path, sensor_path=sensor_path,
                               nozzle_path=nozzle_path, sample_times=sample_times,
                               scale=1) is True
        from PIL import Image
        arr = np.array(Image.open(path).convert("RGB"))
        # Plain start/end dots (green/dark) must NOT appear once sample_times
        # is given -- numbered markers in the path's own colour replace them.
        green = np.any(np.all(arr == (30, 160, 60), axis=-1))
        dark = np.any(np.all(arr == (40, 40, 40), axis=-1))
        assert not green, "plain start dot must not appear alongside markers"
        assert not dark, "plain end dot must not appear alongside markers"
        blue = np.any(np.all(arr == (30, 100, 220), axis=-1))
        orange = np.any(np.all(arr == (230, 90, 20), axis=-1))
        assert blue and orange, "markers must still be drawn, in each path's own colour"
    finally:
        if os.path.exists(path):
            os.remove(path)


def test_render_coverage_without_sample_times_still_draws_plain_dots():
    # Backward compatibility for a caller with paths but no timing info.
    h, w = 60, 60
    ink = np.ones((h, w), dtype=bool)
    printed = ink.copy()
    sensor_path = [(5, 5), (5, 55)]
    nozzle_path = [(50, 5), (50, 55)]
    path = os.path.join(os.environ.get("TMPDIR", "/tmp"), "printhead_cov_notimed.png")
    try:
        assert render_coverage(printed, ink, path, sensor_path=sensor_path,
                               nozzle_path=nozzle_path, scale=1) is True
        from PIL import Image
        arr = np.array(Image.open(path).convert("RGB"))
        green = np.any(np.all(arr == (30, 160, 60), axis=-1))
        dark = np.any(np.all(arr == (40, 40, 40), axis=-1))
        assert green and dark, "plain start/end dots must still appear without sample_times"
    finally:
        if os.path.exists(path):
            os.remove(path)


# ================================== render_coverage: fired (physical ink) mask
def _cov_path(name):
    return os.path.join(os.environ.get("TMPDIR", "/tmp"), name)


def test_render_coverage_covered_panel_follows_fired_not_printed():
    # The reported bug: `printed` is dose-COMPLETION bookkeeping and lags
    # badly on a fast pass, so a printed-based COVERED panel showed vertical
    # striping over squares that came out solid on paper. Given `fired`, the
    # image must be drawn from the physical ink instead.
    #
    # Asserted against a MISSED panel that is genuinely empty rather than
    # against pixel counts: with fired covering all the ink, `ink & ~fired`
    # is empty by construction, so a render that still shows missed pixels
    # would prove the panel is being fed `printed`. `printed` here leaves
    # exactly the alternating columns that produced the reported stripes.
    h, w = 20, 8
    ink = np.ones((h, w), dtype=bool)
    fired = np.ones((h, w), dtype=bool)           # everything got ink
    printed = np.zeros((h, w), dtype=bool)
    printed[:, ::2] = True                        # only half completed a dose

    with_fired = _cov_path("printhead_cov_fired.png")
    printed_only = _cov_path("printhead_cov_printed_only.png")
    try:
        assert render_coverage(printed, ink, with_fired, fired=fired) is True
        assert render_coverage(printed, ink, printed_only) is True
        with open(with_fired, "rb") as fa, open(printed_only, "rb") as fb:
            assert fa.read() != fb.read(), \
                "passing fired must actually change what gets drawn"
    finally:
        for p in (with_fired, printed_only):
            if os.path.exists(p):
                os.remove(p)

    # Unambiguous proof of which mask drives the drawing: a pass where the
    # dose NEVER completed anywhere has an all-empty `printed`, so the old
    # printed-only call has nothing to draw and bails -- while the same pass
    # with ink genuinely on the paper renders. Only possible if COVERED
    # follows `fired`.
    nothing_completed = np.zeros((h, w), dtype=bool)
    only_fired = _cov_path("printhead_cov_only_fired.png")
    try:
        assert render_coverage(nothing_completed, ink,
                               _cov_path("unused.png")) is False
        assert render_coverage(nothing_completed, ink, only_fired,
                               fired=fired) is True
    finally:
        if os.path.exists(only_fired):
            os.remove(only_fired)


def test_render_coverage_returns_false_when_nothing_was_inked():
    # Even if `printed` were somehow non-empty, an empty `fired` means the
    # paper is blank and there is nothing to draw.
    ink = np.ones((10, 5), dtype=bool)
    printed = np.ones((10, 5), dtype=bool)
    fired = np.zeros((10, 5), dtype=bool)
    assert render_coverage(printed, ink, _cov_path("nope.png"), fired=fired) is False


def test_render_coverage_adds_a_thin_panel_when_dose_lagged_behind_ink():
    # "inked but under-dosed" is a different problem from "missed", and calls
    # for a different correction (slow down vs. go back over it), so it gets
    # its own panel -- which makes the image taller.
    h, w = 20, 8
    ink = np.ones((h, w), dtype=bool)
    fired = np.ones((h, w), dtype=bool)
    lagging = np.zeros((h, w), dtype=bool)
    lagging[:, ::2] = True                        # half under-dosed
    complete = np.ones((h, w), dtype=bool)        # nothing under-dosed

    with_thin = _cov_path("printhead_cov_thin.png")
    without_thin = _cov_path("printhead_cov_nothin.png")
    try:
        assert render_coverage(lagging, ink, with_thin, fired=fired) is True
        assert render_coverage(complete, ink, without_thin, fired=fired) is True
        from PIL import Image
        assert Image.open(with_thin).size[1] > Image.open(without_thin).size[1], \
            "the THIN panel should add height only when it has content"
    finally:
        for p in (with_thin, without_thin):
            if os.path.exists(p):
                os.remove(p)


def test_render_coverage_without_fired_reproduces_the_old_image_exactly():
    # Backward compatibility for every caller/test predating `fired`:
    # omitting it must make `printed` stand in for both, byte-for-byte.
    h, w = 30, 6
    ink = np.zeros((h, w), dtype=bool)
    ink[5:15, 1:4] = True
    printed = np.zeros((h, w), dtype=bool)
    printed[5:15, 1:3] = True

    a = _cov_path("printhead_cov_old.png")
    b = _cov_path("printhead_cov_explicit.png")
    try:
        assert render_coverage(printed, ink, a) is True
        assert render_coverage(printed, ink, b, fired=printed) is True
        with open(a, "rb") as fa, open(b, "rb") as fb:
            assert fa.read() == fb.read(), \
                "fired=printed must be identical to omitting fired"
    finally:
        for p in (a, b):
            if os.path.exists(p):
                os.remove(p)



if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All recording tests passed.")
