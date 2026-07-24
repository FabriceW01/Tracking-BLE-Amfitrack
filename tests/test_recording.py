"""
Send-recorder / reconstruction tests (no hardware).

Run with:  python tests/test_recording.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.geometry import IMAGE_HEIGHT                 # noqa: E402
from printhead.recording import SendRecorder, _decode       # noqa: E402
from printhead.rendering import frames_from_ink             # noqa: E402


def _frame_with_row(row):
    ink = np.zeros((IMAGE_HEIGHT, 1), dtype=bool)
    ink[row, 0] = True
    return frames_from_ink(ink)[0]


def test_decode_roundtrips_a_frame():
    for row in (0, 7, 8, IMAGE_HEIGHT - 1):
        col = _decode(_frame_with_row(row))
        assert col[row] and col.sum() == 1, row


def test_burst_collapses_to_latest_frame():
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
    # At the burst position the LATEST frame (fb, row 1) wins; fa (row 0) is lost.
    assert recon[1, 5] and not recon[0, 5]
    # fb spans until fc's position.
    assert recon[1, 9] and not recon[1, 10]
    # fc from x=10 on.
    assert recon[2, 10]


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


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All recording tests passed.")
