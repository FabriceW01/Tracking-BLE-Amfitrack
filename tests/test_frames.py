"""
Protocol-equivalence and smoke tests (no hardware required).

Run with:  python -m pytest -q     or     python tests/test_frames.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.geometry import (          # noqa: E402
    FIRST_NOZZLE, IMAGE_HEIGHT, ROW_BYTES,
)
from printhead.rendering import frames_from_ink  # noqa: E402


def _frames_from_ink_reference(ink):
    """Original per-pixel implementation, kept as the correctness oracle."""
    h, w = ink.shape
    byte_idx = [(FIRST_NOZZLE + y) >> 3 for y in range(h)]
    bitmask = [1 << ((FIRST_NOZZLE + y) & 7) for y in range(h)]
    frames = []
    for x in range(w):
        frame = bytearray(ROW_BYTES)
        for y in np.nonzero(ink[:, x])[0]:
            yy = int(y)
            frame[byte_idx[yy]] |= bitmask[yy]
        frames.append(bytes(frame))
    return frames


def test_packbits_matches_reference():
    rng = np.random.default_rng(1234)
    ink = rng.random((IMAGE_HEIGHT, 200)) < 0.35     # random 164 x 200 mask
    fast = frames_from_ink(ink)
    ref = _frames_from_ink_reference(ink)
    assert fast == ref, "vectorised frames differ from the reference"


def test_edge_nozzles_map_correctly():
    ink = np.zeros((IMAGE_HEIGHT, 1), dtype=bool)
    ink[0, 0] = True                                  # nozzle FIRST_NOZZLE = 2
    ink[IMAGE_HEIGHT - 1, 0] = True                   # nozzle LAST_NOZZLE  = 165
    frame = frames_from_ink(ink)[0]
    assert len(frame) == ROW_BYTES
    # nozzle 2  -> byte 0 bit 2
    assert frame[0] == (1 << 2)
    # nozzle 165 -> byte 20 bit 5
    assert frame[165 >> 3] == (1 << (165 & 7))


if __name__ == "__main__":
    test_packbits_matches_reference()
    test_edge_nozzles_map_correctly()
    print("OK: all frame tests passed.")
