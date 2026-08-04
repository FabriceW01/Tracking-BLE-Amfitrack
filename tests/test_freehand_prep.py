"""
Milestone 1 prep for freehand page-mode printing (no hardware, no behaviour
change to the existing 1D line-mode pipeline):

  * ``tracking.waypoints_path`` -- fakes an arbitrary 2D/3D scribble for
    ``SimulatedTracker``, needed to test calibration/coverage without hardware.
  * ``rendering.pack_nozzle_bits`` -- single-column frame packer, factored out
    of ``frames_from_ink`` for the coverage engine to reuse.
  * ``AmfitrackTracker._extract_pose`` -- position extraction unchanged,
    quaternion extraction added alongside it (confirmed working on real
    hardware).
  * ``ui.server._try_parse_json`` -- JSON-vs-log dispatch shared between the
    sensor stream and print-action handlers.

Run with:  python tests/test_freehand_prep.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.geometry import (                                     # noqa: E402
    IMAGE_HEIGHT, NOZZLE_BAR_WIDTH_MM, NOZZLE_PITCH_MM, NUM_NOZZLES,
)
from printhead.rendering import frames_from_ink, pack_nozzle_bits    # noqa: E402
from printhead.tracking import (                                     # noqa: E402
    AmfitrackTracker, SimulatedTracker, waypoints_path,
)
from printhead.config import TrackingSettings                        # noqa: E402
from printhead.ui.server import _try_parse_json                      # noqa: E402


# =================================================================== geometry
def test_nozzle_pitch_matches_measured_bar_width():
    # User measurement: 152 nozzles span a 15mm-wide bar edge-to-edge, i.e.
    # 151 gaps between 152 nozzle centres.
    assert NOZZLE_BAR_WIDTH_MM == 15.0
    assert NUM_NOZZLES == 152
    assert math.isclose(NOZZLE_PITCH_MM * (NUM_NOZZLES - 1), NOZZLE_BAR_WIDTH_MM)


# ============================================================== waypoints_path
def test_waypoints_path_interpolates_a_straight_segment():
    p = waypoints_path([(0, 0, 0), (10, 0, 0)], speed_mm_s=10.0)
    assert np.allclose(p(0.0), [0, 0, 0])
    assert np.allclose(p(0.5), [5, 0, 0])
    assert np.allclose(p(1.0), [10, 0, 0])


def test_waypoints_path_turns_a_corner():
    p = waypoints_path([(0, 0, 0), (10, 0, 0), (10, 5, 0)], speed_mm_s=10.0)
    assert np.allclose(p(1.0), [10, 0, 0])         # exactly at the corner
    assert np.allclose(p(1.25), [10, 2.5, 0])      # 2.5mm into the second leg


def test_waypoints_path_clamps_past_the_end():
    p = waypoints_path([(0, 0, 0), (10, 0, 0)], speed_mm_s=10.0)
    assert np.allclose(p(100.0), [10, 0, 0])


def test_waypoints_path_loops():
    p = waypoints_path([(0, 0, 0), (10, 0, 0)], speed_mm_s=10.0, loop=True)
    assert np.allclose(p(1.5), [5, 0, 0])          # 15mm travelled, 15 mod 10 = 5


def test_waypoints_path_rejects_degenerate_input():
    for bad in ([(0, 0, 0)], [(0, 0, 0), (0, 0, 0)]):
        try:
            waypoints_path(bad, speed_mm_s=10.0)
        except ValueError:
            continue
        raise AssertionError(f"expected ValueError for {bad}")


def test_simulated_tracker_follows_a_path_fn():
    # Real elapsed wall-clock time between open() and read_position() is not
    # deterministic enough to assert on, so this checks the wiring instead: a
    # fake path_fn records the elapsed time it was called with and returns a
    # position that could never come from the axis-only fallback formula.
    settings = TrackingSettings()
    calls = []

    def fake_path(t):
        calls.append(t)
        return np.array([1.0, 2.0, 3.0])

    tracker = SimulatedTracker(settings, path_fn=fake_path)
    tracker.open()
    pos = tracker.read_position()
    assert np.allclose(pos, [1.0, 2.0, 3.0])
    assert len(calls) == 1 and calls[0] >= 0.0


def test_simulated_tracker_read_pose_has_no_orientation():
    settings = TrackingSettings()
    tracker = SimulatedTracker(settings)
    tracker.open()
    pos, quat = tracker.read_pose()
    assert pos is not None
    assert quat is None


# =========================================================== pack_nozzle_bits
def test_pack_nozzle_bits_matches_frames_from_ink():
    rng = np.random.default_rng(0)
    ink = rng.random((IMAGE_HEIGHT, 12)) < 0.3
    frames = frames_from_ink(ink)
    for x in range(ink.shape[1]):
        assert pack_nozzle_bits(ink[:, x]) == frames[x]


def test_pack_nozzle_bits_all_off():
    assert pack_nozzle_bits(np.zeros(IMAGE_HEIGHT, dtype=bool)) == bytes(19)


def test_pack_nozzle_bits_tolerates_a_short_vector():
    # coverage.py may hand over fewer than IMAGE_HEIGHT rows near an edge;
    # missing rows must be treated as "off", not raise.
    short = np.ones(5, dtype=bool)
    packed = pack_nozzle_bits(short)
    assert len(packed) == 19
    full = np.zeros(IMAGE_HEIGHT, dtype=bool)
    full[:5] = True
    assert packed == pack_nozzle_bits(full)


# ========================================================== AmfitrackTracker
class _FakeEmf:
    def __init__(self, pos=None, quat=None):
        if pos is not None:
            self.pos_x, self.pos_y, self.pos_z = pos
        if quat is not None:
            self.quat_x, self.quat_y, self.quat_z, self.quat_w = quat


class _FakePayload:
    def __init__(self, emf):
        self.emf = emf


def test_extract_pose_position_only_unchanged():
    payload = _FakePayload(_FakeEmf(pos=(1.0, 2.0, 3.0)))
    pos, quat = AmfitrackTracker._extract_pose(payload)
    assert np.allclose(pos, [1.0, 2.0, 3.0])
    assert quat is None


def test_extract_pose_reads_quaternion_when_present():
    payload = _FakePayload(_FakeEmf(pos=(1.0, 2.0, 3.0), quat=(0.1, -0.2, 0.3, 0.9)))
    pos, quat = AmfitrackTracker._extract_pose(payload)
    assert np.allclose(pos, [1.0, 2.0, 3.0])
    assert np.allclose(quat, [0.1, -0.2, 0.3, 0.9])


def test_extract_pose_falls_back_to_flat_xyz():
    class _FlatEmf:
        x, y, z = 4.0, 5.0, 6.0
    pos, quat = AmfitrackTracker._extract_pose(_FakePayload(_FlatEmf()))
    assert np.allclose(pos, [4.0, 5.0, 6.0])
    assert quat is None


def test_extract_pose_unknown_payload_is_none_none():
    pos, quat = AmfitrackTracker._extract_pose(_FakePayload(object()))
    assert pos is None and quat is None


# ==================================================== ui/server dispatch
def test_try_parse_json_rejects_plain_text():
    assert _try_parse_json("not json") is None


def test_try_parse_json_parses_ndjson():
    assert _try_parse_json('{"event": "position", "x": 1.0}') == {
        "event": "position", "x": 1.0}


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All freehand-prep tests passed.")
