"""
Freehand page-mode pass tests (no hardware): PrintController._print_freehand_pass
wiring PageMapper + CoverageEngine + PatternSender together, plus the CLI
plumbing (--mode page requires --page-calibration).

Run with:  python tests/test_freehand_pass.py
"""

import asyncio
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead import cli                                             # noqa: E402
from printhead.calibration import PageCalibration                     # noqa: E402
from printhead.config import BleSettings, RenderSettings, TrackingSettings  # noqa: E402
from printhead.controller import PrintController, _NullPrinthead      # noqa: E402
from printhead.coverage import CoverageEngine                         # noqa: E402
from printhead.tracking import PageMapper                             # noqa: E402


class ScriptedTracker:
    """Returns a predetermined sequence of (x, y, z) positions, holding the
    last one once the sequence is exhausted (mirrors test_position_pass.py's
    ScriptedTracker, generalised from a 1D advance to a full 3D position)."""

    def __init__(self, positions):
        self._seq = [np.asarray(p, dtype=float) for p in positions]
        self._i = 0

    def open(self):
        pass

    def close(self):
        pass

    def read_position(self):
        if self._i < len(self._seq):
            value = self._seq[self._i]
            self._i += 1
        else:
            value = self._seq[-1]
        return value


def _identity_calibration():
    """u_mm == x, v_mm == y -- trivial, easy-to-reason-about frame."""
    return PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                           e_row=np.array([0.0, 1.0, 0.0]))


def _controller(ink, dose_hold_s=0.01, poll_hz=500.0, timeout_s=2.0):
    render = RenderSettings(text="freehand test")
    ble = BleSettings()
    trk = TrackingSettings(mode="page", mm_per_column=1.0, smooth_ms=0.0,
                           poll_hz=poll_hz, timeout_s=timeout_s)
    return PrintController(render, ble, trk, ink=ink,
                           page_calibration=_identity_calibration(),
                           dose_hold_s=dose_hold_s)


def _sweep_positions(n_cols, samples_per_col=12):
    """u_mm = 0, 1, ..., n_cols-1, each held for samples_per_col samples --
    long enough (at the poll rates used below) to clear a small dose_hold_s."""
    positions = []
    for c in range(n_cols):
        positions += [(float(c), 0.0, 0.0)] * samples_per_col
    return positions


# ============================================================ __init__ / mode
def test_page_mode_skips_frames_from_ink_and_allows_a_tall_image():
    tall_ink = np.ones((300, 5), dtype=bool)     # taller than the 152-nozzle bar
    ctrl = _controller(tall_ink)
    assert ctrl.frames is None
    assert ctrl.height == 300 and ctrl.width == 5


def test_line_mode_still_builds_frames_as_before():
    render = RenderSettings(text="x")
    trk = TrackingSettings(mode="line", mm_per_column=0.2)
    ink = np.ones((152, 3), dtype=bool)
    ctrl = PrintController(render, BleSettings(), trk, ink=ink)
    assert ctrl.frames is not None
    assert len(ctrl.frames) == 3


# ===================================================== _print_freehand_pass
def test_freehand_pass_covers_the_page_and_stops_before_timeout():
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0)
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))
    null = _NullPrinthead()

    asyncio.run(ctrl._print_freehand_pass(null, tracker))

    assert null.pattern_writes > 0, "expected at least one pattern write"
    assert null.blank_writes == 1, "must send exactly one final blank"


def test_freehand_pass_actually_covers_every_ink_pixel():
    # Same math as the pass above, but driving CoverageEngine/PageMapper
    # directly to check *what* "covered" means at the pixel level, not just
    # that the pass terminated.
    ink = np.ones((30, 5), dtype=bool)
    mapper = PageMapper(_identity_calibration())
    coverage = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=0.01)

    t = 0.0
    for c in range(5):
        for _ in range(12):
            u, v, _z = mapper.project(np.array([float(c), 0.0, 0.0]))
            coverage.step(u, v, t)
            t += 0.002
    assert coverage.done
    assert coverage.printed.sum() == ink.sum()


def test_freehand_pass_times_out_when_coverage_never_completes():
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=0.05, poll_hz=500.0)
    # Stays at column 0 forever: columns 1..4 never get touched, so
    # coverage.done never goes true -- must fall back to the pass timeout.
    tracker = ScriptedTracker([(0.0, 0.0, 0.0)])
    null = _NullPrinthead()

    asyncio.run(ctrl._print_freehand_pass(null, tracker))

    assert null.blank_writes == 1, "must still shut down cleanly on timeout"


def test_freehand_pass_requires_a_page_calibration():
    render = RenderSettings(text="x")
    trk = TrackingSettings(mode="page", mm_per_column=1.0)
    ink = np.ones((10, 3), dtype=bool)
    ctrl = PrintController(render, BleSettings(), trk, ink=ink, page_calibration=None)
    tracker = ScriptedTracker([(0.0, 0.0, 0.0)])
    try:
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    except RuntimeError:
        return
    raise AssertionError("expected RuntimeError without a page calibration")


# ==================================================== dry-run simulation path
def test_dry_run_freehand_pass_runs_without_crashing():
    ink = np.ones((10, 3), dtype=bool)
    ctrl = _controller(ink, timeout_s=0.2, poll_hz=200.0)
    ctrl.simulate = True
    asyncio.run(ctrl._dry_run_freehand_pass())      # must not raise


# ============================================================= CLI validation
def test_cli_requires_page_calibration_for_page_mode():
    try:
        cli.parse_args(["Hi", "--dry-run", "--mode", "page"])
        assert False, "expected SystemExit"
    except SystemExit:
        pass


def test_cli_accepts_page_mode_with_calibration():
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "page",
                           "--page-calibration", "somewhere.json"])
    assert args.mode == "page"
    assert args.page_calibration == "somewhere.json"


def test_cli_no_track_bypasses_the_page_calibration_requirement():
    # --no-track forces time mode regardless of --mode, so the requirement
    # (which is about the *effective* mode) must not fire here.
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "page", "--no-track"])
    assert args.page_calibration is None


def test_build_page_calibration_loads_a_saved_file():
    cal = _identity_calibration()
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "cal.json")
        cal.save(path)
        args = cli.parse_args(["Hi", "--dry-run", "--mode", "page",
                               "--page-calibration", path])
        loaded = cli.build_page_calibration(args)
    assert np.allclose(loaded.origin, cal.origin)
    assert np.allclose(loaded.e_col, cal.e_col)


def test_build_page_calibration_is_none_without_the_flag():
    args = cli.parse_args(["Hi", "--dry-run"])
    assert cli.build_page_calibration(args) is None


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All freehand-pass tests passed.")
