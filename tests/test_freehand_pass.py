"""
Freehand page-mode pass tests (no hardware): PrintController._print_freehand_pass
wiring PageMapper + CoverageEngine + PatternSender together, plus the CLI
plumbing (--mode page requires --page-calibration).

Run with:  python tests/test_freehand_pass.py
"""

import asyncio
import io
import json
import os
import sys
import tempfile
from contextlib import redirect_stdout

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead import cli                                             # noqa: E402
from printhead.calibration import PageCalibration                     # noqa: E402
from printhead.config import BleSettings, RenderSettings, TrackingSettings  # noqa: E402
from printhead.controller import PrintController, _NullPrinthead      # noqa: E402
from printhead.coverage import CoverageEngine, DEFAULT_DOSE_HOLD_S    # noqa: E402
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


def _controller(ink, dose_hold_s=0.01, poll_hz=500.0, timeout_s=2.0,
                profile=False, profile_csv=None, record=None, progress_json=False):
    render = RenderSettings(text="freehand test")
    ble = BleSettings()
    trk = TrackingSettings(mode="page", mm_per_column=1.0, smooth_ms=0.0,
                           poll_hz=poll_hz, timeout_s=timeout_s)
    return PrintController(render, ble, trk, ink=ink,
                           page_calibration=_identity_calibration(),
                           dose_hold_s=dose_hold_s, profile=profile,
                           profile_csv=profile_csv, record=record,
                           progress_json=progress_json)


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


# ============================================================ --progress-json
def _ndjson_events(output: str):
    events = []
    for line in output.splitlines():
        if not line.strip():
            continue
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return events


def test_progress_json_emits_start_sample_and_done_events():
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0, progress_json=True)
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    events = _ndjson_events(out.getvalue())

    assert events[0] == {"event": "coverage_start", "width": 5, "height": 30}
    samples = [e for e in events if e.get("event") == "coverage"]
    assert samples, "expected at least one coverage sample event"
    assert {"u", "v", "row", "col", "new_cells"} <= samples[0].keys()

    done = events[-1]
    assert done["event"] == "coverage_done"
    assert done["reason"] == "complete"
    assert done["covered"] == done["total"] == 150


def test_progress_json_reports_newly_covered_cells():
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0, progress_json=True)
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    samples = [e for e in _ndjson_events(out.getvalue()) if e.get("event") == "coverage"]

    all_new_cells = [cell for s in samples for cell in s["new_cells"]]
    assert len(all_new_cells) == 150             # every ink pixel reported exactly once
    assert len(set(map(tuple, all_new_cells))) == 150   # no duplicates


def test_progress_json_suppresses_plain_text_output():
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0, progress_json=True)
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    for line in out.getvalue().splitlines():
        if line.strip():
            json.loads(line)          # every non-blank line must be valid JSON


def test_progress_json_off_by_default_stays_plain_text():
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0)          # progress_json=False
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    assert "Page fully covered." in out.getvalue()


def test_cli_progress_json_flag():
    args = cli.parse_args(["Hi", "--dry-run", "--progress-json"])
    assert args.progress_json is True
    args = cli.parse_args(["Hi", "--dry-run"])
    assert args.progress_json is False


# ===================================================== --profile / --record
def test_freehand_pass_with_profile_prints_a_page_mode_report():
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0, profile=True)
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    report = out.getvalue()
    assert "page-mode timing profile" in report, report
    assert "pattern updates sent" in report, report


def test_freehand_pass_with_profile_csv_writes_the_page_schema():
    ink = np.ones((30, 5), dtype=bool)
    with tempfile.TemporaryDirectory() as tmp:
        csv_path = os.path.join(tmp, "profile.csv")
        ctrl = _controller(ink, timeout_s=5.0, profile=True, profile_csv=csv_path)
        tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
        with open(csv_path) as fh:
            header = fh.readline().strip()
        assert header == "t_s,row,col,u_mm,v_mm,speed_mm_s,writes_per_s"


def test_freehand_pass_with_record_writes_a_coverage_png():
    ink = np.ones((30, 5), dtype=bool)
    with tempfile.TemporaryDirectory() as tmp:
        png_path = os.path.join(tmp, "coverage.png")
        ctrl = _controller(ink, timeout_s=5.0, record=png_path)
        tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))
        out = io.StringIO()
        with redirect_stdout(out):
            asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
        assert os.path.exists(png_path)
        assert "Coverage reconstruction" in out.getvalue()


# ==================================================== out-of-page diagnosis (defect 2)
def _out_of_page_positions(n_cols=5, samples_per_col=12, v=500.0):
    """Same shape as _sweep_positions, but at a v_mm far outside the reach of
    the 152-nozzle bar for a 30-row image -- CoverageEngine can never see any
    nozzle land in bounds no matter what u_mm is."""
    positions = []
    for c in range(n_cols):
        positions += [(float(c), v, 0.0)] * samples_per_col
    return positions


def test_freehand_pass_out_of_page_reports_zero_coverage_and_diagnosis():
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=0.2, poll_hz=500.0)
    tracker = ScriptedTracker(_out_of_page_positions())
    null = _NullPrinthead()

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(null, tracker))
    text = out.getvalue()

    assert "Covered 0/" in text
    assert "cart never overlapped" in text, text
    assert "--pos --page-calibration" in text, text

    # The pass-level assertions above only prove the *symptom* (0 covered);
    # confirm the actual signal the fix depends on -- CoverageEngine itself
    # reporting no nozzle was ever in bounds for this position.
    mapper = PageMapper(_identity_calibration())
    coverage = CoverageEngine(ink, mm_per_column=1.0, dose_hold_s=0.01)
    u, v, _z = mapper.project(np.array([2.0, 500.0, 0.0]))
    coverage.step(u, v, 0.0)
    assert coverage.last_in_bounds is False
    assert coverage.printed.sum() == 0


def test_freehand_pass_in_bounds_does_not_print_out_of_page_diagnosis():
    # Guard against a false positive: a normal, fully-covered pass must never
    # trigger the out-of-page diagnosis text.
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0)
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    text = out.getvalue()

    assert "cart never overlapped" not in text, text
    assert "Covered 150/150" in text, text


def test_progress_json_out_of_page_carries_bounds_fields():
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=0.2, poll_hz=500.0, progress_json=True)
    tracker = ScriptedTracker(_out_of_page_positions())

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    events = _ndjson_events(out.getvalue())

    done = events[-1]
    assert done["event"] == "coverage_done"
    assert done["covered"] == 0
    assert done["in_bounds_samples"] == 0
    assert done["samples"] > 0
    assert done["u_min"] is not None and done["u_max"] is not None
    assert done["v_min"] == done["v_max"] == 500.0


# =========================================== dose-hold / poll-interval guard
def test_freehand_pass_warns_when_dose_hold_exceeds_poll_interval():
    # The exact quantization-cliff shape from the dose-hold correction: a
    # dose_hold_s (5.4 ms) that sits AT/ABOVE the poll interval (5.0 ms at
    # poll_hz=200) means two consecutive samples can never complete a dose
    # -- a third (or later) sample is required, and measured coverage
    # collapsed from 100% to 31% at exactly this ratio. The guard must warn
    # at pass start rather than let this surface as silent near-zero
    # coverage with no obvious cause.
    ink = np.zeros((5, 5), dtype=bool)   # nothing to cover -- pass ends immediately
    ctrl = _controller(ink, dose_hold_s=0.0054, poll_hz=200.0, timeout_s=1.0)
    tracker = ScriptedTracker([(0.0, 0.0, 0.0)])

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    text = out.getvalue()

    assert "[warn]" in text, text
    assert "dose_hold_s" in text and "poll interval" in text, text
    assert "three or more samples" in text, text


def test_freehand_pass_does_not_warn_with_a_normally_configured_dose_hold():
    # Guard against a false positive: the corrected production default
    # (coverage.DEFAULT_DOSE_HOLD_S = 4.05 ms, 19% below the 5.0 ms poll
    # interval at poll_hz=200) must never trigger the quantization-cliff
    # warning -- this is the "normally configured" case the guard must stay
    # out of the way of.
    ink = np.zeros((5, 5), dtype=bool)
    ctrl = _controller(ink, dose_hold_s=DEFAULT_DOSE_HOLD_S, poll_hz=200.0,
                       timeout_s=1.0)
    tracker = ScriptedTracker([(0.0, 0.0, 0.0)])

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    text = out.getvalue()

    assert "[warn]" not in text, text


def test_progress_json_suppresses_the_dose_hold_warning_too():
    # --progress-json must stay pure NDJSON (mirrors how the out-of-page
    # warning is suppressed in that mode) even when the quantization-cliff
    # condition holds.
    ink = np.zeros((5, 5), dtype=bool)
    ctrl = _controller(ink, dose_hold_s=0.0054, poll_hz=200.0, timeout_s=1.0,
                       progress_json=True)
    tracker = ScriptedTracker([(0.0, 0.0, 0.0)])

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    for line in out.getvalue().splitlines():
        if line.strip():
            json.loads(line)          # every non-blank line must be valid JSON


# ================================================== interrupted pass cleanup (defect 3)
class _RaisingTracker:
    """Like ScriptedTracker, but raises a distinct exception after
    ``fail_after`` successful reads -- stands in for a KeyboardInterrupt (or
    any other mid-loop failure) landing inside _print_freehand_pass without
    relying on a real KeyboardInterrupt, which is fragile to deliver
    precisely under asyncio. Any exception takes the same cleanup path."""

    class Boom(Exception):
        pass

    def __init__(self, positions, fail_after):
        self._seq = [np.asarray(p, dtype=float) for p in positions]
        self._i = 0
        self._fail_after = fail_after

    def open(self):
        pass

    def close(self):
        pass

    def read_position(self):
        if self._i >= self._fail_after:
            raise _RaisingTracker.Boom("simulated interruption")
        value = self._seq[self._i] if self._i < len(self._seq) else self._seq[-1]
        self._i += 1
        return value


def test_freehand_pass_interrupted_still_closes_profiler_csv_and_attempts_record():
    ink = np.ones((30, 5), dtype=bool)
    with tempfile.TemporaryDirectory() as tmp:
        csv_path = os.path.join(tmp, "profile.csv")
        png_path = os.path.join(tmp, "coverage.png")
        # dose_hold_s=0.0 -> the very first in-bounds sample already dozes
        # its pixels, so coverage.printed is non-empty even though only a
        # few samples run before the simulated interruption -- keeps the
        # "record was attempted and had something to draw" check deterministic.
        ctrl = _controller(ink, timeout_s=5.0, dose_hold_s=0.0,
                           profile=True, profile_csv=csv_path, record=png_path)
        tracker = _RaisingTracker(_sweep_positions(n_cols=5, samples_per_col=12),
                                  fail_after=3)
        null = _NullPrinthead()

        try:
            asyncio.run(ctrl._print_freehand_pass(null, tracker))
            raise AssertionError("expected the simulated interruption to propagate")
        except _RaisingTracker.Boom:
            pass          # this is the whole point: cleanup ran, then it propagated

        assert os.path.exists(csv_path), "profiler CSV must still be closed/flushed"
        with open(csv_path) as fh:
            header = fh.readline().strip()
        assert header == "t_s,row,col,u_mm,v_mm,speed_mm_s,writes_per_s"

        assert os.path.exists(png_path), "coverage PNG must still be attempted"


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


def test_cli_ble_write_ceiling_defaults_to_none_and_parses():
    args = cli.parse_args(["Hi", "--dry-run"])
    assert args.ble_write_ceiling is None
    args = cli.parse_args(["Hi", "--dry-run", "--ble-write-ceiling", "150"])
    assert args.ble_write_ceiling == 150.0


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
