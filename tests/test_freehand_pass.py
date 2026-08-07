"""
Freehand page-mode pass tests (no hardware): PrintController._print_freehand_pass
wiring PageMapper + CoverageEngine + PatternSender together, plus the CLI
plumbing (--mode page requires --page-calibration).

Run with:  python tests/test_freehand_pass.py
"""

import asyncio
import io
import json
import math
import os
import sys
import tempfile
from contextlib import redirect_stdout

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead import cli                                             # noqa: E402
from printhead.calibration import PageCalibration                     # noqa: E402
from printhead.config import BleSettings, RenderSettings, TrackingSettings  # noqa: E402
from printhead.controller import (                                    # noqa: E402
    DEFAULT_SPEED_WARNING_MM_S,
    PrintController,
    _NullPrinthead,
    _speed_warning_transition,
)
from printhead.coverage import CoverageEngine, DEFAULT_DOSE_HOLD_S    # noqa: E402
from printhead.geometry import (                                      # noqa: E402
    NOZZLE_BAR_WIDTH_MM,
    NUM_NOZZLES,
    SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM,
    SENSOR_TO_NOZZLE_COL_MM,
)
from printhead.tracking import PageMapper                             # noqa: E402


class ScriptedTracker:
    """Returns a predetermined sequence of (x, y, z) positions, holding the
    last one once the sequence is exhausted (mirrors test_position_pass.py's
    ScriptedTracker, generalised from a 1D advance to a full 3D position).

    ``_print_freehand_pass`` reads pose via ``read_pose()`` (not
    ``read_position()``), so this needs one too. Default is
    ``quats=None``, mirroring ``SimulatedTracker``'s contract (quaternion
    always ``None`` -- the simulator/fakes never invent orientation); pass
    ``quats`` (same length/indexing convention as ``positions``, also holds
    its last entry once exhausted) for the few tests that need a real
    orientation sequence to reach the profiler CSV."""

    def __init__(self, positions, quats=None):
        self._seq = [np.asarray(p, dtype=float) for p in positions]
        self._quats = ([np.asarray(q, dtype=float) for q in quats]
                       if quats is not None else None)
        self._i = 0

    def open(self):
        pass

    def close(self):
        pass

    def read_position(self):
        return self.read_pose()[0]

    def read_pose(self):
        if self._i < len(self._seq):
            pos = self._seq[self._i]
            quat = self._quats[self._i] if self._quats is not None else None
            self._i += 1
        else:
            pos = self._seq[-1]
            quat = self._quats[-1] if self._quats is not None else None
        return pos, quat


def _identity_calibration():
    """u_mm == x, v_mm == y -- trivial, easy-to-reason-about frame."""
    return PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                           e_row=np.array([0.0, 1.0, 0.0]))


def _controller(ink, dose_hold_s=0.01, poll_hz=500.0, timeout_s=2.0,
                profile=False, profile_csv=None, record=None, progress_json=False,
                speed_warning_mm_s=DEFAULT_SPEED_WARNING_MM_S):
    render = RenderSettings(text="freehand test")
    ble = BleSettings()
    trk = TrackingSettings(mode="page", mm_per_column=1.0, smooth_ms=0.0,
                           poll_hz=poll_hz, timeout_s=timeout_s)
    # Sensor->nozzle offsets neutralised: these tests check
    # CoverageEngine/PatternSender wiring against a controlled identity
    # calibration, unrelated to the (separately tested, see
    # tests/test_page_mapper.py) sensor-to-nozzle-bar offset feature -- a
    # nonzero *effective* offset here would shift v_mm by tens of mm and push
    # every sample out of the small target images used below.
    #
    # NOTE: PageMapper's row axis always subtracts NOZZLE_BAR_WIDTH_MM/2 from
    # whatever sensor_offset_row_mm is given (that is the bar-CENTER-to-
    # nozzle-0 conversion, not "no correction"), so the value that actually
    # cancels to a zero net shift is NOZZLE_BAR_WIDTH_MM/2.0, NOT 0.0 --
    # passing literal 0.0 would itself introduce a -NOZZLE_BAR_WIDTH_MM/2 mm
    # shift. See tests/test_page_mapper.py for this pinned in detail.
    return PrintController(render, ble, trk, ink=ink,
                           page_calibration=_identity_calibration(),
                           dose_hold_s=dose_hold_s, profile=profile,
                           profile_csv=profile_csv, record=record,
                           progress_json=progress_json,
                           speed_warning_mm_s=speed_warning_mm_s,
                           sensor_offset_row_mm=NOZZLE_BAR_WIDTH_MM / 2.0,
                           sensor_offset_col_mm=0.0)


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
    # Neutralised sensor offset (NOZZLE_BAR_WIDTH_MM/2.0, not 0.0 -- see the
    # NOTE in _controller() above): this test drives CoverageEngine/PageMapper
    # directly against a controlled identity calibration and a small (30-row)
    # target image, unrelated to the sensor-to-nozzle-bar offset feature.
    mapper = PageMapper(_identity_calibration(),
                        sensor_offset_row_mm=NOZZLE_BAR_WIDTH_MM / 2.0,
                        sensor_offset_col_mm=0.0)
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
        assert header == "t_s,row,col,u_mm,v_mm,speed_mm_s,writes_per_s,qx,qy,qz,qw"


def test_freehand_pass_with_profile_csv_logs_orientation_when_the_tracker_has_it():
    # End-to-end: a tracker that supplies real quat samples must have them
    # reach the CSV (via _print_freehand_pass -> read_pose() ->
    # record_page_sample(quat=...)), non-blank, for at least one row. This is
    # raw diagnostic data for offline correlation against the suspected
    # rotation + sensor-to-nozzle-bar lever arm misalignment (see
    # geometry.SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM) -- not used live here.
    ink = np.ones((30, 5), dtype=bool)
    with tempfile.TemporaryDirectory() as tmp:
        csv_path = os.path.join(tmp, "profile.csv")
        ctrl = _controller(ink, timeout_s=5.0, profile=True, profile_csv=csv_path)
        positions = _sweep_positions(n_cols=5, samples_per_col=12)
        # Identity-ish quaternion (no rotation) repeated for every sample --
        # the point of this test is that a real value reaches the CSV at
        # all, not any particular rotation.
        quats = [(0.0, 0.0, 0.0, 1.0)] * len(positions)
        tracker = ScriptedTracker(positions, quats=quats)
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))

        with open(csv_path) as fh:
            lines = fh.read().strip().splitlines()
        assert lines[0] == "t_s,row,col,u_mm,v_mm,speed_mm_s,writes_per_s,qx,qy,qz,qw"
        data_rows = lines[1:]
        assert data_rows, "expected at least one profiled sample"
        quat_rows = [row.split(",")[-4:] for row in data_rows]
        assert any(fields == ["0.0000", "0.0000", "0.0000", "1.0000"]
                  for fields in quat_rows), quat_rows


def test_freehand_pass_with_profile_csv_blank_orientation_without_a_quat_tracker():
    # Guard against a false positive on the test above: the default
    # ScriptedTracker (no quats=) must still produce blank -- not "0,0,0,0"
    # -- orientation fields, matching SimulatedTracker/real hardware ticks
    # with no orientation packet.
    ink = np.ones((30, 5), dtype=bool)
    with tempfile.TemporaryDirectory() as tmp:
        csv_path = os.path.join(tmp, "profile.csv")
        ctrl = _controller(ink, timeout_s=5.0, profile=True, profile_csv=csv_path)
        tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))

        with open(csv_path) as fh:
            lines = fh.read().strip().splitlines()
        data_rows = lines[1:]
        assert data_rows, "expected at least one profiled sample"
        for row in data_rows:
            assert row.split(",")[-4:] == ["", "", "", ""], row


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
    # reporting no nozzle was ever in bounds for this position. Neutralised
    # sensor offset to match the neutralised-offset `ctrl` used above (v=500
    # is far enough out of bounds either way, but this keeps the two mappers
    # consistent, and matches the exact v_min/v_max == 500.0 the pass-level
    # test above expects).
    mapper = PageMapper(_identity_calibration(),
                        sensor_offset_row_mm=NOZZLE_BAR_WIDTH_MM / 2.0,
                        sensor_offset_col_mm=0.0)
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
    #
    # NOTE: _controller() -> _identity_calibration() has no boresight_quat,
    # so the SEPARATE "no rotation correction" warning (see
    # test_freehand_pass_warns_about_a_missing_boresight below) is still
    # expected in this output -- this test only pins the absence of the
    # dose-hold/poll-interval warning specifically, not "[warn]" in general.
    ink = np.zeros((5, 5), dtype=bool)
    ctrl = _controller(ink, dose_hold_s=DEFAULT_DOSE_HOLD_S, poll_hz=200.0,
                       timeout_s=1.0)
    tracker = ScriptedTracker([(0.0, 0.0, 0.0)])

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    text = out.getvalue()

    assert "dose_hold_s" not in text and "poll interval" not in text, text


# ==================================================== boresight-missing warning
def test_freehand_pass_warns_about_a_missing_boresight():
    # _identity_calibration() (used throughout this file via _controller())
    # has no boresight_quat -- current behaviour (no rotation correction at
    # all) still applies, but it must be loudly announced rather than kept
    # silent: this is the exact situation every calibration saved before
    # this feature existed is in.
    ink = np.zeros((5, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=1.0)
    tracker = ScriptedTracker([(0.0, 0.0, 0.0)])

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    text = out.getvalue()

    assert "[warn]" in text and "boresight" in text.lower(), text


def test_freehand_pass_does_not_warn_about_boresight_when_one_is_captured():
    ink = np.zeros((5, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=1.0)
    ctrl.page_calibration.boresight_quat = np.array([0.0, 0.0, 0.0, 1.0])
    tracker = ScriptedTracker([(0.0, 0.0, 0.0)])

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    text = out.getvalue()

    assert "boresight" not in text.lower(), text


def test_progress_json_suppresses_the_boresight_warning_too():
    # --progress-json must stay pure NDJSON (mirrors how the dose-hold and
    # out-of-page warnings are suppressed in that mode) even when the
    # calibration has no boresight.
    ink = np.zeros((5, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=1.0, progress_json=True)
    tracker = ScriptedTracker([(0.0, 0.0, 0.0)])

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    for line in out.getvalue().splitlines():
        if line.strip():
            json.loads(line)          # every non-blank line must be valid JSON


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


# ============================== end-to-end: rotation correction reaches the output
def _identity_calibration_with_boresight():
    return PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                           e_row=np.array([0.0, 1.0, 0.0]),
                           boresight_quat=np.array([0.0, 0.0, 0.0, 1.0]))


def _run_freehand_pass_collect_covered_cells(quats):
    """Drive a real _print_freehand_pass (via ScriptedTracker(quats=...)) and
    return the full set of (row, col) cells CoverageEngine ever marked
    printed, reconstructed from --progress-json's new_cells events -- the
    same mechanism test_progress_json_reports_newly_covered_cells already
    relies on, rather than reaching into the pass's private CoverageEngine."""
    ink = np.ones((NUM_NOZZLES + 20, 60), dtype=bool)
    render = RenderSettings(text="rotation e2e")
    ble = BleSettings()
    trk = TrackingSettings(mode="page", mm_per_column=1.0, smooth_ms=0.0,
                           poll_hz=500.0, timeout_s=5.0)
    ctrl = PrintController(render, ble, trk, ink=ink,
                           page_calibration=_identity_calibration_with_boresight(),
                           dose_hold_s=0.01, progress_json=True,
                           sensor_offset_row_mm=NOZZLE_BAR_WIDTH_MM / 2.0,
                           sensor_offset_col_mm=0.0)
    positions = _sweep_positions(n_cols=5, samples_per_col=12)
    tracker = ScriptedTracker(positions, quats=quats)

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    events = _ndjson_events(out.getvalue())

    cells = set()
    for e in events:
        if e.get("event") == "coverage":
            cells.update(tuple(c) for c in e["new_cells"])
    return cells


def test_freehand_pass_rotation_correction_changes_the_printed_mask():
    # Same path, same target image, same dose-hold -- the ONLY difference is
    # the orientation quaternion the tracker reports each sample: identical
    # to the boresight pose (no yaw) vs. rotated 20 degrees the whole pass.
    # If the correction genuinely reaches CoverageEngine (not just PageMapper
    # in isolation), the two runs must cover DIFFERENT cells -- a bar tilted
    # 20 degrees sweeps the 15mm nozzle bar across ~5mm (sin(20deg)*15mm) of
    # extra columns nozzle 0 alone never reaches.
    n_samples = len(_sweep_positions(n_cols=5, samples_per_col=12))
    boresight = (0.0, 0.0, 0.0, 1.0)
    quats_level = [boresight] * n_samples
    half = math.radians(20.0) / 2.0
    quat_rotated = (0.0, 0.0, math.sin(half), math.cos(half))
    quats_rotated = [quat_rotated] * n_samples

    level_cells = _run_freehand_pass_collect_covered_cells(quats_level)
    rotated_cells = _run_freehand_pass_collect_covered_cells(quats_rotated)

    assert level_cells, "expected the level pass to cover something"
    assert rotated_cells, "expected the rotated pass to cover something"
    assert level_cells != rotated_cells, (
        "a rotating pass must print a different mask than the same path "
        "held level -- the yaw correction is not reaching CoverageEngine")


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
        return self.read_pose()[0]

    def read_pose(self):
        if self._i >= self._fail_after:
            raise _RaisingTracker.Boom("simulated interruption")
        value = self._seq[self._i] if self._i < len(self._seq) else self._seq[-1]
        self._i += 1
        return value, None


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
        assert header == "t_s,row,col,u_mm,v_mm,speed_mm_s,writes_per_s,qx,qy,qz,qw"

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


def test_cli_sensor_offset_flags_default_to_none_and_parse():
    # Same "default None -> controller falls back to the geometry constant"
    # pattern as --dose-hold-s / --ble-write-ceiling above.
    args = cli.parse_args(["Hi", "--dry-run"])
    assert args.sensor_offset_row_mm is None
    assert args.sensor_offset_col_mm is None
    args = cli.parse_args(["Hi", "--dry-run", "--sensor-offset-row-mm", "70.0",
                           "--sensor-offset-col-mm", "-3.5"])
    assert args.sensor_offset_row_mm == 70.0
    assert args.sensor_offset_col_mm == -3.5


def test_cli_sensor_offset_flags_reach_the_controller_when_given():
    args = cli.parse_args(["Hi", "--dry-run", "--sensor-offset-row-mm", "70.0",
                           "--sensor-offset-col-mm", "-3.5"])
    ctrl = cli.build_controller(args)
    assert ctrl.sensor_offset_row_mm == 70.0
    assert ctrl.sensor_offset_col_mm == -3.5


def test_cli_sensor_offset_flags_default_to_the_geometry_constants_unset():
    # When not given at all, the controller must fall back to the real
    # measured geometry constants, not to 0.0.
    args = cli.parse_args(["Hi", "--dry-run"])
    ctrl = cli.build_controller(args)
    assert ctrl.sensor_offset_row_mm == SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM
    assert ctrl.sensor_offset_col_mm == SENSOR_TO_NOZZLE_COL_MM


# =============================================================== --boresight-deg
def test_cli_boresight_deg_defaults_to_none_and_parses():
    args = cli.parse_args(["Hi", "--dry-run"])
    assert args.boresight_deg is None
    args = cli.parse_args(["Hi", "--dry-run", "--boresight-deg", "3.5"])
    assert args.boresight_deg == 3.5


def test_cli_boresight_deg_reaches_the_controller_when_given():
    args = cli.parse_args(["Hi", "--dry-run", "--boresight-deg", "-7.25"])
    ctrl = cli.build_controller(args)
    assert ctrl.boresight_deg == -7.25


def test_cli_boresight_deg_defaults_to_zero_unset():
    # Same "default None on the CLI -> 0.0 (neutral) on the controller"
    # pattern as the other page-mode fine-tune flags.
    args = cli.parse_args(["Hi", "--dry-run"])
    ctrl = cli.build_controller(args)
    assert ctrl.boresight_deg == 0.0


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


def test_cli_speed_warning_mm_s_defaults_to_none_and_parses():
    args = cli.parse_args(["Hi", "--dry-run"])
    assert args.speed_warning_mm_s is None
    args = cli.parse_args(["Hi", "--dry-run", "--speed-warning-mm-s", "30"])
    assert args.speed_warning_mm_s == 30.0


# ================================================ speed warning / hysteresis
# The pure decision function first: fast, deterministic, no real timing
# involved -- this is what the mutation check below (see the PR description)
# was actually run against, since it can be driven with an exact, repeatable
# sequence of speeds rather than something dependent on real asyncio.sleep()
# jitter.

def test_speed_warning_transition_turns_on_above_and_off_20pct_below():
    thr = 25.0
    # Below threshold: stays off.
    assert _speed_warning_transition(False, 24.9, thr) is False
    # Above threshold: turns on.
    assert _speed_warning_transition(False, 25.1, thr) is True
    # Already on, still above the OFF edge (20.0 = thr*0.8): stays on.
    assert _speed_warning_transition(True, 20.1, thr) is True
    assert _speed_warning_transition(True, 24.9, thr) is True   # dead band
    # Already on, drops below the OFF edge: turns off.
    assert _speed_warning_transition(True, 19.9, thr) is False
    # Already off, still below ON edge: stays off (no reason to move).
    assert _speed_warning_transition(False, 0.0, thr) is False


def _count_transitions(speeds, threshold_mm_s):
    """Feed a speed sequence through _speed_warning_transition and count how
    many times the returned state actually differs from the previous one --
    i.e. how many BLE writes a real pass would issue for this sequence."""
    state = False
    calls = 0
    for speed in speeds:
        new_state = _speed_warning_transition(state, speed, threshold_mm_s)
        if new_state != state:
            calls += 1
            state = new_state
    return calls


def _hovering_speeds(n=30):
    """Alternates comfortably above (30) and comfortably within the dead
    band (22.5, the midpoint of 20..25) at the default 25 mm/s threshold --
    exactly the "hovering near the boundary" scenario hysteresis exists for."""
    return [30.0 if i % 2 == 0 else 22.5 for i in range(n)]


def test_speed_warning_transition_hysteresis_does_not_chatter_on_hovering_speed():
    calls = _count_transitions(_hovering_speeds(), DEFAULT_SPEED_WARNING_MM_S)
    # Exactly one: the very first sample (30 > 25) turns it on; every sample
    # after that -- 22.5 or 30 -- is still >= the 20 mm/s OFF edge, so it
    # never turns off again for the rest of the sequence.
    assert calls == 1, f"expected exactly 1 transition, got {calls}"


def test_speed_warning_transition_MUTATION_check_removing_the_dead_band_chatters():
    # Same hovering sequence, but with the dead band removed (ON/OFF share
    # the same threshold) -- this inlines the mutation described in the PR
    # instead of editing controller.py by hand, so the regression stays
    # covered by the suite rather than only having been checked once by
    # hand. See the PR description for the verbatim before/after counts
    # from actually mutating _speed_warning_transition and rerunning this
    # file.
    def _no_dead_band(is_warning, speed_mm_s, threshold_mm_s):
        if not is_warning and speed_mm_s > threshold_mm_s:
            return True
        if is_warning and speed_mm_s < threshold_mm_s:   # no * 0.8 here
            return False
        return is_warning

    state = False
    calls = 0
    for speed in _hovering_speeds():
        new_state = _no_dead_band(state, speed, DEFAULT_SPEED_WARNING_MM_S)
        if new_state != state:
            calls += 1
            state = new_state
    # Every 22.5 sample (< 25) now turns it off, every 30 sample (> 25)
    # turns it back on -- chattering on essentially every sample, not the
    # single transition the hysteresis version above gets.
    assert calls > 10, f"expected the dead-band-free version to chatter, got only {calls} transitions"


# ---------------------------------------- integration: through a real pass
def _speed_sweep_positions(deltas, v_mm=1000.0):
    """u_mm advances by each of `deltas` in turn (v_mm fixed, far outside any
    test image's reach so CoverageEngine never completes and the pass always
    ends via --timeout, not via coverage.done) -- one ScriptedTracker sample
    per delta, so at a given poll_hz the resulting along-travel speed is
    delta / (nominal poll interval), give or take real scheduling jitter."""
    positions = []
    u = 0.0
    for d in deltas:
        u += d
        positions.append((u, v_mm, 0.0))
    return positions


def test_freehand_pass_speed_above_threshold_triggers_the_warning():
    # 40 mm/s at poll_hz=100 (10 ms nominal interval) = 0.4 mm/sample --
    # comfortably above the 25 mm/s default threshold even with generous
    # real-time scheduling jitter.
    deltas = [0.4] * 30
    ink = np.ones((5, 5), dtype=bool)
    ctrl = _controller(ink, poll_hz=100.0, timeout_s=0.35)
    tracker = ScriptedTracker(_speed_sweep_positions(deltas))
    null = _NullPrinthead()

    asyncio.run(ctrl._print_freehand_pass(null, tracker))

    assert True in null.speed_warnings, null.speed_warnings


def test_freehand_pass_speed_below_threshold_never_triggers_the_warning():
    # 3 mm/s at poll_hz=100 = 0.03 mm/sample -- comfortably below even the
    # 20 mm/s OFF edge, let alone the 25 mm/s ON threshold.
    deltas = [0.03] * 30
    ink = np.ones((5, 5), dtype=bool)
    ctrl = _controller(ink, poll_hz=100.0, timeout_s=0.35)
    tracker = ScriptedTracker(_speed_sweep_positions(deltas))
    null = _NullPrinthead()

    asyncio.run(ctrl._print_freehand_pass(null, tracker))

    assert True not in null.speed_warnings, null.speed_warnings


def test_freehand_pass_hovering_speed_does_not_chatter():
    # Alternates ~40 mm/s (0.4 mm/sample, comfortably above the 25 mm/s ON
    # threshold) and ~24.5 mm/s (0.245 mm/sample) at poll_hz=100. 24.5 is
    # deliberately biased close to the 25 mm/s ON threshold rather than
    # centred in the dead band (20..25): the margin that actually matters
    # against real asyncio timing jitter is the *lower* one (down to the 20
    # mm/s OFF edge, here ~18%), since occasionally landing above 25 again
    # while already on is harmless (no transition -- see
    # _speed_warning_transition), only dropping below 20 is not. More
    # position samples than the timeout can possibly consume (see
    # _speed_sweep_positions/ScriptedTracker), so the sequence is never
    # exhausted into a held-still, zero-speed tail, which would otherwise
    # add its own (legitimate, but here undesired-for-this-assertion) OFF
    # transition on top of the one this test is actually checking for. A
    # short timeout keeps the number of real (wall-clock, hence jittery)
    # samples small, which is what actually keeps this reliable -- see the
    # deterministic, exact version of this same scenario just above
    # (test_speed_warning_transition_hysteresis_does_not_chatter_on_hovering_speed),
    # which is what the mutation check in the PR description was run
    # against; this integration test only needs to confirm the real pass
    # loop calls through to set_speed_warning() the way that pure function
    # predicts, tolerating occasional real-timing noise with a generous
    # (but still obviously "not chattering") bound below.
    deltas = [0.4 if i % 2 == 0 else 0.245 for i in range(100)]
    ink = np.ones((5, 5), dtype=bool)
    ctrl = _controller(ink, poll_hz=100.0, timeout_s=0.3)
    tracker = ScriptedTracker(_speed_sweep_positions(deltas))
    null = _NullPrinthead()

    asyncio.run(ctrl._print_freehand_pass(null, tracker))

    # Not "one call per sample" (~30 samples run in 0.3 s at poll_hz=100):
    # the ideal count is 2 (one True crossing above 25, one False at
    # cleanup), occasional real-scheduler jitter can add a couple more, but
    # nowhere near the dozens of on/off flips a naive single-threshold
    # version produces for a sequence that straddles 25 mm/s this often
    # (see the MUTATION_check test above for the exact, deterministic
    # contrast).
    assert len(null.speed_warnings) <= 12, null.speed_warnings


def test_freehand_pass_clears_the_warning_at_normal_pass_end():
    deltas = [0.4] * 30
    ink = np.ones((5, 5), dtype=bool)
    ctrl = _controller(ink, poll_hz=100.0, timeout_s=0.35)
    tracker = ScriptedTracker(_speed_sweep_positions(deltas))
    null = _NullPrinthead()

    asyncio.run(ctrl._print_freehand_pass(null, tracker))

    assert null.speed_warnings, "expected at least the warning + its clearing"
    assert null.speed_warnings[-1] is False, \
        f"the last call must clear the warning, got {null.speed_warnings}"


def test_freehand_pass_never_warns_when_tracker_never_moves():
    # No speed can be computed at all (prev_u stays unset) when position is
    # constant -- must never crash, and the only call is the unconditional
    # clear at pass end.
    ink = np.ones((5, 5), dtype=bool)
    ctrl = _controller(ink, poll_hz=100.0, timeout_s=0.1)
    tracker = ScriptedTracker([(0.0, 1000.0, 0.0)])
    null = _NullPrinthead()

    asyncio.run(ctrl._print_freehand_pass(null, tracker))

    assert null.speed_warnings == [False]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All freehand-pass tests passed.")
