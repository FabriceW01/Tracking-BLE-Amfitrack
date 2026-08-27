"""
Freehand page-mode pass tests (no hardware): PrintController._print_freehand_pass
wiring PageMapper + CoverageEngine + PatternSender together, plus the CLI
plumbing (--mode page requires --page-calibration).

Run with:  python tests/test_freehand_pass.py
"""

import asyncio
import io
import re
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
from printhead.controller import (
    DEFAULT_PROGRESS_HZ,                                    # noqa: E402
    DEFAULT_SPEED_WARNING_MM_S,
    PrintController,
    _NullPrinthead,
    _extrapolate_uv,
    _speed_warning_transition,
)
from printhead.coverage import CoverageEngine                          # noqa: E402
from printhead.geometry import (                                      # noqa: E402
    NOZZLE_BAR_SPAN_MM,
    NOZZLE_PITCH_MM,
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


def _controller(ink, drops_per_pixel=2, poll_hz=500.0, timeout_s=2.0,
                profile=False, profile_csv=None, record=None, progress_json=False,
                speed_warning_mm_s=DEFAULT_SPEED_WARNING_MM_S, verbose=False,
                latency_compensate_s=0.0, startpoint_anchor="center",
                progress_hz=DEFAULT_PROGRESS_HZ):
    render = RenderSettings(text="freehand test")
    ble = BleSettings(verbose=verbose)
    trk = TrackingSettings(mode="page", mm_per_column=1.0, smooth_ms=0.0,
                           poll_hz=poll_hz, timeout_s=timeout_s)
    # Sensor->nozzle offsets neutralised: these tests check
    # CoverageEngine/PatternSender wiring against a controlled identity
    # calibration, unrelated to the (separately tested, see
    # tests/test_page_mapper.py) sensor-to-nozzle-bar offset feature -- a
    # nonzero *effective* offset here would shift v_mm by tens of mm and push
    # every sample out of the small target images used below.
    #
    # NOTE: PageMapper's row axis always subtracts NOZZLE_BAR_SPAN_MM/2 from
    # whatever sensor_offset_row_mm is given (that is the bar-CENTER-to-
    # nozzle-0 conversion, not "no correction"), so the value that actually
    # cancels to a zero net shift is NOZZLE_BAR_SPAN_MM/2.0, NOT 0.0 --
    # passing literal 0.0 would itself introduce a -NOZZLE_BAR_SPAN_MM/2 mm
    # shift. See tests/test_page_mapper.py for this pinned in detail.
    return PrintController(render, ble, trk, ink=ink,
                           page_calibration=_identity_calibration(),
                           drops_per_pixel=drops_per_pixel, profile=profile,
                           profile_csv=profile_csv, record=record,
                           progress_json=progress_json,
                           speed_warning_mm_s=speed_warning_mm_s,
                           sensor_offset_row_mm=NOZZLE_BAR_SPAN_MM / 2.0,
                           sensor_offset_col_mm=0.0,
                           latency_compensate_s=latency_compensate_s,
                           startpoint_anchor=startpoint_anchor,
                           progress_hz=progress_hz)


def _sweep_positions(n_cols, samples_per_col=12):
    """u_mm = 0, 1, ..., n_cols-1, each held for samples_per_col samples --
    one full column of travel per step, so every column collects a full
    dose (mm_per_column is 1.0 in these tests)."""
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


def test_simple_frame_pass_zeroes_at_the_nozzle_bar_and_prints():
    # REGRESSION: --page-frame simple's origin must land under the nozzle
    # bar, not the sensor. Zeroing at the sensor leaves the bar
    # abs(SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM - NOZZLE_BAR_SPAN_MM/2) ~=
    # 69.91mm off along v (magnitude only -- the sign/direction depends on
    # the constant's current, hardware-measured sign), every sample reads
    # out of bounds, and the pass completes "successfully" having printed
    # NOTHING -- which is exactly what the first simulated
    # simple-frame pass did. Deliberately uses the REAL sensor offsets (not
    # the neutralised ones _controller() passes) because the bug only exists
    # when the offset is nonzero.
    ink = np.ones((30, 5), dtype=bool)
    render = RenderSettings(text="simple frame test")
    trk = TrackingSettings(mode="page", page_frame="simple", mm_per_column=1.0,
                           smooth_ms=0.0, poll_hz=500.0, timeout_s=5.0)
    ctrl = PrintController(render, BleSettings(), trk, ink=ink,
                           page_calibration=PageCalibration.simple_frame(),
                           drops_per_pixel=2)
    # Sweep in tracker x from wherever the cart starts -- the frame is zeroed
    # at the first sample, so absolute placement is irrelevant by design.
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))
    null = _NullPrinthead()

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(null, tracker))
    text = out.getvalue()

    # NOT pattern_writes: the very first step() reports `changed` even for an
    # all-zero pattern, so a pass that covers nothing still writes once (seen
    # for real: "issued 1 pattern writes" alongside "Covered 0/6080"). Assert
    # on the covered pixel count, which is the thing that actually breaks.
    covered = re.search(r"Covered (\d+)/(\d+) ink pixels", text)
    assert covered, text
    assert int(covered.group(1)) > 0, (
        "simple frame covered 0 pixels -- origin is probably zeroed at the "
        "sensor instead of the nozzle bar:\n" + text)
    assert null.blank_writes == 1


def test_simple_frame_pass_does_not_mutate_a_shared_frame():
    # The pass mutates calibration.origin in place; two passes must not drift
    # (each PageCalibration.simple_frame() is independent, and the zeroing is
    # absolute rather than cumulative).
    ink = np.ones((30, 5), dtype=bool)
    render = RenderSettings(text="simple frame test")
    trk = TrackingSettings(mode="page", page_frame="simple", mm_per_column=1.0,
                           smooth_ms=0.0, poll_hz=500.0, timeout_s=5.0)
    cal = PageCalibration.simple_frame()
    ctrl = PrintController(render, BleSettings(), trk, ink=ink,
                           page_calibration=cal, drops_per_pixel=2)

    covered_counts = []
    for _ in range(2):
        out = io.StringIO()
        with redirect_stdout(out):
            asyncio.run(ctrl._print_freehand_pass(
                _NullPrinthead(),
                ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))))
        m = re.search(r"Covered (\d+)/(\d+) ink pixels", out.getvalue())
        assert m, out.getvalue()
        covered_counts.append(int(m.group(1)))
    assert all(c > 0 for c in covered_counts), covered_counts


def test_simple_frame_pinned_boresight_is_not_overwritten_at_start():
    # REGRESSION: a pass used to call PageMapper.capture_boresight
    # unconditionally at START, which would have silently clobbered an
    # operator-verified --simple-boresight reference with whatever pose the
    # cart happens to hold at THIS pass's start -- exactly the failure the
    # flag exists to prevent. The pinned quat must survive the pass
    # untouched, and the reported yaw must be measured against IT, not
    # against the ScriptedTracker's actual (different) start orientation.
    q_pinned = np.array([-0.5, -0.5, -0.51, 0.49])
    q_pinned /= np.linalg.norm(q_pinned)
    q_start = np.array([0.0, 0.0, 0.0, 1.0])  # a different pose at START

    ink = np.ones((30, 5), dtype=bool)
    render = RenderSettings(text="pinned boresight test")
    trk = TrackingSettings(mode="page", page_frame="simple", mm_per_column=1.0,
                           smooth_ms=0.0, poll_hz=500.0, timeout_s=5.0)
    cal = PageCalibration.simple_frame(boresight_quat=q_pinned)
    ctrl = PrintController(render, BleSettings(), trk, ink=ink,
                           page_calibration=cal, drops_per_pixel=2)

    positions = _sweep_positions(n_cols=5, samples_per_col=12)
    tracker = ScriptedTracker(positions, quats=[q_start] * len(positions))

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    text = out.getvalue()

    assert np.allclose(cal.boresight_quat, q_pinned), (
        "pinned boresight was overwritten by auto-capture")
    assert "using pinned yaw reference" in text, text
    assert "auto-captured" not in text, text


def test_simple_frame_pass_records_sensor_and_nozzle_paths_for_record():
    # --record's path overlay (see recording.render_coverage): the controller
    # must collect one (row, col) point per sample for both the raw sensor
    # centre and the nozzle-BAR centre (not nozzle 0), and pass them into
    # render_coverage. Patches render_coverage itself (a local import inside
    # _print_freehand_pass, re-resolved from the module each call) to
    # capture exactly what the controller handed it, rather than trying to
    # infer the paths back out of pixel colours in a rendered PNG.
    import printhead.recording as recording_module
    captured = {}
    real_render_coverage = recording_module.render_coverage

    def fake_render_coverage(printed, ink, path, sensor_path=None, nozzle_path=None,
                             sample_times=None, **kwargs):
        captured["sensor_path"] = sensor_path
        captured["nozzle_path"] = nozzle_path
        captured["sample_times"] = sample_times
        return real_render_coverage(printed, ink, path, sensor_path=sensor_path,
                                    nozzle_path=nozzle_path, sample_times=sample_times,
                                    **kwargs)

    recording_module.render_coverage = fake_render_coverage
    try:
        ink = np.ones((300, 5), dtype=bool)   # tall enough that the ~55mm sensor
                                               # lag (see below) can still land
                                               # on a valid (if empty) row
        render = RenderSettings(text="path recording test")
        # timeout_s short and real: ScriptedTracker HOLDS its last position
        # once its scripted sequence is exhausted (see its own docstring)
        # rather than ending the pass, so this always runs to the real-time
        # timeout regardless of how many positions were scripted -- keep it
        # short purely so the test itself stays fast, not to bound sample
        # count (that assertion below is a ratio/relationship check, not an
        # exact count, for exactly this reason).
        trk = TrackingSettings(mode="page", page_frame="simple", mm_per_column=1.0,
                               smooth_ms=0.0, poll_hz=500.0, timeout_s=0.05)
        with tempfile.TemporaryDirectory() as tmp:
            record_path = os.path.join(tmp, "coverage.png")
            ctrl = PrintController(render, BleSettings(), trk, ink=ink,
                                   page_calibration=PageCalibration.simple_frame(),
                                   drops_per_pixel=2, record=record_path)
            tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))
            asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    finally:
        recording_module.render_coverage = real_render_coverage

    assert "sensor_path" in captured, "render_coverage was never called"
    sensor_path, nozzle_path = captured["sensor_path"], captured["nozzle_path"]
    assert sensor_path and nozzle_path
    assert len(sensor_path) == len(nozzle_path) > 0

    sample_times = captured["sample_times"]
    assert len(sample_times) == len(sensor_path), (
        "sample_times must be recorded at the same index as both paths")
    # Not exactly 0.0: t_start is set, then read_pose()/pos_filter.update()
    # etc. run before the first sample_times.append() below, so a little
    # real wall-clock time has already elapsed by the first sample -- assert
    # "small and non-negative", not bit-exact zero.
    assert 0.0 <= sample_times[0] < 1.0, sample_times[0]
    assert all(b >= a for a, b in zip(sample_times, sample_times[1:])), (
        "elapsed time must be non-decreasing")

    # ScriptedTracker never fakes orientation (quat always None -- same
    # contract as SimulatedTracker), so yaw stays 0 for the whole pass and
    # every point differs only in v, by SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM
    # (sensor -> nozzle-bar-CENTRE is the full lever arm; sensor -> nozzle-0
    # alone would be a different, smaller distance -- see tracking.
    # PageMapper's docstring / geometry.py). Tolerance of 1: the recorded
    # row_diff is the difference of two INDEPENDENTLY rounded pixel rows
    # (sensor's own row, nozzle-centre's own row), not a single rounding of
    # the raw mm difference -- when the sensor row lands within half a pixel
    # of a rounding boundary (it does at this constant's current sign), the
    # two can differ by 1 from round(constant / pitch) even though nothing
    # is wrong; only a fixed offset that's flat-out MISSING or of the wrong
    # sign should ever produce a difference this test would miss.
    row_diffs = [n[0] - s[0] for s, n in zip(sensor_path, nozzle_path)]
    expected_row_diff = round(SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM / NOZZLE_PITCH_MM)
    assert all(abs(d - expected_row_diff) <= 1 for d in row_diffs), (
        row_diffs[:5], expected_row_diff)
    assert len(set(row_diffs)) == 1, "row_diff must be CONSTANT across a zero-yaw pass"
    # u (column): the two paths differ by exactly the column-axis offset
    # (geometry.SENSOR_TO_NOZZLE_COL_MM). That constant used to be 0, which
    # made this an equality check; it is a measured value like the row one
    # and has since moved off zero, so this now checks the SHIFT rather
    # than assuming the axis is offset-free. Same +-1 tolerance and same
    # reasoning as the row check above: each path rounds its own mm to a
    # column independently, so they can land a column apart without
    # anything being wrong.
    col_diffs = [n[1] - s[1] for s, n in zip(sensor_path, nozzle_path)]
    expected_col_diff = round(SENSOR_TO_NOZZLE_COL_MM / trk.mm_per_column)
    assert all(abs(d - expected_col_diff) <= 1 for d in col_diffs), (
        col_diffs[:5], expected_col_diff)
    # Deliberately NOT the row check's "exactly one distinct value": this
    # sweep moves along u, so the fractional part of u/mm_per_column walks
    # across a rounding boundary during the pass and the two independently
    # rounded paths legitimately land 1 column apart part of the time. v is
    # constant here, which is why the row diff above can be stricter. Two
    # adjacent values is the most a pure offset can produce -- more than
    # that would mean the shift is not constant in mm.
    assert len(set(col_diffs)) <= 2, col_diffs[:10]
    assert max(col_diffs) - min(col_diffs) <= 1, col_diffs[:10]


def test_freehand_pass_actually_covers_every_ink_pixel():
    # Same math as the pass above, but driving CoverageEngine/PageMapper
    # directly to check *what* "covered" means at the pixel level, not just
    # that the pass terminated.
    ink = np.ones((30, 5), dtype=bool)
    # Neutralised sensor offset (NOZZLE_BAR_SPAN_MM/2.0, not 0.0 -- see the
    # NOTE in _controller() above): this test drives CoverageEngine/PageMapper
    # directly against a controlled identity calibration and a small (30-row)
    # target image, unrelated to the sensor-to-nozzle-bar offset feature.
    mapper = PageMapper(_identity_calibration(),
                        sensor_offset_row_mm=NOZZLE_BAR_SPAN_MM / 2.0,
                        sensor_offset_col_mm=0.0)
    coverage = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=2)

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


# ============================== drop accounting: what actually goes over BLE
def _uniform_tracker(n_cols, samples_per_col, park=True):
    """Even sweep across ``n_cols`` columns (mm_per_column is 1.0 here), then
    optionally park far off the page so the last column stops collecting."""
    schritt = 1.0 / samples_per_col
    positions = [(i * schritt, 0.0, 0.0)
                 for i in range(int(n_cols / schritt))]
    if park:
        positions.append((0.0, 1000.0, 0.0))
    return ScriptedTracker(positions)


def test_a_uniform_region_keeps_sending_instead_of_going_quiet():
    # THE bug this whole conversion exists to fix. The old firmware held the
    # last pattern and re-fired it forever, so the client only had to write
    # when the pattern CHANGED. The firmware now fires each column once and
    # never repeats, and over a solid block the wanted-nozzle set does not
    # change from sample to sample -- so "send on change" sends twice for a
    # whole pass and the block comes out blank while coverage reports 100%.
    #
    # A solid 40-column block owes 40 * drops_per_pixel columns of ink; the
    # pattern changes about twice in it. Requiring far more writes than
    # changes is what pins the rule.
    ink = np.ones((30, 40), dtype=bool)
    ctrl = _controller(ink, drops_per_pixel=3, timeout_s=5.0)
    null = _NullPrinthead()
    asyncio.run(ctrl._print_freehand_pass(null, _uniform_tracker(40, 4)))

    assert null.pattern_columns >= 100, (
        "a solid block sent only "
        f"{null.pattern_columns} columns -- 'send only when the pattern "
        "changed' cannot ink a uniform region under a fire-once firmware")


def test_columns_sent_track_the_drops_per_pixel_budget():
    # Ink is a decision made entirely on this side now, so it has to be the
    # right amount: n_cols * drops_per_pixel, and proportional to the
    # setting rather than to the sample count.
    for dose in (1, 3, 6):
        ink = np.ones((30, 20), dtype=bool)
        ctrl = _controller(ink, drops_per_pixel=dose, timeout_s=5.0)
        null = _NullPrinthead()
        asyncio.run(ctrl._print_freehand_pass(null, _uniform_tracker(20, 5)))
        soll = 20 * dose
        assert abs(null.pattern_columns - soll) <= 0.15 * soll, (
            f"drops_per_pixel={dose}: sent {null.pattern_columns} columns "
            f"against a budget of {soll}")


def test_a_stationary_cart_sends_nothing_after_its_first_column():
    # Ink follows travel, so a parked cart is owed nothing -- this is what
    # replaced the old stall-grace anti-blob logic. It must not keep
    # dribbling columns into a stationary spot.
    ink = np.ones((30, 20), dtype=bool)
    ctrl = _controller(ink, drops_per_pixel=3, timeout_s=0.4)
    null = _NullPrinthead()
    tracker = ScriptedTracker([(0.0, 0.0, 0.0)])       # never moves
    asyncio.run(ctrl._print_freehand_pass(null, tracker))

    # Only the first sample's seeded dose (one column's worth), out of the
    # ~200 samples a 0.4 s pass at 500 Hz takes.
    assert null.pattern_columns <= 3, (
        f"a parked cart sent {null.pattern_columns} columns -- travel, not "
        "time, is supposed to decide ink")


def test_fractional_drops_are_accumulated_rather_than_truncated():
    # At 8 samples per column and drops_per_pixel=3 a sample is worth 0.375
    # columns. Truncating each sample to an integer would send NOTHING at
    # all; the accumulator has to carry the remainder across samples.
    ink = np.ones((30, 10), dtype=bool)
    ctrl = _controller(ink, drops_per_pixel=3, timeout_s=5.0)
    null = _NullPrinthead()
    asyncio.run(ctrl._print_freehand_pass(null, _uniform_tracker(10, 8)))

    assert null.pattern_columns >= 25, (
        f"sent {null.pattern_columns} columns where 30 were owed -- "
        "per-sample truncation is dropping the fractional remainder")


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


def _pj_events(ctrl, tracker):
    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    return _ndjson_events(out.getvalue())


def test_progress_events_are_throttled_but_never_drop_a_cell():
    # The whole point of the throttle: it changes the SIZE of a batch, not
    # its contents. Emitting one event per poll sample cost enough
    # per-sample work to drag the freehand loop from a nominal 500 Hz down
    # to a measured ~71 Hz on a 200x100 mm target -- and since the
    # column-skipping edge is mm_per_column * poll_hz, that was a ~6 mm/s
    # speed limit on the print itself.
    ink = np.ones((30, 5), dtype=bool)
    positionen = _sweep_positions(n_cols=5, samples_per_col=12)

    schnell = _pj_events(_controller(ink, timeout_s=5.0, progress_json=True,
                                     progress_hz=0.0),
                         ScriptedTracker(positionen))
    langsam = _pj_events(_controller(ink, timeout_s=5.0, progress_json=True,
                                     progress_hz=1.0),
                         ScriptedTracker(positionen))

    def zellen(events):
        return [tuple(c) for e in events
                if e.get("event") == "coverage" for c in e["new_cells"]]

    n_schnell = len([e for e in schnell if e.get("event") == "coverage"])
    n_langsam = len([e for e in langsam if e.get("event") == "coverage"])
    assert n_langsam < n_schnell, (n_langsam, n_schnell)

    # ...and the union is identical and still exactly-once either way.
    for label, events in (("unthrottled", schnell), ("throttled", langsam)):
        c = zellen(events)
        assert len(c) == 150, (label, len(c))
        assert len(set(c)) == 150, f"{label}: duplicates"


def test_progress_json_flushes_the_tail_of_the_pass():
    # A pass ends between two throttle ticks, so the cells inked since the
    # last event have not gone out yet. Without a final flush they never
    # would -- and it is the END of the pass that goes missing, exactly
    # where an operator looks when a print stops short.
    #
    # The rate is low enough that the pass (60 samples at 500 Hz, ~0.12 s)
    # ends well inside the first interval, so EVERYTHING after the mandatory
    # first event depends on the flush.
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0, progress_json=True, progress_hz=0.5)
    events = _pj_events(ctrl, ScriptedTracker(_sweep_positions(5, 12)))

    proben = [e for e in events if e.get("event") == "coverage"]
    assert len(proben) >= 2, "expected the immediate first event plus a flush"
    assert proben[-1]["new_cells"], "the flush carried no cells"
    zellen = [tuple(c) for e in proben for c in e["new_cells"]]
    assert len(zellen) == 150 and len(set(zellen)) == 150, len(zellen)
    # The flush must be a `coverage` event, not folded into coverage_done:
    # consumers select on event == "coverage" to collect cells.
    assert events[-1]["event"] == "coverage_done"
    assert "new_cells" not in events[-1]


def test_first_progress_event_does_not_wait_for_the_throttle():
    # At 0.5 Hz a consumer would otherwise sit blind for two seconds after
    # pressing START. The first sample always emits.
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0, progress_json=True, progress_hz=0.5)
    events = _pj_events(ctrl, ScriptedTracker(_sweep_positions(5, 12)))

    assert events[0]["event"] == "coverage_start"
    assert events[1]["event"] == "coverage", events[1]


def test_cli_progress_hz_defaults_to_none_and_parses():
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "page",
                           "--page-frame", "simple"])
    assert args.progress_hz is None        # unset -> the controller's default
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "page",
                           "--page-frame", "simple", "--progress-hz", "5"])
    assert args.progress_hz == 5.0
    ctrl = cli.build_controller(args)
    assert ctrl.progress_hz == 5.0


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
    # --mode line: this test is about --progress-json, not mode selection.
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line", "--progress-json"])
    assert args.progress_json is True
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line"])
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
    assert "columns queued" in report, report
    # The profiler must be told how many columns each send carried, or the
    # reported rate is a send rate wearing a column label.
    m = re.search(r"columns queued\s*:\s*(\d+)", report)
    assert m and int(m.group(1)) == 5 * 2, report   # 5 columns x 2 drops


def test_freehand_pass_with_profile_csv_writes_the_page_schema():
    ink = np.ones((30, 5), dtype=bool)
    with tempfile.TemporaryDirectory() as tmp:
        csv_path = os.path.join(tmp, "profile.csv")
        ctrl = _controller(ink, timeout_s=5.0, profile=True, profile_csv=csv_path)
        tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
        with open(csv_path) as fh:
            header = fh.readline().strip()
        assert header == "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,qx,qy,qz,qw"


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
        assert lines[0] == "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,qx,qy,qz,qw"
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
                        sensor_offset_row_mm=NOZZLE_BAR_SPAN_MM / 2.0,
                        sensor_offset_col_mm=0.0)
    coverage = CoverageEngine(ink, mm_per_column=1.0, drops_per_pixel=2)
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


# ====================================== poll-rate / speed-warning guard
def test_freehand_pass_warns_when_the_poll_rate_cannot_reach_the_speed_warning():
    # The drop-count model's replacement for the old dose-hold cliff. Ink no
    # longer depends on dwell, so a slow poll rate cannot thin a print -- but
    # a column the tracker never samples is never fired, so past
    # mm_per_column * poll_hz whole columns drop out. If that edge sits at or
    # below the configured speed warning, the warning can never arrive before
    # the damage does, and the operator gets silent gaps with no cause.
    #
    # Here: 1.0 mm/column at 20 Hz = 20 mm/s, under the 25 mm/s default.
    ink = np.zeros((5, 5), dtype=bool)   # nothing to cover -- pass ends immediately
    ctrl = _controller(ink, poll_hz=20.0, timeout_s=1.0)
    tracker = ScriptedTracker([(0.0, 0.0, 0.0)])

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    text = out.getvalue()

    assert "[warn]" in text, text
    assert "--poll-hz" in text and "speed warning" in text, text
    assert "20.0 mm/s" in text, text


def test_freehand_pass_does_not_warn_at_a_normal_poll_rate():
    # Guard against a false positive: at the production settings the edge is
    # far above the warning (0.087 mm/column * 500 Hz = 43.5 mm/s against
    # 25 mm/s), and this test's 1.0 mm/column * 500 Hz is further still.
    #
    # NOTE: _controller() -> _identity_calibration() has no boresight_quat,
    # so the SEPARATE "no rotation correction" warning (see
    # test_freehand_pass_warns_about_a_missing_boresight below) is still
    # expected in this output -- this test only pins the absence of the
    # poll-rate warning specifically, not "[warn]" in general.
    ink = np.zeros((5, 5), dtype=bool)
    ctrl = _controller(ink, poll_hz=500.0, timeout_s=1.0)
    tracker = ScriptedTracker([(0.0, 0.0, 0.0)])

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    text = out.getvalue()

    assert "--poll-hz" not in text and "speed warning" not in text, text


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
    # --progress-json must stay pure NDJSON (mirrors how the poll-rate and
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


def test_progress_json_suppresses_the_poll_rate_warning_too():
    # --progress-json must stay pure NDJSON (mirrors how the out-of-page
    # warning is suppressed in that mode) even when the poll-rate condition
    # holds.
    ink = np.zeros((5, 5), dtype=bool)
    ctrl = _controller(ink, poll_hz=20.0, timeout_s=1.0,
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
                           drops_per_pixel=2, progress_json=True,
                           sensor_offset_row_mm=NOZZLE_BAR_SPAN_MM / 2.0,
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
    # Same path, same target image, same dose -- the ONLY difference is
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
        # drops_per_pixel=1 -> the very first in-bounds sample already doses
        # its pixels, so coverage.printed is non-empty even though only a
        # few samples run before the simulated interruption -- keeps the
        # "record was attempted and had something to draw" check deterministic.
        ctrl = _controller(ink, timeout_s=5.0, drops_per_pixel=1,
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
        assert header == "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,qx,qy,qz,qw"

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
    # --mode line throughout this block: these tests are about the
    # individual flags below, not mode selection, and pass no
    # --page-calibration.
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line"])
    assert args.ble_write_ceiling is None
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line",
                           "--ble-write-ceiling", "150"])
    assert args.ble_write_ceiling == 150.0


def test_cli_sensor_offset_flags_default_to_none_and_parse():
    # Same "default None -> controller falls back to the geometry constant"
    # pattern as --drops-per-pixel / --ble-write-ceiling above.
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line"])
    assert args.sensor_offset_row_mm is None
    assert args.sensor_offset_col_mm is None
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line",
                           "--sensor-offset-row-mm", "70.0",
                           "--sensor-offset-col-mm", "-3.5"])
    assert args.sensor_offset_row_mm == 70.0
    assert args.sensor_offset_col_mm == -3.5


def test_cli_sensor_offset_flags_reach_the_controller_when_given():
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line",
                           "--sensor-offset-row-mm", "70.0",
                           "--sensor-offset-col-mm", "-3.5"])
    ctrl = cli.build_controller(args)
    assert ctrl.sensor_offset_row_mm == 70.0
    assert ctrl.sensor_offset_col_mm == -3.5


def test_cli_sensor_offset_flags_default_to_the_geometry_constants_unset():
    # When not given at all, the controller must fall back to the real
    # measured geometry constants, not to 0.0.
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line"])
    ctrl = cli.build_controller(args)
    assert ctrl.sensor_offset_row_mm == SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM
    assert ctrl.sensor_offset_col_mm == SENSOR_TO_NOZZLE_COL_MM


# =============================================================== --boresight-deg
def test_cli_boresight_deg_defaults_to_none_and_parses():
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line"])
    assert args.boresight_deg is None
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line",
                           "--boresight-deg", "3.5"])
    assert args.boresight_deg == 3.5


def test_cli_boresight_deg_reaches_the_controller_when_given():
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line",
                           "--boresight-deg", "-7.25"])
    ctrl = cli.build_controller(args)
    assert ctrl.boresight_deg == -7.25


def test_cli_boresight_deg_defaults_to_zero_unset():
    # Same "default None on the CLI -> 0.0 (neutral) on the controller"
    # pattern as the other page-mode fine-tune flags.
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line"])
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
    # --mode line: the point here is exercising the "no --page-calibration
    # given" path through build_page_calibration() itself; --mode page would
    # instead be rejected earlier, by parse_args's own validation.
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line"])
    assert cli.build_page_calibration(args) is None


def test_cli_speed_warning_mm_s_defaults_to_none_and_parses():
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line"])
    assert args.speed_warning_mm_s is None
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line",
                           "--speed-warning-mm-s", "30"])
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


# ============================================================= --verbose
def test_verbose_prints_a_live_status_line_while_printing():
    # The --pos equivalent, but usable while an actual pass is running (see
    # this method's docstring) -- must show page position and yaw/roll/pitch,
    # not just bare BLE write logging (that's --mode time's older, unrelated
    # meaning of the same flag).
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0, verbose=True)
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    text = out.getvalue()

    assert "page u=" in text and "yaw=" in text, text
    assert "covered " in text, text


def test_verbose_off_by_default_prints_no_status_line():
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0)          # verbose=False
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    assert "page u=" not in out.getvalue()


def test_verbose_status_line_is_suppressed_in_progress_json_mode():
    # progress_json must stay pure NDJSON for the UI consumer (see the
    # method's docstring) -- --verbose must not leak a plain-text `\r` line
    # into that stream even when both flags are set together.
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0, verbose=True, progress_json=True)
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    for line in out.getvalue().splitlines():
        if line.strip():
            json.loads(line)          # every non-blank line must be valid JSON


def test_verbose_status_line_covered_counts_never_decrease_and_stay_in_bounds():
    # The --verbose line is throttled (_VERBOSE_STATUS_INTERVAL_S), so its
    # last printed snapshot is not guaranteed to land on the exact same
    # sample that later flips coverage.done -- do NOT assert exact equality
    # against the final "Covered N/M" tally (that is timing-dependent and
    # flaky by construction). Instead check what must ALWAYS hold: every
    # line reports the same (constant) total, "covered" never goes
    # backwards across samples, and it never exceeds the true final count --
    # i.e. the live number genuinely tracks CoverageEngine.printed, not a
    # separately-computed or stale counter.
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0, verbose=True)
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    text = out.getvalue()

    final = re.search(r"Covered (\d+)/(\d+) ink pixels", text)
    assert final, text
    final_covered, final_total = int(final.group(1)), int(final.group(2))

    verbose_lines = [l for l in re.split(r"[\r\n]+", text) if "covered " in l]
    assert verbose_lines, text
    counts = [tuple(map(int, re.search(r"covered (\d+)/(\d+)", l).groups()))
             for l in verbose_lines]

    assert all(total == final_total for _, total in counts), (counts, final_total)
    covered_values = [c for c, _ in counts]
    assert covered_values == sorted(covered_values), covered_values
    assert covered_values[-1] <= final_covered, (covered_values, final_covered)


# ================================== STARTPOINT button (page mode: place/stop)
class StopAfter:
    """STARTPOINT-button stub whose is_set() starts returning True on the
    k-th call, mirroring test_position_pass.py's FireOnce (line mode's
    re-zero press) for the page-mode STOP press."""

    def __init__(self, at_check):
        self._checks = 0
        self._at = at_check
        self._fired = False

    def is_set(self):
        self._checks += 1
        if self._checks >= self._at:
            self._fired = True
        return self._fired

    def clear(self):
        self._fired = False


def test_startpoint_press_stops_a_running_pass():
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0)
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))
    null = _NullPrinthead()

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(null, tracker, StopAfter(at_check=4)))
    text = out.getvalue()

    assert "[startpoint] pass stopped by button press." in text, text
    # Stopped early, so NOT everything got covered -- this is what separates
    # a real stop from the check simply never firing (the pass otherwise
    # reaches "Page fully covered." on this sweep, see
    # test_freehand_pass_covers_the_page_and_stops_before_timeout).
    assert "Page fully covered." not in text, text
    m = re.search(r"Covered (\d+)/(\d+) ink pixels", text)
    assert m, text
    assert int(m.group(1)) < int(m.group(2)), m.group(0)


def test_startpoint_stop_still_runs_the_normal_cleanup():
    # The stop breaks out of the loop, so the usual `finally` must still
    # blank the head -- a stopped pass that leaves nozzles firing would be
    # worse than not having the button at all.
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0)
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))
    null = _NullPrinthead()

    with redirect_stdout(io.StringIO()):
        asyncio.run(ctrl._print_freehand_pass(null, tracker, StopAfter(at_check=4)))

    assert null.blank_writes == 1, null.blank_writes
    assert null.speed_warnings[-1] is False, null.speed_warnings


def test_startpoint_stop_reports_a_stopped_reason_in_progress_json():
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0, progress_json=True)
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker,
                                              StopAfter(at_check=4)))
    events = [json.loads(l) for l in out.getvalue().splitlines() if l.strip()]
    done = [e for e in events if e["event"] == "coverage_done"]
    assert done and done[-1]["reason"] == "stopped", events[-3:]


def test_startpoint_stop_skips_the_misleading_out_of_page_diagnosis():
    # A deliberate early stop before ever reaching the page must not be
    # reported as "the calibration origin/axes do not correspond to where
    # you think the page corner is" -- that advice is about a different bug.
    ink = np.ones((5, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0)
    tracker = ScriptedTracker([(0.0, 1000.0, 0.0)])      # far off the page

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker,
                                              StopAfter(at_check=4)))
    text = out.getvalue()
    assert "never overlapped the target page" not in text, text
    assert "Covered 0/25 ink pixels." in text, text


def test_startpoint_stop_MUTATION_check_without_the_break_the_pass_runs_on():
    # Confirms the test above measures the stop and not just a short pass:
    # the SAME sweep with no startpoint event at all reaches full coverage.
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0)
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    text = out.getvalue()
    assert "Page fully covered." in text, text
    assert "[startpoint]" not in text, text


def test_set_page_origin_moves_only_the_origin_and_zeroes_the_held_pose():
    # The whole point of the idle press: place WHERE the image is CENTRED
    # without touching the traced plane definition (axes/scales) the
    # calibration file exists to provide.
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink)
    cal = ctrl.page_calibration
    e_col_before, e_row_before = cal.e_col.copy(), cal.e_row.copy()
    scales_before = (cal.scale_col, cal.scale_row)
    held = (12.0, 34.0, 0.0)

    with redirect_stdout(io.StringIO()):
        asyncio.run(ctrl._set_page_origin(ScriptedTracker([held])))

    assert np.allclose(cal.e_col, e_col_before)
    assert np.allclose(cal.e_row, e_row_before)
    assert (cal.scale_col, cal.scale_row) == scales_before
    assert ctrl._page_origin_pinned is True

    # zero_at_nozzle, not set_origin: the pose held at the press must project
    # to the image's CENTRE (not the corner) through the SAME offsets the
    # pass uses, i.e. the nozzle bar lands on the pattern's middle, not the
    # sensor ~62mm away and not the top-left corner.
    mapper = PageMapper(cal, sensor_offset_row_mm=NOZZLE_BAR_SPAN_MM / 2.0,
                        sensor_offset_col_mm=0.0)
    u, v, _ = mapper.project(np.array(held, dtype=float))
    expected_u = (ctrl.width - 1) / 2.0 * ctrl.tracking.mm_per_column
    expected_v = (ctrl.height - 1) / 2.0 * NOZZLE_PITCH_MM
    assert abs(u - expected_u) < 1e-9 and abs(v - expected_v) < 1e-9, \
        (u, v, expected_u, expected_v)


# --------------------------------------------------- --startpoint-anchor
def _placed_uv(ctrl, held=(12.0, 34.0, 0.0)):
    """Run _set_page_origin() and return the (u, v) the held pose now
    projects to -- i.e. where the STARTPOINT press actually landed."""
    cal = ctrl.page_calibration
    with redirect_stdout(io.StringIO()):
        asyncio.run(ctrl._set_page_origin(ScriptedTracker([held])))
    mapper = PageMapper(cal, sensor_offset_row_mm=NOZZLE_BAR_SPAN_MM / 2.0,
                        sensor_offset_col_mm=0.0)
    return mapper.project(np.array(held, dtype=float))[:2]


def test_startpoint_anchor_left_middle_places_the_left_edge_not_the_centre():
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, startpoint_anchor="left-middle")
    u, v = _placed_uv(ctrl)
    expected_v = (ctrl.height - 1) / 2.0 * NOZZLE_PITCH_MM
    assert abs(u - 0.0) < 1e-9, u                # left edge, not centred
    assert abs(v - expected_v) < 1e-9, (v, expected_v)   # still vertically centred


def test_startpoint_anchor_top_left_places_the_literal_corner():
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, startpoint_anchor="top-left")
    u, v = _placed_uv(ctrl)
    assert abs(u - 0.0) < 1e-9 and abs(v - 0.0) < 1e-9, (u, v)


def test_startpoint_anchor_default_is_center():
    # Omitting the option must reproduce today's only behaviour exactly --
    # existing calibration workflows must not change under them.
    from printhead.controller import DEFAULT_STARTPOINT_ANCHOR
    assert DEFAULT_STARTPOINT_ANCHOR == "center"
    ink = np.ones((30, 5), dtype=bool)
    assert _controller(ink).startpoint_anchor == "center"


def test_startpoint_anchor_message_names_the_anchor_used():
    # The operator has to be able to tell, from the printed confirmation
    # alone, which point of the pattern just landed under the nozzle bar.
    ink = np.ones((30, 5), dtype=bool)
    for anchor, phrase in (("center", "CENTRE"),
                          ("left-middle", "LEFT EDGE"),
                          ("top-left", "TOP-LEFT CORNER")):
        ctrl = _controller(ink, startpoint_anchor=anchor)
        out = io.StringIO()
        with redirect_stdout(out):
            asyncio.run(ctrl._set_page_origin(ScriptedTracker([(1.0, 2.0, 0.0)])))
        assert phrase in out.getvalue(), (anchor, out.getvalue())


def test_startpoint_anchor_rejects_an_unknown_value():
    ink = np.ones((30, 5), dtype=bool)
    try:
        _controller(ink, startpoint_anchor="bottom-right")
    except ValueError as exc:
        assert "bottom-right" in str(exc)
    else:
        raise AssertionError("expected ValueError for an unknown anchor")


def test_cli_startpoint_anchor_defaults_to_none_and_parses():
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "page",
                           "--page-frame", "simple"])
    assert args.startpoint_anchor is None      # unset -> the controller's default
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "page",
                           "--page-frame", "simple",
                           "--startpoint-anchor", "top-left"])
    assert args.startpoint_anchor == "top-left"
    ctrl = cli.build_controller(args)
    assert ctrl.startpoint_anchor == "top-left"


def test_cli_startpoint_anchor_rejects_an_unknown_choice():
    try:
        cli.parse_args(["Hi", "--dry-run", "--mode", "page",
                        "--page-frame", "simple",
                        "--startpoint-anchor", "bottom-right"])
    except SystemExit:
        pass
    else:
        raise AssertionError("argparse must reject an unknown --startpoint-anchor")


def test_set_page_origin_survives_a_tracker_that_never_yields_a_pose():
    # A button press must never tear down the BLE session; the idle loop has
    # to keep waiting for START instead.
    class DeadTracker:
        def open(self): pass
        def close(self): pass
        def read_position(self): return None
        def read_pose(self): return None, None

    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink)
    origin_before = ctrl.page_calibration.origin.copy()

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._set_page_origin(DeadTracker()))

    assert "origin NOT placed" in out.getvalue(), out.getvalue()
    assert ctrl._page_origin_pinned is False
    assert np.allclose(ctrl.page_calibration.origin, origin_before)


def _run_idle_wait(ctrl, mode, tracker):
    """Drive _wait_for_start_press with a STARTPOINT already latched and a
    START arriving shortly after; returns (returned_after_start, startpoint)."""
    async def scenario():
        press, startpoint = asyncio.Event(), asyncio.Event()
        startpoint.set()                       # STARTPOINT arrives first
        async def press_later():
            await asyncio.sleep(0.05)
            press.set()
        later = asyncio.ensure_future(press_later())
        await ctrl._wait_for_start_press(press, startpoint, tracker, mode)
        returned_after_start = press.is_set()
        await later
        return returned_after_start, startpoint.is_set()
    return asyncio.run(scenario())


def test_idle_startpoint_places_the_origin_and_keeps_waiting_for_start():
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink)

    out = io.StringIO()
    with redirect_stdout(out):
        returned_after_start, _ = _run_idle_wait(ctrl, "page",
                                                 ScriptedTracker([(5.0, 6.0, 0.0)]))

    # Did NOT return on the startpoint press alone (it waited for START), and
    # placed the origin while waiting.
    assert returned_after_start is True
    assert ctrl._page_origin_pinned is True
    assert "page origin placed" in out.getvalue(), out.getvalue()


def test_idle_startpoint_is_left_alone_outside_page_mode():
    # Line mode gives the same button an established, different meaning
    # (--origin startpoint waits on this very event at pass start), so the
    # idle handler must not consume the press or place anything.
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink)

    out = io.StringIO()
    with redirect_stdout(out):
        returned_after_start, still_set = _run_idle_wait(
            ctrl, "line", ScriptedTracker([(5.0, 6.0, 0.0)]))

    assert returned_after_start is True
    assert still_set is True, "line mode must leave the latched press alone"
    assert ctrl._page_origin_pinned is False
    assert "page origin placed" not in out.getvalue()


def test_simple_frame_placed_origin_is_not_rezeroed_at_start():
    # REGRESSION guard: the simple frame re-zeros at whatever pose START
    # catches. Once the operator has placed an origin with the button, that
    # blind re-zero would silently throw the placement away.
    ink = np.ones((30, 5), dtype=bool)
    render = RenderSettings(text="placed origin test")
    trk = TrackingSettings(mode="page", page_frame="simple", mm_per_column=1.0,
                           smooth_ms=0.0, poll_hz=500.0, timeout_s=1.0)
    cal = PageCalibration.simple_frame()
    ctrl = PrintController(render, BleSettings(), trk, ink=ink,
                           page_calibration=cal, drops_per_pixel=2)

    with redirect_stdout(io.StringIO()):
        asyncio.run(ctrl._set_page_origin(ScriptedTracker([(5.0, 6.0, 0.0)])))
    placed_origin = cal.origin.copy()

    # Pass starts from a DIFFERENT position: a re-zero would move the origin.
    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(
            _NullPrinthead(), ScriptedTracker([(80.0, 90.0, 0.0)])))

    assert np.allclose(cal.origin, placed_origin), (cal.origin, placed_origin)
    assert "placed with the STARTPOINT button" in out.getvalue(), out.getvalue()


def test_simple_frame_without_a_placed_origin_still_zeroes_at_start():
    # The unpinned path must keep working exactly as before.
    ink = np.ones((30, 5), dtype=bool)
    render = RenderSettings(text="unpinned origin test")
    trk = TrackingSettings(mode="page", page_frame="simple", mm_per_column=1.0,
                           smooth_ms=0.0, poll_hz=500.0, timeout_s=1.0)
    cal = PageCalibration.simple_frame()
    ctrl = PrintController(render, BleSettings(), trk, ink=ink,
                           page_calibration=cal, drops_per_pixel=2)

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(
            _NullPrinthead(), ScriptedTracker([(80.0, 90.0, 0.0)])))
    text = out.getvalue()
    assert "page origin zeroed at the nozzle bar's current position" in text, text
    assert "placed with the STARTPOINT button" not in text, text


def test_verbose_status_line_does_not_garble_the_final_message():
    # REGRESSION guard: the status line ends every write with `\r`, not
    # `\n` (see the throttled block above), so without the newline flush
    # right before "Page fully covered."/"Freehand pass timed out." (and the
    # finally-block fallback for the exception path), that message would
    # land on top of the partial status line instead of a fresh one.
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0, verbose=True)
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    text = out.getvalue()
    # The line immediately preceding "Finished pass..." must start clean
    # (preceded by \n, not by a bare \r) -- assert on the exact boundary.
    assert re.search(r"\n(Page fully covered\.|Freehand pass timed out\.)\n"
                     r"Finished pass; sent blank frame\.", text), text


# ============================ pass reporting counts physical ink, not dose
# A pass whose LAST column ends up inked but under-dosed. Under the
# drop-count model that is the only way a swept column comes out thin: ink is
# derived from travel, so a crossing the cart actually completes always
# delivers the full dose, at any speed. To produce one deliberately, the cart
# crosses columns 0..38 cleanly at 4 samples per column, takes a SINGLE
# sample into column 39, and then parks far off the page -- without that park
# the last column would keep collecting samples and complete after all.
#
# 4 samples per column at drops_per_pixel=4 means one drop per sample, so the
# single sample leaves column 39 on 1 of 4 -- clear of the one-sample report
# slack in CoverageEngine.step()'s Step 5, which would otherwise (correctly)
# call a column one drop short finished.
_PARTIAL_PASS = dict(drops_per_pixel=4, poll_hz=500.0, timeout_s=1.0)
_PARTIAL_COLS = 40


def _partial_tracker(n_cols=_PARTIAL_COLS, per_col=4):
    # mm_per_column is 1.0 in these tests, so one column == 1.0 mm and
    # column k owns u in [k-0.5, k+0.5). Sweep in even steps and cut the
    # moment the FIRST sample lands in the last column, so that column is
    # left holding exactly one step's worth of dose.
    schritt = 1.0 / per_col
    letzte = n_cols - 1
    positions = []
    i = 0
    while True:
        u = i * schritt
        positions.append((u, 0.0, 0.0))
        if round(u) >= letzte:
            break
        i += 1
    positions.append((0.0, 1000.0, 0.0))          # park far off the page
    return ScriptedTracker(positions)


def test_pass_end_covered_count_reports_physical_ink_not_dose_completion():
    # Reported from hardware: the real print's fill was perfect while
    # coverage.png (and the count beside it) showed heavy gaps, because
    # `printed` only marks a pixel once its dose completes. The pass-end
    # count must describe the paper -- every column the cart swept received
    # ink, including the one it only half crossed.
    ink = np.ones((30, _PARTIAL_COLS), dtype=bool)
    ctrl = _controller(ink, **_PARTIAL_PASS)
    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), _partial_tracker()))
    text = out.getvalue()

    m = re.search(r"Covered (\d+)/(\d+) ink pixels", text)
    assert m, text
    assert int(m.group(1)) == int(m.group(2)), (
        "every swept pixel received ink, so the count must be complete -- "
        "coming up short here means the count came from the dose mask:\n"
        + text)


def test_pass_end_flags_inked_but_underdosed_pixels_separately():
    # "inked but light" and "missed entirely" need different corrections, so
    # the summary says which one happened rather than folding them together.
    ink = np.ones((30, _PARTIAL_COLS), dtype=bool)
    ctrl = _controller(ink, **_PARTIAL_PASS)
    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), _partial_tracker()))
    text = out.getvalue()
    assert "fewer than --drops-per-pixel drops" in text, text
    # Exactly the one half-crossed column, not a speed-dependent smear.
    m = re.search(r"of those, (\d+) got fewer", text)
    assert m and int(m.group(1)) == 30, text


def test_pass_end_omits_the_underdose_note_when_every_dose_completed():
    # On a pass that crosses every column cleanly the two counts agree and
    # the extra line would be pure noise.
    ink = np.ones((30, 5), dtype=bool)
    ctrl = _controller(ink, timeout_s=5.0)
    tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
    text = out.getvalue()
    assert "Page fully covered." in text, text
    assert "fewer than --drops-per-pixel drops" not in text, text


def test_progress_json_carries_both_the_inked_and_full_dose_counts():
    ink = np.ones((30, _PARTIAL_COLS), dtype=bool)
    ctrl = _controller(ink, progress_json=True, **_PARTIAL_PASS)
    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), _partial_tracker()))
    events = [json.loads(l) for l in out.getvalue().splitlines() if l.strip()]
    done = [e for e in events if e["event"] == "coverage_done"][-1]
    # Decisive: the two must be REPORTED SEPARATELY and actually differ here
    # -- a `covered` taken from the dose mask would equal full_dose.
    assert done["covered"] > done["full_dose"], done
    assert done["covered"] - done["full_dose"] == 30, done   # one thin column


def test_verbose_live_count_agrees_with_the_pass_end_count():
    # The live --verbose count and the pass-end summary must describe the
    # SAME quantity. On a pass where every dose completes they agree
    # trivially, so this pins the case where they could diverge: a live count
    # read from the dose mask would leave out the half-crossed column the
    # summary includes -- exactly the contradiction that made the coverage
    # report untrustworthy.
    ink = np.ones((30, _PARTIAL_COLS), dtype=bool)
    ctrl = _controller(ink, verbose=True, **_PARTIAL_PASS)
    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), _partial_tracker()))
    text = out.getvalue()

    final = re.search(r"Covered (\d+)/(\d+) ink pixels", text)
    assert final, text
    live = [re.search(r"covered (\d+)/(\d+)", l)
            for l in re.split(r"[\r\n]+", text) if "covered " in l]
    live = [m for m in live if m]
    assert live, text
    assert live[-1].groups() == final.groups(), (live[-1].group(0), final.group(0))


def test_record_is_given_the_fired_mask_so_the_image_matches_the_paper():
    # Guards the wiring specifically: the controller must hand
    # render_coverage the physical-ink mask, not just the dose mask.
    from printhead import recording as recording_module
    captured = {}
    real = recording_module.render_coverage

    def fake(printed, ink, path, **kw):
        captured["fired"] = kw.get("fired")
        return real(printed, ink, path, **kw)

    ink = np.ones((30, 5), dtype=bool)
    with tempfile.TemporaryDirectory() as tmp:
        ctrl = _controller(ink, timeout_s=5.0,
                           record=os.path.join(tmp, "coverage.png"))
        tracker = ScriptedTracker(_sweep_positions(n_cols=5, samples_per_col=12))
        recording_module.render_coverage = fake
        try:
            with redirect_stdout(io.StringIO()):
                asyncio.run(ctrl._print_freehand_pass(_NullPrinthead(), tracker))
        finally:
            recording_module.render_coverage = real

    assert captured.get("fired") is not None, \
        "render_coverage was called without the fired mask"



# ==================================================== --latency-compensate-s
def test_extrapolate_uv_is_a_noop_at_zero_latency():
    # 0.0 is the default and must be bit-identical to never calling this at
    # all -- not just "adds 0.0", but literally returns the same values.
    assert _extrapolate_uv(12.5, -3.2, 999.0, -999.0, 0.0) == (12.5, -3.2)


def test_extrapolate_uv_projects_forward_along_velocity():
    # Direct formula pin: u/v shift by velocity * latency, independently
    # per axis (unequal vx/vy, unequal signs, so a swapped-axis bug would
    # show up immediately).
    u, v = _extrapolate_uv(u_mm=10.0, v_mm=5.0, vx_mm_s=20.0, vy_mm_s=-4.0,
                           latency_s=0.1)
    assert abs(u - 12.0) < 1e-9, u          # 10.0 + 20.0*0.1
    assert abs(v - 4.6) < 1e-9, v           # 5.0 + (-4.0)*0.1


def test_extrapolate_uv_negative_latency_projects_backward():
    # Deliberately not clamped to >= 0 -- an operator experimenting with the
    # sign gets exactly what they asked for (see the function's docstring).
    u, v = _extrapolate_uv(u_mm=10.0, v_mm=5.0, vx_mm_s=20.0, vy_mm_s=-4.0,
                           latency_s=-0.5)
    assert abs(u - 0.0) < 1e-9, u           # 10.0 + 20.0*(-0.5)
    assert abs(v - 7.0) < 1e-9, v           # 5.0 + (-4.0)*(-0.5)


def test_extrapolate_uv_MUTATION_check_ignoring_v_reintroduces_the_swap_bug():
    # Confirms the two axes are actually independent in the implementation,
    # not one shared scalar accidentally applied to both.
    u, v = _extrapolate_uv(0.0, 0.0, vx_mm_s=1.0, vy_mm_s=2.0, latency_s=1.0)
    assert (u, v) == (1.0, 2.0)
    assert u != v, "vx != vy here, so a per-axis bug would make these equal"


def test_latency_compensate_reaches_pixels_the_real_position_never_touches():
    # Wiring + effect, end to end: a cart that (for real) only ever reaches
    # u=0..50mm must not ink anything at u=51..90mm on its own -- but WITH
    # compensation, the extrapolated fire position briefly does reach past
    # 50mm while the cart is still moving fast, which must ink cells there.
    # The exact shift depends on real wall-clock sample timing (jitter), so
    # this asserts a wide target range and a generous latency rather than an
    # exact column -- see the inline math for why the margin holds even
    # under realistic jitter.
    ink = np.zeros((10, 90), dtype=bool)
    ink[:, 51:90] = True                          # only reachable by extrapolation
    positions = [(float(c), 0.0, 0.0) for c in range(51)]     # u = 0..50mm

    def run(latency_s):
        captured = {}
        from printhead import recording as recording_module
        real = recording_module.render_coverage

        def fake(printed, ink_, path, **kw):
            captured["fired"] = kw.get("fired")
            return real(printed, ink_, path, **kw)

        with tempfile.TemporaryDirectory() as tmp:
            ctrl = _controller(ink, timeout_s=0.3,
                               record=os.path.join(tmp, "coverage.png"),
                               latency_compensate_s=latency_s)
            recording_module.render_coverage = fake
            try:
                with redirect_stdout(io.StringIO()):
                    asyncio.run(ctrl._print_freehand_pass(
                        _NullPrinthead(), ScriptedTracker(positions)))
            finally:
                recording_module.render_coverage = real
        return captured["fired"]

    uncompensated = run(0.0)
    # 0.1s is deliberately much larger than any real pipeline delay -- a
    # generous margin against real wall-clock sample-timing jitter is more
    # important here than a realistic value (realistic values belong in the
    # CLI help text, not in a test that must not flake).
    compensated = run(0.1)

    assert not uncompensated[:, 51:90].any(),         "the real sweep never reaches past u=50mm -- must not ink there"
    assert compensated[:, 51:90].any(),         "compensation should reach ahead of the real position into 51..90mm"


def test_latency_compensate_off_by_default_matches_omitting_it():
    ink = np.ones((30, 5), dtype=bool)
    ctrl_default = _controller(ink, timeout_s=5.0)                          # omitted
    ctrl_explicit = _controller(ink, timeout_s=5.0, latency_compensate_s=0.0)
    assert ctrl_default.latency_compensate_s == ctrl_explicit.latency_compensate_s == 0.0


def test_cli_drops_per_pixel_accepts_a_fraction_and_reaches_the_controller():
    # The dose is a density (drops/mm once divided by --mm-per-column), so
    # whole drops are too coarse a dial: parsing this as an int would make
    # the only step below the default "no ink at all", and would silently
    # truncate 0.5 to 0.
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "page",
                           "--page-frame", "simple"])
    assert args.drops_per_pixel is None      # unset -> the engine's default
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "page",
                           "--page-frame", "simple",
                           "--drops-per-pixel", "0.5"])
    assert args.drops_per_pixel == 0.5
    ctrl = cli.build_controller(args)
    assert ctrl.drops_per_pixel == 0.5


def test_cli_latency_compensate_s_defaults_to_none_and_parses():
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line"])
    assert args.latency_compensate_s is None
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line",
                           "--latency-compensate-s", "0.013"])
    assert args.latency_compensate_s == 0.013


def test_cli_latency_compensate_s_reaches_the_controller_when_given():
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line",
                           "--latency-compensate-s", "0.02"])
    ctrl = cli.build_controller(args)
    assert ctrl.latency_compensate_s == 0.02


def test_cli_latency_compensate_s_defaults_to_zero_unset():
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line"])
    ctrl = cli.build_controller(args)
    assert ctrl.latency_compensate_s == 0.0



if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All freehand-pass tests passed.")
