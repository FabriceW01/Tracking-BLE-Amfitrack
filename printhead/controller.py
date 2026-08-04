"""
Print controller
=================

Ties rendering, BLE transport and Amfitrack tracking together and runs a print
pass in one of three modes:

  * ``line`` - 1D closed loop: read the sensor position, convert it to a column
    index and fire that column. The horizontal scale is set by ``mm_per_column``
    and no longer depends on the cart speed.
  * ``page`` - freehand 2D closed loop: the cart can move anywhere over a
    calibrated page; a per-nozzle coverage engine decides what still needs ink,
    and the current pattern is streamed "latest wins" (see ``pattern_sender.py``)
    rather than a queue of distinct columns.
  * ``time`` - legacy: stream one column every ``period`` seconds.
"""

from __future__ import annotations

import asyncio
from typing import Optional

import numpy as np

from .ble_client import PrintheadBLE
from .calibration import PageCalibration
from .config import BleSettings, NozzleMapSettings, RenderSettings, TrackingSettings
from .coverage import DEFAULT_DOSE_HOLD_S, CoverageEngine
from .geometry import BLANK_FRAME, NUM_NOZZLES
from .nozzle_map import remap_rows
from .pattern_sender import PatternSender
from .rendering import frames_from_ink, render_text, save_preview
from .tracking import AdvanceMapper, PageMapper, PositionFilter, make_tracker

# How long the head may sit still (having accumulated < min_move_mm) before we
# stop firing its column. Tolerates slow feed while preventing a stationary blob.
_STALL_GRACE_S = 0.2


class _NullPrinthead:
    """Stand-in for PrintheadBLE used by ``--dry-run --simulate`` (no BLE)."""

    def __init__(self):
        self.column_writes = 0
        self.blank_writes = 0
        self.pattern_writes = 0

    async def write_column(self, frame):
        self.column_writes += 1

    async def write_columns(self, frames):
        self.column_writes += len(list(frames))

    async def write_blank(self):
        self.blank_writes += 1

    async def write_pattern(self, pattern):
        self.pattern_writes += 1


class _ImmediateEvent:
    """asyncio.Event lookalike whose wait() returns at once (simulation only)."""

    async def wait(self):
        return True

    def is_set(self):
        return False

    def set(self):
        pass

    def clear(self):
        pass


class PrintController:
    def __init__(self, render: RenderSettings, ble: BleSettings,
                 tracking: TrackingSettings, simulate: bool = False,
                 preview: Optional[str] = None, dry_run: bool = False,
                 ink: Optional[np.ndarray] = None,
                 nozzle_map: Optional[NozzleMapSettings] = None,
                 profile: bool = False, profile_csv: Optional[str] = None,
                 record: Optional[str] = None,
                 page_calibration: Optional[PageCalibration] = None,
                 dose_hold_s: float = DEFAULT_DOSE_HOLD_S):
        self.render = render
        self.ble = ble
        self.tracking = tracking
        self.simulate = simulate
        self.preview = preview
        self.dry_run = dry_run
        self.profile = profile
        self.profile_csv = profile_csv
        self.record = record
        self.page_calibration = page_calibration
        self.dose_hold_s = dose_hold_s

        # Rendered once up front, unless the caller already built the ink
        # (calibration ruler / test patterns bypass text rendering entirely).
        if ink is None:
            ink = render_text(render)
        if nozzle_map is not None and nozzle_map.block_size:
            ink = remap_rows(ink, nozzle_map.block_size, nozzle_map.order)
        self._ink = ink
        self.height, self.width = ink.shape

        if tracking.mode == "page":
            # Page mode doses live from this array via CoverageEngine, so it
            # is never packed into fixed per-column frames -- unlike
            # frames_from_ink(), it is not capped to IMAGE_HEIGHT rows, which
            # is exactly what lets an image taller than the 152-nozzle bar be
            # reached through vertical travel.
            self.frames = None
            extra = f" (> {NUM_NOZZLES} nozzles -> needs vertical travel)" \
                if self.height > NUM_NOZZLES else ""
            print(f"Rendered '{render.text}' -> {self.width} columns x "
                  f"{self.height} rows{extra}")
        else:
            self.frames = frames_from_ink(ink)
            print(f"Rendered '{render.text}' -> {self.width} columns x "
                  f"{self.height} rows")

    # ------------------------------------------------------------------ run
    async def run(self) -> None:
        if self.preview:
            save_preview(self._ink, self.preview)
            print(f"Preview written to {self.preview}")

        if self.dry_run:
            if self.simulate and self.tracking.mode == "line":
                await self._dry_run_line_pass()
            elif self.simulate and self.tracking.mode == "page":
                await self._dry_run_freehand_pass()
            print("Dry run: not connecting to BLE.")
            return

        if self.tracking.mode == "page":
            if not self._ink.any():
                print("Nothing to send.")
                return
        elif not self.frames:
            print("Nothing to send.")
            return

        await self._run_ble()

    # -------------------------------------------------------------- BLE run
    async def _run_ble(self) -> None:
        press_event = asyncio.Event()
        startpoint_event = asyncio.Event()
        state = {"busy": False}

        def on_start(val):
            print(f"[start-btn] {val}")
            if val == 1 and not state["busy"]:      # rising edge only
                press_event.set()

        def on_startpoint(val):
            print(f"[startpoint] {val}")
            if val == 1:
                startpoint_event.set()

        tracker = None
        mode = self.tracking.mode
        use_tracker = self.tracking.enabled and mode in ("line", "page")

        async with PrintheadBLE(self.ble) as ble:
            await ble.start_notifications(on_start, on_startpoint)

            if use_tracker:
                tracker = make_tracker(self.tracking, self.simulate)
                tracker.open()

            if self.ble.auto_start:
                press_event.set()
            print("Ready. Press the START button on the device to print."
                  if not self.ble.auto_start else "Auto-start engaged.")

            try:
                while True:
                    await press_event.wait()
                    press_event.clear()
                    state["busy"] = True
                    try:
                        if mode == "line":
                            await self._print_line_pass(ble, tracker, startpoint_event)
                        elif mode == "page":
                            await self._print_freehand_pass(ble, tracker)
                        else:
                            await ble.stream_time(self.frames, self.ble.period,
                                                  self.ble.verbose)
                    except Exception as exc:
                        print(f"ERROR during pass: {exc}")
                    finally:
                        state["busy"] = False
                        startpoint_event.clear()

                    if self.ble.once:
                        break
                    print("Waiting for next START press ...")
            finally:
                if tracker is not None:
                    tracker.close()

    # ------------------------------------------------- position-based pass
    async def _print_line_pass(self, ble, tracker, startpoint_event) -> None:
        """Fire the column that matches the measured head position.

        A startpoint-button press during the pass re-zeros the origin at the
        current position and resets the frontier, restarting the print from
        column 0."""
        t = self.tracking
        mapper = AdvanceMapper(t)
        pos_filter = PositionFilter(t.smooth_ms / 1000.0)
        loop = asyncio.get_event_loop()
        interval = 1.0 / t.poll_hz

        # 1) establish the origin (button = current pos; startpoint = wait first)
        if t.origin == "startpoint":
            print("Waiting for startpoint signal to zero position ...")
            await startpoint_event.wait()
        origin = pos_filter.update(await self._wait_for_position(tracker, loop),
                                   loop.time())
        mapper.set_origin(origin)
        # Ignore any startpoint press that belonged to the setup; only presses
        # during the loop below act as a live reset.
        startpoint_event.clear()
        print(f"Origin set. Printing {self.width} columns @ "
              f"{t.mm_per_column:.3f} mm/col "
              f"(~{self.width * t.mm_per_column:.1f} mm wide).")

        # Optional real-time timing profiler (see printhead/profiling.py).
        profiler = None
        if self.profile:
            from .profiling import PassProfiler
            profiler = PassProfiler(t.mm_per_column, csv_path=self.profile_csv)
            profiler.start()

        # Optional send recorder: reconstruct what is actually deposited on paper.
        recorder = None
        if self.record:
            from .recording import SendRecorder
            recorder = SendRecorder(t.mm_per_column)

        # 2) drive columns from position.
        # ``frontier`` is the highest column index already printed. Columns are
        # only ever printed while advancing past the frontier, so moving the head
        # back over already-printed columns never reprints them.
        frontier = -1
        firing = False
        ref_pos = np.asarray(origin, dtype=float)
        ref_t = loop.time()
        t_start = ref_t
        prev_adv = None
        prev_t = None

        while True:
            now = loop.time()

            # Startpoint button: re-zero the origin at the current position and
            # reset the stored progress so printing restarts from column 0.
            if startpoint_event.is_set():
                startpoint_event.clear()
                pos_filter.reset()
                origin = pos_filter.update(
                    await self._wait_for_position(tracker, loop), loop.time())
                mapper.set_origin(origin)      # re-zero (also clears auto-calib dir.)
                frontier = -1
                if firing:
                    await ble.write_blank()
                firing = False
                ref_pos, ref_t = np.asarray(origin, dtype=float), loop.time()
                t_start = ref_t                # give the restarted pass a fresh timeout
                print("[startpoint] origin reset to current position; "
                      "printing from column 0.")
                await asyncio.sleep(interval)
                continue

            pos = tracker.read_position()
            if pos is not None:
                pos = pos_filter.update(pos, now)   # low-pass the noisy signal
                if np.linalg.norm(pos - ref_pos) >= t.min_move_mm:
                    ref_pos, ref_t = pos, now        # accumulated real movement
                moving = (now - ref_t) <= _STALL_GRACE_S

                adv = mapper.advance(pos)            # None while auto-calibrating
                if adv is not None:
                    # Along-travel speed (mm/s) for the profiler.
                    speed = None
                    if prev_adv is not None and now > prev_t:
                        speed = abs(adv - prev_adv) / (now - prev_t)
                    prev_adv, prev_t = adv, now

                    col = int(round(adv / t.mm_per_column))
                    if col >= self.width:
                        break                        # reached the end of the text
                    col = max(0, col)

                    if not moving:
                        # head stopped -> stop firing (avoid an ink blob)
                        if firing:
                            await ble.write_blank()
                            if recorder is not None:
                                recorder.record(adv, BLANK_FRAME)
                            firing = False
                    elif col > frontier:
                        # advancing into new territory: print each new column
                        # once, filling any columns skipped by a fast feed.
                        start = col if frontier < 0 else frontier + 1
                        cols = list(range(start, col + 1))
                        tw = loop.time()
                        # One write per MTU-worth of columns: the firmware queues
                        # them and prints them in order, so this is equivalent to
                        # writing them one at a time but no longer depends on the
                        # connection interval carrying that many packets.
                        await ble.write_columns([self.frames[c] for c in cols])
                        per_col = (loop.time() - tw) / len(cols)
                        for c in cols:
                            if profiler is not None:
                                profiler.record_write(c, adv, per_col, speed)
                            if recorder is not None:
                                recorder.record(adv, self.frames[c])
                        frontier = col
                        firing = True
                    elif col < frontier:
                        # moving back over already-printed columns: do NOT
                        # reprint -> blank so no ink is deposited on the return.
                        if firing:
                            await ble.write_blank()
                            if recorder is not None:
                                recorder.record(adv, BLANK_FRAME)
                            firing = False
                    # col == frontier while moving: keep the leading column firing

            if now - t_start > t.timeout_s:
                print("Position pass timed out.")
                break
            await asyncio.sleep(interval)

        if profiler is not None:
            profiler.finish()
        if recorder is not None:
            if recorder.render(self.record, self._ink):
                print(f"Reconstruction of what was sent -> {self.record}")
            else:
                print("Nothing was recorded (no columns sent).")
        await ble.write_blank()
        print("Finished pass; sent blank frame.")

    async def _wait_for_position(self, tracker, loop, timeout=5.0):
        """Block until the tracker yields a first position sample."""
        t0 = loop.time()
        while True:
            pos = tracker.read_position()
            if pos is not None:
                return pos
            if loop.time() - t0 > timeout:
                raise RuntimeError("No position from tracker (is it in range?).")
            await asyncio.sleep(0.005)

    # --------------------------------------------------- freehand page pass
    async def _print_freehand_pass(self, ble, tracker) -> None:
        """
        Freehand 2D pass: project live position through a fixed
        ``PageCalibration`` (no per-pass origin -- the calibration already
        anchors ``(u, v)`` to the traced page corner, unlike line mode's
        button-zeroed origin), dose per-nozzle via ``CoverageEngine``, and
        stream the live pattern through a ``PatternSender`` ("latest wins",
        see ``pattern_sender.py``) instead of a queue of distinct columns.
        Runs until the whole target image is covered or the pass times out.

        Unlike ``_print_line_pass``, there is no separate stall-grace/anti-
        blob logic here: ``CoverageEngine`` already stops firing a pixel once
        it has been held for ``dose_hold_s``, whether the head is moving or
        stalled -- that cutoff *is* the anti-blob protection, per pixel
        rather than per pass.

        A startpoint-button press is not handled here yet (unlike line mode,
        where it re-zeros the origin) -- there is no obvious equivalent
        gesture for a fixed page calibration, so this is left for later
        rather than guessing.
        """
        if self.page_calibration is None:
            raise RuntimeError("Freehand pass requires a page calibration "
                               "(PrintController(page_calibration=...)).")
        t = self.tracking
        mapper = PageMapper(self.page_calibration)
        coverage = CoverageEngine(self._ink, t.mm_per_column, dose_hold_s=self.dose_hold_s)
        pos_filter = PositionFilter(t.smooth_ms / 1000.0)
        sender = PatternSender(ble)
        loop = asyncio.get_event_loop()
        interval = 1.0 / t.poll_hz

        print(f"Printing freehand: {self.width} columns x {self.height} rows, "
              f"dose_hold={self.dose_hold_s * 1000:.0f} ms. Move the cart over "
              f"the calibrated page.")

        t_start = loop.time()
        try:
            while True:
                now = loop.time()
                pos = tracker.read_position()
                if pos is not None:
                    pos = pos_filter.update(pos, now)   # low-pass the noisy signal
                    u_mm, v_mm, _z_mm = mapper.project(pos)
                    pattern, changed = coverage.step(u_mm, v_mm, now)
                    if changed:
                        sender.send(pattern)

                if coverage.done:
                    print("Page fully covered.")
                    break
                if now - t_start > t.timeout_s:
                    print("Freehand pass timed out.")
                    break
                await asyncio.sleep(interval)
        finally:
            await sender.close()
        await ble.write_blank()
        covered = int(coverage.printed.sum())
        total = int(coverage.ink.sum())
        print(f"Finished pass; sent blank frame. Covered {covered}/{total} ink pixels.")

    # ---------------------------------------------- dry-run simulation path
    async def _dry_run_line_pass(self) -> None:
        """Run the position loop against a null printhead and report coverage."""
        tracker = make_tracker(self.tracking, simulate=True)
        tracker.open()
        null = _NullPrinthead()
        try:
            await self._print_line_pass(null, tracker, _ImmediateEvent())
        finally:
            tracker.close()
        print(f"[sim] position loop issued {null.column_writes} column writes "
              f"for {self.width} columns.")

    async def _dry_run_freehand_pass(self) -> None:
        """
        Run the freehand loop against a null printhead. The default
        SimulatedTracker only moves along one fixed axis (no synthetic 2D
        scribble), so this mainly smoke-tests the wiring -- coverage.done is
        unlikely to go true for a real target image, so this typically runs
        for the full --timeout; pass a short one for a quick check.
        """
        tracker = make_tracker(self.tracking, simulate=True)
        tracker.open()
        null = _NullPrinthead()
        try:
            await self._print_freehand_pass(null, tracker)
        finally:
            tracker.close()
        print(f"[sim] freehand loop issued {null.pattern_writes} pattern writes "
              f"for a {self.width}x{self.height} target image.")
