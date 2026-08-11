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
import json
import math
from typing import List, Optional, Tuple

import numpy as np

from .ble_client import PrintheadBLE
from .calibration import PageCalibration
from .config import BleSettings, NozzleMapSettings, RenderSettings, TrackingSettings
from .coverage import DEFAULT_DOSE_HOLD_S, CoverageEngine, bar_offset_uv
from .geometry import (
    BLANK_FRAME,
    NOZZLE_BAR_WIDTH_MM,
    NOZZLE_MODE_LINE,
    NOZZLE_MODE_PAGE,
    NOZZLE_PITCH_MM,
    NUM_NOZZLES,
    SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM,
    SENSOR_TO_NOZZLE_COL_MM,
)
from .nozzle_map import remap_rows
from .pattern_sender import PatternSender
from .profiling import DEFAULT_BLE_WRITE_CEILING_PER_S
from .rendering import frames_from_ink, render_text, save_preview
from .tracking import AdvanceMapper, PageMapper, PositionFilter, make_tracker

# How long the head may sit still (having accumulated < min_move_mm) before we
# stop firing its column. Tolerates slow feed while preventing a stationary blob.
_STALL_GRACE_S = 0.2

# Cart speed (mm/s) above which the freehand pass warns the firmware "too
# fast" over BLE (see PrintController.speed_warning_mm_s and
# _print_freehand_pass's hysteresis below). Derived from the dose-tuning
# measurement documented in coverage.DEFAULT_DOSE_HOLD_S's comment: a
# simulated pass at the production dose_hold_s/poll_hz defaults gives 100%
# coverage at the measured median hand speed (17.3 mm/s), falling to 60% by
# 25 mm/s and 14% by 35 mm/s -- 25 mm/s is picked here as the point past
# which a meaningful fraction of a pass is already going unprinted, worth
# surfacing to the operator (and the firmware's LED) rather than a hard
# cutoff derived from any firmware constant.
DEFAULT_SPEED_WARNING_MM_S = 25.0


def _speed_warning_transition(is_warning: bool, speed_mm_s: float,
                               threshold_mm_s: float) -> bool:
    """
    Hysteresis for the speed-warning flag: turn ON once speed exceeds
    ``threshold_mm_s``, turn OFF only once it drops 20% below that (a dead
    band of ``threshold_mm_s * 0.8 .. threshold_mm_s``), so a speed hovering
    right at the threshold does not flip the BLE characteristic (and the
    firmware's LED) on every sample.

    Pure function of the current state and one new sample -- kept free of
    asyncio/BLE so it is directly, deterministically unit-testable (see
    tests/test_freehand_pass.py, including its mutation check: removing the
    dead band, i.e. using ``threshold_mm_s`` for both edges, reintroduces
    chattering on a hovering speed sequence).
    """
    if not is_warning and speed_mm_s > threshold_mm_s:
        return True
    if is_warning and speed_mm_s < threshold_mm_s * 0.8:
        return False
    return is_warning


class _NullPrinthead:
    """Stand-in for PrintheadBLE used by ``--dry-run --simulate`` (no BLE)."""

    def __init__(self):
        self.column_writes = 0
        self.blank_writes = 0
        self.pattern_writes = 0
        self.print_mode = None
        self.speed_warnings = []   # every set_speed_warning() call, in order

    async def write_column(self, frame):
        self.column_writes += 1

    async def write_columns(self, frames):
        self.column_writes += len(list(frames))

    async def write_blank(self):
        self.blank_writes += 1

    async def write_pattern(self, pattern):
        self.pattern_writes += 1

    async def set_print_mode(self, mode, required: bool = True):
        self.print_mode = mode
        return True

    async def set_speed_warning(self, warn: bool):
        self.speed_warnings.append(warn)


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
                 dose_hold_s: float = DEFAULT_DOSE_HOLD_S,
                 spray_radius_mm: float = 0.0,
                 spray_strength: float = 0.0,
                 nozzle_group: int = 1,
                 ble_write_ceiling: float = DEFAULT_BLE_WRITE_CEILING_PER_S,
                 speed_warning_mm_s: float = DEFAULT_SPEED_WARNING_MM_S,
                 progress_json: bool = False,
                 sensor_offset_row_mm: float = SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM,
                 sensor_offset_col_mm: float = SENSOR_TO_NOZZLE_COL_MM,
                 boresight_deg: float = 0.0):
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
        self.spray_radius_mm = spray_radius_mm
        self.spray_strength = spray_strength
        # Page mode only (see CoverageEngine.step()'s docstring for the
        # exact per-group rule) -- coarser vertical addressing, requested by
        # the hardware owner; not threaded through anywhere on the line/time
        # path, which never constructs a CoverageEngine at all.
        self.nozzle_group = nozzle_group
        self.ble_write_ceiling = ble_write_ceiling
        self.speed_warning_mm_s = speed_warning_mm_s
        self.progress_json = progress_json
        self.sensor_offset_row_mm = sensor_offset_row_mm
        self.sensor_offset_col_mm = sensor_offset_col_mm
        # Additive fine-tune (degrees) on top of the yaw computed from the
        # captured boresight -- see PageMapper.boresight_offset_rad / CLI's
        # --boresight-deg. 0.0 = trust the captured boresight exactly.
        self.boresight_deg = boresight_deg

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

            # The firmware defaults to (and silently stays in) line mode, so
            # the mode matching this pass must be selected explicitly here --
            # otherwise every write below is dosed with the wrong model
            # (see ble_client.set_print_mode). Page mode is a hard
            # requirement: a silently-wrong dose model is worse than not
            # printing. Line/time mode is best-effort: it is also the
            # firmware's own default, so older firmware without this
            # characteristic still behaves correctly without it.
            if mode == "page":
                await ble.set_print_mode(NOZZLE_MODE_PAGE, required=True)
            else:
                await ble.set_print_mode(NOZZLE_MODE_LINE, required=False)

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

        try:
            while True:
                now = loop.time()

                # Startpoint button: re-zero the origin at the current position and
                # reset the stored progress so printing restarts from column 0.
                if startpoint_event.is_set():
                    startpoint_event.clear()
                    pos_filter.reset()
                    origin = pos_filter.update(
                        await self._wait_for_position(tracker, loop), loop.time())
                    mapper.set_origin(origin)  # re-zero (also clears auto-calib dir.)
                    frontier = -1
                    if firing:
                        await ble.write_blank()
                    firing = False
                    ref_pos, ref_t = np.asarray(origin, dtype=float), loop.time()
                    t_start = ref_t             # give the restarted pass a fresh timeout
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
        finally:
            # Same cleanup-ordering bug as the freehand pass (defect 3): an
            # exception out of the loop above (KeyboardInterrupt included)
            # must not skip closing the profiler CSV or rendering --record.
            # write_blank() alone is allowed to fail here (link may already
            # be down) without masking whatever is already propagating.
            if profiler is not None:
                profiler.finish()
            if recorder is not None:
                if recorder.render(self.record, self._ink):
                    print(f"Reconstruction of what was sent -> {self.record}")
                else:
                    print("Nothing was recorded (no columns sent).")
            try:
                await ble.write_blank()
            except Exception as exc:
                print(f"[warn] could not send final blank frame: {exc}")
            else:
                print("Finished pass; sent blank frame.")

    async def _wait_for_pose(self, tracker, loop, timeout=5.0):
        """Block until the tracker yields a first position, returning
        ``(pos, quat)`` from that same read.

        The simple page frame needs both from the SAME sample: the position
        anchors the origin and the orientation becomes the yaw reference, and
        pairing a position with a quaternion read at some other instant would
        mis-reference the yaw. ``quat`` may still be ``None`` -- not every
        tracker/packet carries orientation -- which callers must handle
        rather than assume."""
        t0 = loop.time()
        while True:
            pos, quat = tracker.read_pose()
            if pos is not None:
                return pos, quat
            if loop.time() - t0 > timeout:
                raise RuntimeError("No position from tracker (is it in range?).")
            await asyncio.sleep(0.005)

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

        ``self.progress_json``, if set, switches stdout from the plain-text
        status lines below to NDJSON progress events -- one ``coverage_start``
        up front, one ``coverage`` per sample (current ``u``/``v``/``row``/
        ``col`` plus any cells that just finished dosing), and it suppresses
        the plain-text lines this would otherwise interleave with (mirrors
        ``diagnostics.monitor_position``'s ``ndjson`` switch). This is what
        the web UI's live coverage canvas consumes (see ``ui/server.py``).

        Also warns the firmware over BLE (``ble.set_speed_warning``) when the
        along-travel speed already being computed here for the profiler
        exceeds ``self.speed_warning_mm_s``, with hysteresis
        (``_speed_warning_transition``) so a speed hovering at the threshold
        does not chatter the characteristic every sample. Cleared
        unconditionally at pass end, in the same tolerant ``finally`` path
        as ``sender.close()``/the final blank frame, so a stale warning never
        outlives the pass that raised it.
        """
        if self.page_calibration is None:
            raise RuntimeError("Freehand pass requires a page calibration "
                               "(PrintController(page_calibration=...)).")
        t = self.tracking
        pj = self.progress_json
        mapper = PageMapper(self.page_calibration,
                           sensor_offset_row_mm=self.sensor_offset_row_mm,
                           sensor_offset_col_mm=self.sensor_offset_col_mm,
                           boresight_offset_rad=math.radians(self.boresight_deg))
        coverage = CoverageEngine(self._ink, t.mm_per_column,
                                  dose_hold_s=self.dose_hold_s,
                                  spray_radius_mm=self.spray_radius_mm,
                                  spray_strength=self.spray_strength,
                                  nozzle_group=self.nozzle_group)
        pos_filter = PositionFilter(t.smooth_ms / 1000.0)
        sender = PatternSender(ble)
        loop = asyncio.get_event_loop()
        interval = 1.0 / t.poll_hz

        # Optional real-time timing profiler (see printhead/profiling.py).
        profiler = None
        if self.profile:
            from .profiling import PassProfiler
            profiler = PassProfiler(t.mm_per_column, csv_path=self.profile_csv,
                                    mode="page", ble_write_ceiling=self.ble_write_ceiling)
            profiler.start()

        if pj:
            print(json.dumps({"event": "coverage_start", "width": self.width,
                              "height": self.height}), flush=True)
        else:
            print(f"Printing freehand: {self.width} columns x {self.height} rows, "
                  f"dose_hold={self.dose_hold_s * 1000:.0f} ms. Move the cart over "
                  f"the calibrated page.")

        # Quantization-cliff guard (see coverage.DEFAULT_DOSE_HOLD_S for the
        # measured example): CoverageEngine.step() only marks a pixel
        # printed on a *sample* that finds elapsed dwell >= dose_hold_s,
        # measured from the first sample on that pixel -- so completion
        # costs whole poll intervals, not continuous time. Once
        # dose_hold_s >= 1/poll_hz, a second sample one interval later is
        # never enough (it lands at exactly one interval, still short of a
        # hold that is itself >= one interval); a THIRD sample landing on
        # the same column is required, which is a much narrower window at
        # realistic hand speeds and collapses coverage rather than merely
        # reducing it. Warn here instead of letting that surface later as a
        # near-empty coverage report with no obvious cause. Suppressed in
        # --progress-json mode, which must stay pure NDJSON for the UI
        # consumer (mirrors the out-of-page warning's `not pj` gating below).
        if not pj and self.dose_hold_s >= 1.0 / t.poll_hz:
            poll_interval_ms = 1000.0 / t.poll_hz
            print(f"[warn] dose_hold_s={self.dose_hold_s * 1000:.2f} ms >= poll "
                  f"interval={poll_interval_ms:.2f} ms (--poll-hz {t.poll_hz:g}): "
                  f"a dose then needs three or more samples to land on the same "
                  f"column, and coverage will be very low. Use a shorter "
                  f"--dose-hold-s or a higher --poll-hz.")

        # No boresight_quat on this calibration (every calibration saved
        # before this feature existed): PageMapper.project() therefore never
        # rotates the sensor->nozzle offset and CoverageEngine.step() is
        # always called with yaw_rad=0.0 below -- i.e. exactly today's
        # behaviour, silently. Loud rather than silent on purpose (see
        # tracking.PageMapper's docstring / README): re-calibrating and
        # capturing a boresight is the fix, not something this pass can
        # guess its way around (e.g. from the cart's pose at pass start).
        #
        # page_frame == "simple" is deliberately EXCLUDED here: its own
        # boresight status gets a more specific, more accurate message just
        # below ("auto-captured at START" / "using pinned yaw reference" /
        # the no-orientation fallback) -- this generic one's instructions
        # ("re-run page calibration... trace the row edge") describe the
        # CALIBRATED frame's workflow and would be actively wrong advice for
        # simple mode, which has no calibration file to re-run at all.
        if (not pj and t.page_frame != "simple"
                and self.page_calibration.boresight_quat is None):
            print("[warn] page calibration has no boresight (captured "
                  "before cart-rotation correction existed) -- printing "
                  "with NO rotation correction, same as before. Re-run page "
                  "calibration and capture a boresight (hold the cart flat "
                  "with the nozzle bar aligned along the traced row edge) "
                  "to enable it.")

        # Simple (calibration-free) frame: its origin is the tracker's world
        # zero -- somewhere on the table, not on the paper -- so zero it here
        # at the cart's current position. "Where you press START" becomes the
        # page's (0, 0), exactly like line mode's --origin button. A traced
        # calibration is deliberately NOT re-zeroed: its origin is a measured
        # page corner meant to outlive a single pass (see PageMapper.
        # set_origin).
        if t.page_frame == "simple":
            raw_pos, start_quat = await self._wait_for_pose(tracker, loop)
            start_pos = pos_filter.update(raw_pos, loop.time())
            # Only auto-capture a reference pose if none was already pinned
            # via --simple-boresight (self.page_calibration built by
            # cli.build_page_calibration). Capturing unconditionally here
            # would silently overwrite an operator-verified reference with
            # whatever pose the cart happens to be in AT THIS PASS's start --
            # which is exactly the failure blind first-sample auto-capture
            # has in the field (BLE still settling, hand not yet still): a
            # wrong pose becomes "0 deg" with no way to notice. See
            # PageCalibration.simple_frame's docstring.
            had_pinned_boresight = mapper.calibration.boresight_quat is not None
            if not had_pinned_boresight:
                mapper.capture_boresight(start_quat)
            # zero_at_nozzle, not set_origin: the origin has to land under the
            # NOZZLE BAR, not the sensor ~62mm away, or every sample reads
            # out of bounds and nothing prints -- see zero_at_nozzle.
            mapper.zero_at_nozzle(start_pos, start_quat)
            if not pj:
                print(f"[simple] page origin zeroed at the nozzle bar's current "
                      f"position (sensor at {start_pos[0]:.1f}, "
                      f"{start_pos[1]:.1f}, {start_pos[2]:.1f} mm); page axes "
                      f"= tracker x/y")
                if had_pinned_boresight:
                    q = mapper.calibration.boresight_quat
                    print(f"[simple] using pinned yaw reference from "
                          f"--simple-boresight (qx={q[0]:+.4f} qy={q[1]:+.4f} "
                          f"qz={q[2]:+.4f} qw={q[3]:+.4f}) -- NOT re-captured "
                          f"at this START.")
                elif start_quat is None:
                    print("[warn] no orientation from the tracker at START: "
                          "yaw reference not captured, printing with NO "
                          "rotation correction (cart must stay at the START "
                          "angle for the whole pass).")
                else:
                    print(f"[simple] yaw reference auto-captured at START "
                          f"(qx={start_quat[0]:+.4f} qy={start_quat[1]:+.4f} "
                          f"qz={start_quat[2]:+.4f} qw={start_quat[3]:+.4f}); "
                          f"the pose held now counts as 0 deg. If this wasn't "
                          f"truly flat, capture-and-verify first with --pos, "
                          f"then pin it with --simple-boresight instead of "
                          f"relying on this auto-capture.")

        t_start = loop.time()
        prev_u, prev_v, prev_t = None, None, None
        prev_printed = coverage.printed.copy() if pj else None
        done_reason = None
        speed_warn_state = False   # current value of the speed-warning flag

        # Out-of-page visibility (defect 2): a pass whose (u, v) never lands
        # inside the target image is otherwise indistinguishable from a
        # normal pass at the API level -- `coverage.step()` just returns an
        # all-zero pattern forever, `changed` goes False after the first
        # sample, nothing gets sent, no profiler sample is recorded, and the
        # pass exits 0 with "Covered 0/N". Track the observed extents and
        # whether anything was ever in bounds so the end (and, in plain-text
        # mode, a live warning) can tell the user *why* nothing happened
        # instead of leaving them to guess.
        u_min = u_max = v_min = v_max = None
        in_bounds_samples = 0
        samples = 0
        last_warn_t = None

        # --record path overlay (see recording.render_coverage): one
        # (row, col) point per sample for the raw sensor centre and the
        # nozzle-bar centre, so the reconstruction can trace where the cart
        # physically went, not just what got covered. Only collected when
        # actually recording -- a multi-minute pass at up to --poll-hz
        # samples/s has no reason to grow these lists otherwise.
        # sample_times is the elapsed pass time (seconds since t_start) at
        # each of those same points -- same index space as both path lists,
        # since all three are appended together once per sample below -- so
        # render_coverage can place a timestamped marker at, say, the sample
        # nearest 2.0s/4.0s/6.0s... on both paths at once.
        sensor_path: Optional[List[Tuple[int, int]]] = [] if self.record else None
        nozzle_path: Optional[List[Tuple[int, int]]] = [] if self.record else None
        sample_times: Optional[List[float]] = [] if self.record else None

        try:
            while True:
                now = loop.time()
                pos, quat = tracker.read_pose()
                if pos is not None:
                    pos = pos_filter.update(pos, now)   # low-pass the noisy signal
                    # project() computes (and caches on mapper.last_yaw_rad)
                    # this sample's yaw as a side effect -- read it back
                    # below rather than calling rotation.yaw_about_normal a
                    # second time, so it's computed exactly once per sample
                    # and shared between the position projection and
                    # CoverageEngine.step()'s per-nozzle placement.
                    u_mm, v_mm, _z_mm = mapper.project(pos, quat)
                    yaw_rad = mapper.last_yaw_rad

                    if self.record:
                        # Sensor centre: the RAW page-plane position, before
                        # PageMapper.project() adds the sensor->nozzle
                        # offset -- i.e. calibration.project() alone, not
                        # mapper.project()'s (u_mm, v_mm) above (that is
                        # already nozzle-0-referenced, see PageMapper's
                        # class docstring).
                        su, sv, _ = mapper.calibration.project(pos)
                        sensor_path.append(
                            (int(round(sv / NOZZLE_PITCH_MM)),
                             int(round(su / t.mm_per_column))))
                        # Nozzle-bar CENTRE: nozzle 0's (u_mm, v_mm) shifted
                        # by half the bar width along the CURRENT (yaw-
                        # rotated) bar direction -- bar_offset_uv is the same
                        # formula CoverageEngine.step() places every
                        # individual nozzle with (see its docstring), just
                        # evaluated once per sample for the bar's midpoint
                        # rather than per nozzle.
                        ndu, ndv = bar_offset_uv(NOZZLE_BAR_WIDTH_MM / 2.0, yaw_rad)
                        nozzle_path.append(
                            (int(round((v_mm + ndv) / NOZZLE_PITCH_MM)),
                             int(round((u_mm + ndu) / t.mm_per_column))))
                        sample_times.append(now - t_start)

                    samples += 1
                    u_min = u_mm if u_min is None else min(u_min, u_mm)
                    u_max = u_mm if u_max is None else max(u_max, u_mm)
                    v_min = v_mm if v_min is None else min(v_min, v_mm)
                    v_max = v_mm if v_max is None else max(v_max, v_mm)

                    speed = None
                    if prev_u is not None and now > prev_t:
                        speed = ((u_mm - prev_u) ** 2 + (v_mm - prev_v) ** 2) ** 0.5 \
                            / (now - prev_t)
                    prev_u, prev_v, prev_t = u_mm, v_mm, now

                    # Speed warning, with hysteresis (see
                    # _speed_warning_transition): only ever writes BLE on an
                    # actual state change, which in practice is rare enough
                    # that the extra await here is a non-issue -- no need
                    # for a PatternSender-style background task for
                    # something this infrequent.
                    if speed is not None:
                        new_warn = _speed_warning_transition(
                            speed_warn_state, speed, self.speed_warning_mm_s)
                        if new_warn != speed_warn_state:
                            speed_warn_state = new_warn
                            await ble.set_speed_warning(speed_warn_state)

                    pattern, changed = coverage.step(u_mm, v_mm, now, yaw_rad=yaw_rad)
                    if coverage.last_in_bounds:
                        in_bounds_samples += 1
                    if changed:
                        sender.send(pattern)
                        if profiler is not None:
                            profiler.record_page_sample(u_mm, v_mm, speed, quat=quat)

                    # Nothing has ever been in bounds yet: say so periodically
                    # instead of running the whole pass in silence (plain-text
                    # mode only -- progress-json carries this in coverage_done
                    # for the UI instead, see below).
                    if not pj and in_bounds_samples == 0:
                        if last_warn_t is None or now - last_warn_t >= 2.0:
                            last_warn_t = now
                            req_u = self.width * t.mm_per_column
                            req_v = (NUM_NOZZLES - 1) * NOZZLE_PITCH_MM
                            print(f"[warn] cart not over the target page yet: "
                                  f"u={u_mm:7.2f} v={v_mm:7.2f} mm (need u in "
                                  f"[0, {req_u:.1f}] mm, v within "
                                  f"+/-{req_v:.1f} mm of the calibration origin)")

                    new_cells = []
                    if pj:
                        # NOT gated on `changed`: a nozzle's dose completing
                        # updates printed[row, col] on this tick, but its bit
                        # in `pattern` only flips off on the *next* tick (once
                        # wanted becomes False for that pixel) -- so `changed`
                        # lags a fresh completion by one sample and would miss
                        # it here if the pass ends (coverage.done) before that
                        # next tick ever happens.
                        new_mask = coverage.printed & ~prev_printed
                        if new_mask.any():
                            rows, cols = np.nonzero(new_mask)
                            new_cells = list(zip(rows.tolist(), cols.tolist()))
                            prev_printed = coverage.printed.copy()

                    if pj:
                        col = int(round(u_mm / t.mm_per_column)) if t.mm_per_column else 0
                        row = int(round(v_mm / NOZZLE_PITCH_MM))
                        print(json.dumps({"event": "coverage", "u": round(u_mm, 3),
                                          "v": round(v_mm, 3), "row": row, "col": col,
                                          "new_cells": new_cells}), flush=True)

                if coverage.done:
                    done_reason = "complete"
                    if not pj:
                        print("Page fully covered.")
                    break
                if now - t_start > t.timeout_s:
                    done_reason = "timeout"
                    if not pj:
                        print("Freehand pass timed out.")
                    break
                await asyncio.sleep(interval)
        finally:
            # Everything below must still run on a KeyboardInterrupt or any
            # other exception raised out of the loop above (defect 3): the
            # profiler CSV is otherwise never closed/flushed (0-byte file),
            # --record is never rendered, and the head is left firing. Only
            # write_blank() is allowed to fail here without derailing this --
            # the link may already be down, and a secondary exception from
            # cleanup must not mask whatever is already propagating out of
            # the loop.
            await sender.close()
            # A stale "too fast" warning must not linger once the pass has
            # ended. set_speed_warning() already fails soft on its own (see
            # PrintheadBLE.set_speed_warning), but this is wrapped the same
            # tolerant way as write_blank() below anyway: cleanup running
            # during an already-propagating exception (KeyboardInterrupt
            # included, defect 3) must never raise a second one that masks
            # the first.
            try:
                await ble.set_speed_warning(False)
            except Exception:
                pass
            try:
                await ble.write_blank()
            except Exception as exc:
                print(f"[warn] could not send final blank frame: {exc}")

            if profiler is not None:
                profiler.finish()
            if self.record:
                from .recording import render_coverage
                if render_coverage(coverage.printed, coverage.ink, self.record,
                                   sensor_path=sensor_path, nozzle_path=nozzle_path,
                                   sample_times=sample_times):
                    if not pj:
                        print(f"Coverage reconstruction -> {self.record}")
                elif not pj:
                    print("Nothing was recorded (nothing printed).")

            covered = int(coverage.printed.sum())
            total = int(coverage.ink.sum())
            if pj:
                print(json.dumps({"event": "coverage_done", "reason": done_reason,
                                  "covered": covered, "total": total,
                                  "in_bounds_samples": in_bounds_samples,
                                  "samples": samples,
                                  "u_min": round(u_min, 3) if u_min is not None else None,
                                  "u_max": round(u_max, 3) if u_max is not None else None,
                                  "v_min": round(v_min, 3) if v_min is not None else None,
                                  "v_max": round(v_max, 3) if v_max is not None else None}),
                      flush=True)
            elif in_bounds_samples == 0 and samples > 0:
                req_u = self.width * t.mm_per_column
                req_v = (NUM_NOZZLES - 1) * NOZZLE_PITCH_MM
                print("Finished pass; sent blank frame.")
                print(f"Covered 0/{total} ink pixels -- the cart never overlapped "
                      f"the target page during this pass.")
                print(f"  observed u: {u_min:7.2f} .. {u_max:7.2f} mm "
                      f"(page needs u in [0.0, {req_u:.1f}] mm)")
                print(f"  observed v: {v_min:7.2f} .. {v_max:7.2f} mm "
                      f"(page needs v within +/-{req_v:.1f} mm of the "
                      f"calibration origin)")
                print("  Likely cause: (1) the cart was physically somewhere "
                      "else on the page, or (2) the calibration origin/axes "
                      "do not correspond to where you think the page corner "
                      "is.")
                print("  Check with: --pos --page-calibration PATH to see live "
                      "(u, v) against known hand motion before printing.")
            else:
                print(f"Finished pass; sent blank frame. Covered {covered}/{total} "
                      f"ink pixels.")

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
