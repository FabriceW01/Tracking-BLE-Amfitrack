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
    NOZZLE_BAR_SPAN_MM,
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

# Throttle for --verbose's live status line during a print pass (see
# _print_line_pass/_print_freehand_pass below): matches --pos's own default
# hz=15 in diagnostics.monitor_position, so the two read at the same rate
# even though a print pass itself samples at --poll-hz (default now 500) --
# printing every single poll sample would flood the terminal for no benefit,
# since nobody reads faster than this anyway.
_VERBOSE_STATUS_INTERVAL_S = 1.0 / 15.0


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


def _extrapolate_uv(u_mm: float, v_mm: float, vx_mm_s: float, vy_mm_s: float,
                    latency_s: float) -> "tuple[float, float]":
    """
    Linearly project ``(u_mm, v_mm)`` forward by ``latency_s`` seconds along
    the velocity ``(vx_mm_s, vy_mm_s)`` -- see
    ``PrintController._print_freehand_pass``'s docstring (``--latency-
    compensate-s``) for the full rationale. ``latency_s <= 0`` (including the
    default 0.0) is a no-op, returned unchanged rather than as
    ``u_mm + 0.0`` -- keeps the disabled path bit-identical to never having
    called this at all, and lets a caller pass a negative value through
    without silently clamping it to zero (an operator experimenting with the
    sign gets exactly what they asked for, not a surprise no-op).

    Pure function of its five inputs -- kept free of asyncio/BLE/CoverageEngine
    so it is directly, deterministically unit-testable (see
    tests/test_freehand_pass.py), same reasoning as
    ``_speed_warning_transition`` above.
    """
    if latency_s == 0.0:
        return u_mm, v_mm
    return u_mm + vx_mm_s * latency_s, v_mm + vy_mm_s * latency_s


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
                 boresight_deg: float = 0.0,
                 latency_compensate_s: float = 0.0):
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
        # Seconds to linearly extrapolate the position forward by before
        # handing it to CoverageEngine.step() -- see CLI's
        # --latency-compensate-s and _print_freehand_pass's docstring for the
        # full rationale/caveats. 0.0 (default) = today's behaviour, no
        # extrapolation.
        self.latency_compensate_s = latency_compensate_s
        # True once a STARTPOINT button press between passes has re-zeroed
        # page_calibration.origin (see _set_page_origin). Read by
        # _print_freehand_pass, which then does NOT re-zero the simple frame
        # at pass start -- an operator-placed origin must outrank the blind
        # "wherever the cart is at START" fallback, exactly as an explicitly
        # pinned --simple-boresight outranks blind boresight auto-capture.
        # Deliberately sticky across passes rather than cleared at pass end:
        # a placed origin, like a traced calibration, is meant to survive
        # until the operator moves it again.
        self._page_origin_pinned = False

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
                    # In page mode a STARTPOINT press while idle places the
                    # page origin instead of starting a pass; see
                    # _wait_for_start_press. Every other mode keeps the plain
                    # "block until START" behaviour it always had.
                    await self._wait_for_start_press(press_event, startpoint_event,
                                                     tracker, mode)
                    press_event.clear()
                    # Page mode only: drop a STARTPOINT press that arrived
                    # while idle and was already acted on (or raced in just
                    # before START). During a page pass the same button means
                    # STOP, so inheriting a stale press would abort the pass
                    # on its very first sample. Line mode deliberately keeps
                    # its latched press -- `--origin startpoint` waits on
                    # exactly this event at pass start, and clearing it here
                    # would force a second press it never used to need.
                    if mode == "page":
                        startpoint_event.clear()
                    state["busy"] = True
                    try:
                        if mode == "line":
                            await self._print_line_pass(ble, tracker, startpoint_event)
                        elif mode == "page":
                            await self._print_freehand_pass(ble, tracker,
                                                            startpoint_event)
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

    # --------------------------------------------- idle STARTPOINT handling
    async def _wait_for_start_press(self, press_event, startpoint_event,
                                    tracker, mode) -> None:
        """Block until the START button is pressed.

        In **page mode** a STARTPOINT press arriving while no pass is running
        does not start anything: it places the page origin at the cart's
        current position (see :meth:`_set_page_origin`) and goes back to
        waiting, so the operator can aim, place, re-place, and only then
        press START. That is the idle half of the button's page-mode role --
        the other half is STOP during a running pass (see
        ``_print_freehand_pass``).

        Every other mode falls straight through to the plain
        ``press_event.wait()`` this replaced: line mode already gives the
        same button a different, established meaning (``--origin startpoint``
        waits on it at pass start, and a mid-pass press re-zeros the origin),
        and consuming presses here would break both.
        """
        if mode != "page" or tracker is None:
            await press_event.wait()
            return
        while True:
            # Flags first, so a press latched before this call (or acted on
            # by the previous iteration) is seen without waiting for a
            # further edge. START wins a simultaneous pair: the pending
            # STARTPOINT is dropped by the caller rather than acted on here,
            # since the operator's intent in that case is to print.
            if press_event.is_set():
                return
            if startpoint_event.is_set():
                startpoint_event.clear()
                await self._set_page_origin(tracker)
                continue
            waiters = [asyncio.ensure_future(press_event.wait()),
                       asyncio.ensure_future(startpoint_event.wait())]
            try:
                await asyncio.wait(waiters, return_when=asyncio.FIRST_COMPLETED)
            finally:
                # Both are awaited after cancelling: a bare cancel() leaves
                # the loser pending long enough to surface as a "Task was
                # destroyed but it is pending" warning on shutdown.
                for w in waiters:
                    w.cancel()
                await asyncio.gather(*waiters, return_exceptions=True)

    async def _set_page_origin(self, tracker) -> None:
        """Re-zero the page origin so the target image's CENTRE lands under
        the nozzle bar at the cart's current position (page mode, STARTPOINT
        button while idle).

        Deliberately the image's centre, not its top-left corner (which is
        what ``target_uv``'s default ``(0, 0)`` would place here): the
        operator points at the spot they want the PATTERN centred on, not at
        a corner they'd otherwise have to estimate by eye and by the
        pattern's own width/height, which nothing about the physical paper
        tells them. Corner placement is still available -- it's just
        ``zero_at_nozzle``'s own default -- but centring is what this button
        is for.

        Only ``PageCalibration.origin`` moves. The traced axes/scales from a
        ``--page-calibration`` file -- the plane definition itself -- are
        deliberately untouched: this answers "where on the sheet does the
        image start", not "where is the sheet", which is exactly the split
        the calibration file already encodes.

        Uses ``zero_at_nozzle``, not ``set_origin``: the target point has to
        land under the NOZZLE BAR, not under the sensor ~62mm away, or every
        sample reads out of bounds and nothing prints (see that method).

        A tracker that yields no pose is reported and otherwise ignored --
        returning to the START wait beats letting a RuntimeError out of the
        idle loop and tearing down the whole BLE session over a button press.
        """
        if self.page_calibration is None:
            print("[startpoint] no page calibration loaded -- cannot place "
                  "an origin.")
            return
        loop = asyncio.get_event_loop()
        try:
            pos, quat = await self._wait_for_pose(tracker, loop)
        except RuntimeError as exc:
            print(f"[startpoint] origin NOT placed: {exc}")
            return

        mapper = PageMapper(self.page_calibration,
                            sensor_offset_row_mm=self.sensor_offset_row_mm,
                            sensor_offset_col_mm=self.sensor_offset_col_mm,
                            boresight_offset_rad=math.radians(self.boresight_deg))
        # Simple frame with no reference pose yet: adopt the one being held
        # right now, so the placement gesture fixes position AND heading
        # together -- the pass would otherwise capture its heading later,
        # from a different pose, and the placed origin would be referenced
        # to an angle the operator never confirmed. Guarded on `is None` so
        # an explicitly pinned --simple-boresight is never overwritten,
        # matching _print_freehand_pass's own guard.
        if (self.tracking.page_frame == "simple"
                and self.page_calibration.boresight_quat is None):
            mapper.capture_boresight(quat)
        # Middle INDEX of 0..width-1 / 0..height-1, in mm along each axis --
        # a continuous mm value, not rounded to a pixel, so it lands exactly
        # between the two centre pixels on an even width/height rather than
        # being pulled a half-pixel toward one side.
        center_u_mm = (self.width - 1) / 2.0 * self.tracking.mm_per_column
        center_v_mm = (self.height - 1) / 2.0 * NOZZLE_PITCH_MM
        mapper.zero_at_nozzle(pos, quat, target_uv=(center_u_mm, center_v_mm))
        self._page_origin_pinned = True
        print(f"[startpoint] page origin placed -- pattern CENTRE now at the "
              f"nozzle bar's current position (sensor at {pos[0]:.1f}, "
              f"{pos[1]:.1f}, {pos[2]:.1f} mm). Press START to print from here.")

    # ------------------------------------------------- position-based pass
    async def _print_line_pass(self, ble, tracker, startpoint_event) -> None:
        """Fire the column that matches the measured head position.

        A startpoint-button press during the pass re-zeros the origin at the
        current position and resets the frontier, restarting the print from
        column 0.

        ``--verbose`` (``self.ble.verbose``) prints a live, self-overwriting
        status line (position/advance/column/firing state) throttled to
        ``_VERBOSE_STATUS_INTERVAL_S`` -- the printing-time equivalent of
        ``--pos``, which cannot be combined with an actual print pass since
        it is one of the standalone diagnostics that connect, report, and
        exit (see cli.py's mutually-exclusive debug group)."""
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
        last_verbose_t = None

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

                        if (self.ble.verbose and (last_verbose_t is None
                                or now - last_verbose_t >= _VERBOSE_STATUS_INTERVAL_S)):
                            last_verbose_t = now
                            print(f"x={pos[0]:9.2f}  y={pos[1]:9.2f}  z={pos[2]:9.2f} mm  |  "
                                  f"advance={adv:9.2f} mm  |  col={col:5d}/{self.width}  |  "
                                  f"{'firing' if firing else 'idle '}", end="\r", flush=True)

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
            # The --verbose status line above ends every write with `\r`,
            # not `\n`, so it can overwrite itself in place; the messages
            # below use ordinary print() calls and would otherwise land on
            # top of that partial line instead of a fresh one (mirrors
            # diagnostics.monitor_position's identical "print() once before
            # real messages resume" cleanup).
            if self.ble.verbose:
                print()
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
    async def _print_freehand_pass(self, ble, tracker,
                                   startpoint_event=None) -> None:
        """
        Freehand 2D pass: project live position through a fixed
        ``PageCalibration`` (no per-pass origin -- the calibration already
        anchors ``(u, v)`` to the traced page corner, unlike line mode's
        button-zeroed origin), dose per-nozzle via ``CoverageEngine``, and
        stream the live pattern through a ``PatternSender`` ("latest wins",
        see ``pattern_sender.py``) instead of a queue of distinct columns.
        Runs until the whole target image is covered, the pass times out, or
        the operator stops it (below).

        Unlike ``_print_line_pass``, there is no separate stall-grace/anti-
        blob logic here: ``CoverageEngine`` already stops firing a pixel once
        it has been held for ``dose_hold_s``, whether the head is moving or
        stalled -- that cutoff *is* the anti-blob protection, per pixel
        rather than per pass.

        ``startpoint_event`` is the STARTPOINT button, and in page mode it
        means **STOP**: a press ends this pass immediately (blank frame,
        profiler/record still flushed through the usual ``finally``) and
        returns to "Waiting for next START press ...". Deliberately NOT line
        mode's meaning -- there, mid-pass, the same button re-zeros the origin
        and reprints from column 0, which needs a monotonically advancing
        frontier to be worth doing. A page pass has no frontier to rewind and
        an already-placed origin (see ``_set_page_origin``, the idle half of
        this button's page-mode role), so "abort what I'm doing" is the
        useful gesture that was missing. ``None`` disables it, for the
        dry-run/simulate path that has no button at all.

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

        ``--verbose`` (``self.ble.verbose``) prints a live, self-overwriting
        status line -- sensor x/y/z, page u/v, row/col, yaw/roll/pitch and
        the running covered/total count -- throttled to
        ``_VERBOSE_STATUS_INTERVAL_S``, so the position readout ``--pos``
        gives can be watched *while actually printing* instead of only as a
        separate standalone run beforehand: ``--pos`` is one of cli.py's
        mutually-exclusive debug checks (connect, report, exit) and cannot
        be combined with a real pass at all. Suppressed whenever
        ``self.progress_json`` is set -- that stream must stay pure NDJSON
        for the UI consumer, same reasoning as the plain-text warnings
        above.

        ``self.latency_compensate_s`` (``--latency-compensate-s``, default
        0.0 = off), if set, linearly extrapolates the ``(u, v)`` handed to
        ``CoverageEngine.step()`` forward by that many seconds along the
        current sample's own velocity estimate, to correct for the measured
        pipeline delay between reading a position and that position's ink
        actually landing (BLE connection interval + firmware queue + fire
        slot -- see the CLI flag's help text for the numbers this is tuned
        against). Deliberately narrow in scope: only the coverage-engine
        input is shifted. Every other use of position in this method --
        ``--record``'s path panels, the out-of-page bounds tracking, the
        profiler, the speed warning above -- keeps the real, uncompensated
        ``u_mm``/``v_mm``, since those exist to show where the cart actually
        was; extrapolating them too would make ``--record``'s own diagnostic
        image lie about that. This is a heuristic tuned to one measured
        delay estimate, not a general smoothing knob -- a value that is too
        large overshoots most visibly right as the cart decelerates or
        reverses, since the extrapolation still uses the velocity from just
        before that happened.
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
        #
        # _page_origin_pinned skips this entirely: the operator already placed
        # the origin with the STARTPOINT button while idle (see
        # _set_page_origin), and blindly re-zeroing at whatever pose START
        # happens to catch would silently throw that placement away -- the
        # same "explicit beats blind auto-capture" rule --simple-boresight
        # follows for the yaw reference just below.
        if t.page_frame == "simple" and self._page_origin_pinned:
            if not pj:
                print("[simple] using the page origin placed with the "
                      "STARTPOINT button -- NOT re-zeroed at this START.")
        elif t.page_frame == "simple":
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
        prev_printed = coverage.fired.copy() if pj else None
        done_reason = None
        speed_warn_state = False   # current value of the speed-warning flag
        last_verbose_t = None      # throttle for --verbose's live status line
        verbose_flushed = False    # see the finally block's newline flush below

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

                # STARTPOINT button = STOP in page mode (see the docstring).
                # Checked before reading the tracker so a press lands within
                # one poll interval rather than after another full sample's
                # worth of dosing/sending.
                if startpoint_event is not None and startpoint_event.is_set():
                    startpoint_event.clear()
                    done_reason = "stopped"
                    if not pj:
                        if self.ble.verbose:
                            print()   # end the overwriting --verbose line first
                            verbose_flushed = True
                        print("[startpoint] pass stopped by button press.")
                    break

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
                        # by half the nozzle-0-to-last-nozzle SPAN (NOT the
                        # outer bar WIDTH -- see geometry.py's
                        # NOZZLE_BAR_SPAN_MM comment) along the CURRENT (yaw-
                        # rotated) bar direction -- bar_offset_uv is the same
                        # formula CoverageEngine.step() places every
                        # individual nozzle with (see its docstring), just
                        # evaluated once per sample for the bar's midpoint
                        # rather than per nozzle.
                        ndu, ndv = bar_offset_uv(NOZZLE_BAR_SPAN_MM / 2.0, yaw_rad)
                        nozzle_path.append(
                            (int(round((v_mm + ndv) / NOZZLE_PITCH_MM)),
                             int(round((u_mm + ndu) / t.mm_per_column))))
                        sample_times.append(now - t_start)

                    samples += 1
                    u_min = u_mm if u_min is None else min(u_min, u_mm)
                    u_max = u_mm if u_max is None else max(u_max, u_mm)
                    v_min = v_mm if v_min is None else min(v_min, v_mm)
                    v_max = v_mm if v_max is None else max(v_max, v_mm)

                    # vx/vy (mm/s along u/v) are the same backward finite
                    # difference `speed` (below) has always used, just kept
                    # as signed components instead of collapsed into a
                    # magnitude -- --latency-compensate-s (see below) needs
                    # the direction, not just how fast.
                    speed = None
                    vx = vy = 0.0
                    if prev_u is not None and now > prev_t:
                        dt_uv = now - prev_t
                        vx = (u_mm - prev_u) / dt_uv
                        vy = (v_mm - prev_v) / dt_uv
                        speed = (vx ** 2 + vy ** 2) ** 0.5
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

                    # --latency-compensate-s: linearly extrapolate ONLY the
                    # coordinates fed to the coverage engine, forward along
                    # the current velocity estimate, to correct for the
                    # measured position-read -> ink-placed pipeline delay
                    # (BLE connection interval + firmware queue + fire slot,
                    # see the CLI flag's help text for the numbers). u_mm/
                    # v_mm themselves stay the real, uncompensated position
                    # for everything else below (path recording, the
                    # out-of-page bounds, the profiler, the speed warning
                    # above) -- those exist to show where the cart actually
                    # was, and compensating them too would make --record's
                    # own diagnostic path lie about that. 0.0 (default)
                    # short-circuits to exactly today's behaviour.
                    u_fire, v_fire = _extrapolate_uv(u_mm, v_mm, vx, vy,
                                                     self.latency_compensate_s)

                    pattern, changed = coverage.step(u_fire, v_fire, now, yaw_rad=yaw_rad)
                    if coverage.last_in_bounds:
                        in_bounds_samples += 1
                    if changed:
                        sender.send(pattern)
                        if profiler is not None:
                            profiler.record_page_sample(u_mm, v_mm, speed, quat=quat)

                    # Live --verbose status line (see this method's docstring):
                    # the --pos equivalent, but usable while an actual pass is
                    # running instead of only as a separate standalone check.
                    if self.ble.verbose and not pj and (
                            last_verbose_t is None
                            or now - last_verbose_t >= _VERBOSE_STATUS_INTERVAL_S):
                        last_verbose_t = now
                        v_row = int(round(v_mm / NOZZLE_PITCH_MM))
                        v_col = (int(round(u_mm / t.mm_per_column))
                                if t.mm_per_column else 0)
                        # Physically-inked count, same quantity the pass-end
                        # summary and the rendered COVERED panel report --
                        # a live number that later disagreed with the final
                        # one would be worse than no live number at all.
                        covered = int((coverage.ink & coverage.fired).sum())
                        total = int(coverage.ink.sum())
                        line = (f"x={pos[0]:9.2f}  y={pos[1]:9.2f}  z={pos[2]:9.2f} mm  |  "
                               f"page u={u_mm:8.2f}  v={v_mm:8.2f} mm  "
                               f"row={v_row:4d} col={v_col:5d}  |  "
                               f"yaw={math.degrees(yaw_rad):+6.2f} deg  "
                               f"roll={math.degrees(mapper.last_roll_rad):+6.2f} deg  "
                               f"pitch={math.degrees(mapper.last_pitch_rad):+6.2f} deg  |  "
                               f"covered {covered}/{total}")
                        print(line, end="\r", flush=True)

                    # Nothing has ever been in bounds yet: say so periodically
                    # instead of running the whole pass in silence (plain-text
                    # mode only -- progress-json carries this in coverage_done
                    # for the UI instead, see below).
                    if not pj and in_bounds_samples == 0:
                        if last_warn_t is None or now - last_warn_t >= 2.0:
                            last_warn_t = now
                            req_u = self.width * t.mm_per_column
                            req_v = NOZZLE_BAR_SPAN_MM
                            print(f"[warn] cart not over the target page yet: "
                                  f"u={u_mm:7.2f} v={v_mm:7.2f} mm (need u in "
                                  f"[0, {req_u:.1f}] mm, v within "
                                  f"+/-{req_v:.1f} mm of the calibration origin)")

                    new_cells = []
                    if pj:
                        # NOT gated on `changed`: a nozzle's dose completing
                        # updates its mask entry on this tick, but its bit
                        # in `pattern` only flips off on the *next* tick (once
                        # wanted becomes False for that pixel) -- so `changed`
                        # lags a fresh completion by one sample and would miss
                        # it here if the pass ends (coverage.done) before that
                        # next tick ever happens.
                        #
                        # Diffed against `fired`, not `printed`: the UI canvas
                        # draws what is on the paper, and must agree with the
                        # COVERED panel and the pass-end count rather than
                        # showing gaps that only exist in the dose bookkeeping
                        # (see CoverageEngine.fired).
                        new_mask = coverage.fired & ~prev_printed
                        if new_mask.any():
                            rows, cols = np.nonzero(new_mask)
                            new_cells = list(zip(rows.tolist(), cols.tolist()))
                            prev_printed = coverage.fired.copy()

                    if pj:
                        col = int(round(u_mm / t.mm_per_column)) if t.mm_per_column else 0
                        row = int(round(v_mm / NOZZLE_PITCH_MM))
                        print(json.dumps({"event": "coverage", "u": round(u_mm, 3),
                                          "v": round(v_mm, 3), "row": row, "col": col,
                                          "new_cells": new_cells}), flush=True)

                if coverage.done:
                    done_reason = "complete"
                    if not pj:
                        if self.ble.verbose:
                            print()   # end the overwriting --verbose line first
                            verbose_flushed = True
                        print("Page fully covered.")
                    break
                if now - t_start > t.timeout_s:
                    done_reason = "timeout"
                    if not pj:
                        if self.ble.verbose:
                            print()
                            verbose_flushed = True
                        print("Freehand pass timed out.")
                    break
                await asyncio.sleep(interval)
        finally:
            # The --verbose status line ends every write with `\r`, not `\n`
            # (see the throttled block above), so it can overwrite itself in
            # place; an exception/KeyboardInterrupt breaking out of the loop
            # above skips the two explicit newline flushes next to "Page
            # fully covered."/"Freehand pass timed out." above (verbose_flushed
            # stays False on that path), so this covers it too -- guarded on
            # the flag rather than unconditional so the normal-exit paths
            # above don't also print a second, redundant blank line here.
            if self.ble.verbose and not pj and not verbose_flushed:
                print()
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
                                   sample_times=sample_times,
                                   fired=coverage.fired):
                    if not pj:
                        print(f"Coverage reconstruction -> {self.record}")
                elif not pj:
                    print("Nothing was recorded (nothing printed).")

            # "Covered" counts where ink physically LANDED, matching the
            # rendered image's COVERED panel and the paper itself -- not the
            # dose-completion mask, which undercounts badly above ~2 samples
            # per column (see CoverageEngine.fired). full_dose is reported
            # alongside it whenever the two differ, since that gap is the
            # actionable "slow down" signal rather than a coverage hole.
            covered = int((coverage.ink & coverage.fired).sum())
            full_dose = int((coverage.ink & coverage.printed).sum())
            total = int(coverage.ink.sum())
            if pj:
                print(json.dumps({"event": "coverage_done", "reason": done_reason,
                                  "covered": covered, "total": total,
                                  "full_dose": full_dose,
                                  "in_bounds_samples": in_bounds_samples,
                                  "samples": samples,
                                  "u_min": round(u_min, 3) if u_min is not None else None,
                                  "u_max": round(u_max, 3) if u_max is not None else None,
                                  "v_min": round(v_min, 3) if v_min is not None else None,
                                  "v_max": round(v_max, 3) if v_max is not None else None}),
                      flush=True)
            elif (in_bounds_samples == 0 and samples > 0
                    and done_reason != "stopped"):
                # done_reason "stopped" is excluded: an operator who aborts
                # with the STARTPOINT button before ever reaching the page
                # has not hit the bug this diagnosis describes, and its
                # advice ("the calibration origin/axes do not correspond to
                # where you think the page corner is") would be actively
                # misleading about a pass that simply ended early on purpose.
                req_u = self.width * t.mm_per_column
                req_v = NOZZLE_BAR_SPAN_MM
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
                # Only when it actually differs: on a normally-paced pass the
                # two are equal and a second identical-looking number would
                # just be noise.
                if full_dose < covered:
                    thin = covered - full_dose
                    print(f"  of those, {thin} got ink but never completed "
                          f"--dose-hold-s ({full_dose}/{total} did) -- the cart "
                          f"passed over them faster than the dose needs, so "
                          f"they are inked but lighter. Move slower, or raise "
                          f"--poll-hz, if the print looks faint there.")

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
