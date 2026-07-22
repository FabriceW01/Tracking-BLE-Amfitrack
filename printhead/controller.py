"""
Print controller
=================

Ties rendering, BLE transport and Amfitrack tracking together and runs a print
pass in one of two modes:

  * ``position`` - closed loop: read the sensor position, convert it to a column
    index and fire that column. The horizontal scale is set by ``mm_per_column``
    and no longer depends on the cart speed.
  * ``time`` - legacy: stream one column every ``period`` seconds.
"""

from __future__ import annotations

import asyncio
from typing import Optional

import numpy as np

from .ble_client import PrintheadBLE
from .config import BleSettings, NozzleMapSettings, RenderSettings, TrackingSettings
from .nozzle_map import remap_rows
from .rendering import frames_from_ink, render_text, save_preview
from .tracking import AdvanceMapper, make_tracker

# How long the head may sit still (having accumulated < min_move_mm) before we
# stop firing its column. Tolerates slow feed while preventing a stationary blob.
_STALL_GRACE_S = 0.2


class _NullPrinthead:
    """Stand-in for PrintheadBLE used by ``--dry-run --simulate`` (no BLE)."""

    def __init__(self):
        self.column_writes = 0
        self.blank_writes = 0

    async def write_column(self, frame):
        self.column_writes += 1

    async def write_blank(self):
        self.blank_writes += 1


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
                 profile: bool = False, profile_csv: Optional[str] = None):
        self.render = render
        self.ble = ble
        self.tracking = tracking
        self.simulate = simulate
        self.preview = preview
        self.dry_run = dry_run
        self.profile = profile
        self.profile_csv = profile_csv

        # Rendered once up front, unless the caller already built the ink
        # (calibration ruler / test patterns bypass text rendering entirely).
        if ink is None:
            ink = render_text(render)
        if nozzle_map is not None and nozzle_map.block_size:
            ink = remap_rows(ink, nozzle_map.block_size, nozzle_map.order)
        self.frames = frames_from_ink(ink)
        self.width = len(self.frames)
        self._ink = ink
        print(f"Rendered '{render.text}' -> {self.width} columns x "
              f"{ink.shape[0]} rows")

    # ------------------------------------------------------------------ run
    async def run(self) -> None:
        if self.preview:
            save_preview(self._ink, self.preview)
            print(f"Preview written to {self.preview}")

        if self.dry_run:
            if self.simulate and self.tracking.mode == "position":
                await self._dry_run_position_pass()
            print("Dry run: not connecting to BLE.")
            return

        if not self.frames:
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
        use_position = self.tracking.enabled and self.tracking.mode == "position"

        async with PrintheadBLE(self.ble) as ble:
            await ble.start_notifications(on_start, on_startpoint)

            if use_position:
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
                        if use_position:
                            await self._print_position_pass(ble, tracker, startpoint_event)
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
    async def _print_position_pass(self, ble, tracker, startpoint_event) -> None:
        """Fire the column that matches the measured head position.

        A startpoint-button press during the pass re-zeros the origin at the
        current position and resets the frontier, restarting the print from
        column 0."""
        t = self.tracking
        mapper = AdvanceMapper(t)
        loop = asyncio.get_event_loop()
        interval = 1.0 / t.poll_hz

        # 1) establish the origin (button = current pos; startpoint = wait first)
        if t.origin == "startpoint":
            print("Waiting for startpoint signal to zero position ...")
            await startpoint_event.wait()
        origin = await self._wait_for_position(tracker, loop)
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
                origin = await self._wait_for_position(tracker, loop)
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
                pos = np.asarray(pos, dtype=float)
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
                            firing = False
                    elif col > frontier:
                        # advancing into new territory: print each new column
                        # once, filling any columns skipped by a fast feed.
                        start = col if frontier < 0 else frontier + 1
                        for c in range(start, col + 1):
                            if profiler is None:
                                await ble.write_column(self.frames[c])
                            else:
                                tw = loop.time()
                                await ble.write_column(self.frames[c])
                                profiler.record_write(c, adv, loop.time() - tw, speed)
                        frontier = col
                        firing = True
                    elif col < frontier:
                        # moving back over already-printed columns: do NOT
                        # reprint -> blank so no ink is deposited on the return.
                        if firing:
                            await ble.write_blank()
                            firing = False
                    # col == frontier while moving: keep the leading column firing

            if now - t_start > t.timeout_s:
                print("Position pass timed out.")
                break
            await asyncio.sleep(interval)

        if profiler is not None:
            profiler.finish()
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

    # ---------------------------------------------- dry-run simulation path
    async def _dry_run_position_pass(self) -> None:
        """Run the position loop against a null printhead and report coverage."""
        tracker = make_tracker(self.tracking, simulate=True)
        tracker.open()
        null = _NullPrinthead()
        try:
            await self._print_position_pass(null, tracker, _ImmediateEvent())
        finally:
            tracker.close()
        print(f"[sim] position loop issued {null.column_writes} column writes "
              f"for {self.width} columns.")
