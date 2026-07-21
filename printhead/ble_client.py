"""
BLE transport to the PrintheadBLE ESP32
=======================================

Thin async wrapper around ``bleak`` that hides connection/scan handling and the
nozzle write characteristic. The firmware always reprints the *latest* frame it
received, so both print strategies boil down to "write the right 21-byte frame
at the right moment":
  * time-based   -> :meth:`stream_time` writes one column every ``period`` s;
  * position-based -> the controller calls :meth:`write_column` whenever the
    measured position crosses into a new column.
"""

from __future__ import annotations

import asyncio
from typing import Callable, Optional

from .config import BleSettings
from .geometry import (
    BLANK_FRAME,
    NOZZLE_UUID,
    START_BTN_UUID,
    STARTPOINT_UUID,
)


class PrintheadBLE:
    """Connect to the ESP32 and push nozzle frames. Use as an async context mgr."""

    def __init__(self, settings: BleSettings):
        self.settings = settings
        self._client: Optional[BleakClient] = None

    # ------------------------------------------------------------------ scan
    async def find_target(self):
        from bleak import BleakScanner   # lazy: only needed for real BLE
        s = self.settings
        if s.address:
            return s.address
        print(f"Scanning for '{s.device_name}' (timeout {s.scan_timeout:.0f}s) ...")
        dev = await BleakScanner.find_device_by_name(
            s.device_name, timeout=s.scan_timeout)
        if dev is None:
            raise RuntimeError(
                f"Device '{s.device_name}' not found. Is it advertising?")
        print(f"Found '{s.device_name}' @ {dev.address}")
        return dev

    # ------------------------------------------------------- connect / close
    async def __aenter__(self) -> "PrintheadBLE":
        from bleak import BleakClient   # lazy: only needed for real BLE
        target = await self.find_target()
        self._client = BleakClient(target)
        await self._client.connect()
        print("Connected.")
        # Make sure the printhead starts blank.
        await self.write_blank()
        return self

    async def __aexit__(self, exc_type, exc, tb):
        if self._client is not None:
            try:
                await self.write_blank()
            except Exception:
                pass
            await self._client.disconnect()
            print("Disconnected.")
        self._client = None

    # -------------------------------------------------------------- notify
    async def start_notifications(
        self,
        on_start: Callable[[int], None],
        on_startpoint: Optional[Callable[[int], None]] = None,
    ) -> None:
        """Subscribe to the START button and (optionally) the startpoint char."""
        def _start_cb(_sender, data):
            on_start(data[0] if data else 0)

        await self._client.start_notify(START_BTN_UUID, _start_cb)

        if on_startpoint is not None:
            def _sp_cb(_sender, data):
                on_startpoint(data[0] if data else 0)
            try:
                await self._client.start_notify(STARTPOINT_UUID, _sp_cb)
            except Exception as exc:
                print(f"(startpoint notify unavailable: {exc})")

    # --------------------------------------------------------------- write
    async def write_column(self, frame: bytes) -> None:
        await self._client.write_gatt_char(NOZZLE_UUID, frame, response=False)

    async def write_blank(self) -> None:
        await self._client.write_gatt_char(NOZZLE_UUID, BLANK_FRAME, response=False)

    # ------------------------------------------------------ time-based mode
    async def stream_time(self, frames, period: float, verbose: bool = False) -> None:
        """Send one column per ``period`` seconds, then a blank frame to stop."""
        total = len(frames)
        print(f"Streaming {total} columns @ {period * 1000:.1f} ms/col "
              f"(~{total * period:.2f}s)")
        loop = asyncio.get_event_loop()
        t0 = loop.time()
        for i, frame in enumerate(frames):
            await self.write_column(frame)
            # Absolute pacing to avoid cumulative drift.
            dt = (t0 + (i + 1) * period) - loop.time()
            if dt > 0:
                await asyncio.sleep(dt)
            if verbose and i % 50 == 0:
                print(f"  col {i}/{total}")
        await self.write_blank()
        print("Finished pass; sent blank frame.")
