"""
BLE "latest wins" pattern sender
==================================

``write_columns`` (see ``ble_client.py``) is correctly a FIFO: every column
carries unique, never-repeated content, so queuing and sending them in order
is exactly right. ``PatternSender`` is the deliberate opposite, for freehand
page mode: the coverage engine's pattern is a live *snapshot* of which
nozzles should currently be firing, not a sequence of distinct values -- an
intermediate snapshot already superseded by a newer one before it went out
is worthless, so queuing it would only add latency for no benefit.

This is a single-slot mailbox instead: :meth:`PatternSender.send` replaces
whatever hasn't been sent yet, and a background task always transmits
whatever was most recently handed to it.
"""

from __future__ import annotations

import asyncio
from typing import Optional


class PatternSender:
    """
    Keeps the printhead's nozzle state in sync with the latest pattern handed
    to :meth:`send` via a background task, dropping any intermediate pattern
    superseded before it was sent.

    Must be constructed from a running event loop (starts its background task
    immediately). Call :meth:`close` to stop it, e.g. at the end of a print
    pass -- nothing sent through :meth:`send` after that is delivered.
    """

    def __init__(self, ble):
        self._ble = ble
        self._latest: Optional[bytes] = None
        self._dirty = asyncio.Event()
        self._task = asyncio.ensure_future(self._run())
        # A failed write is dropped, not retried -- the next send() will
        # naturally supersede it with whatever is current by then, which is
        # the same "latest wins" contract as a slow write. Recorded so a
        # caller can notice a BLE link that's failing repeatedly without
        # having to poll write-by-write.
        self.last_error: Optional[Exception] = None

    def send(self, pattern: bytes) -> None:
        """Queue ``pattern`` to be sent, replacing anything not yet sent."""
        self._latest = pattern
        self._dirty.set()

    async def _run(self) -> None:
        try:
            while True:
                await self._dirty.wait()
                self._dirty.clear()
                pattern = self._latest
                try:
                    await self._ble.write_pattern(pattern)
                except Exception as exc:
                    self.last_error = exc
        except asyncio.CancelledError:
            pass

    async def close(self) -> None:
        """Stop the background task and wait for it to finish."""
        self._task.cancel()
        try:
            await self._task
        except asyncio.CancelledError:
            pass
