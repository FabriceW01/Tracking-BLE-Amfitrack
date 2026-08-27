"""
BLE pattern sender (page mode)
===============================

Feeds the firmware's column FIFO with the coverage engine's current nozzle
pattern.

**Why this is no longer "latest wins".** It used to be: the firmware held the
last pattern it received and re-fired it every ``PATTERN_STRIDE`` ticks, so an
intermediate snapshot superseded before it went out was genuinely worthless --
the held one kept inking either way. The firmware now fires each column it
receives **exactly once** and never repeats, so a dropped pattern is dropped
*ink*. Coalescing would silently thin the print exactly where the link is
busiest.

What replaces it: a bounded queue of columns. :meth:`send` appends ``copies``
of a pattern -- that count IS the ink, decided by the caller from how far the
cart travelled (see ``PrintController._print_freehand_pass``) -- and the
background task drains it, packing as many columns into each BLE write as the
negotiated MTU allows.

The queue is bounded because BLE can be slower than the demand for a moment
and an unbounded backlog would print ink further and further behind the cart.
On overflow the OLDEST columns are dropped and counted in :attr:`dropped`: a
stale column would land at a position the cart has already left, so if
something has to go, it is the one that would be most wrong. The count is
surfaced rather than swallowed, because silent ink loss is what this whole
redesign exists to prevent.

Sizing: at the measured median hand speed (17.3 mm/s), 3 drops per pixel and
0.087 mm columns the demand is ~600 columns/s, while the measured BLE ceiling
(~270 writes/s, 12 columns per write at MTU 247) is ~3200 columns/s. The
queue is therefore expected to sit near empty; it exists for bursts, not as a
working buffer.
"""

from __future__ import annotations

import asyncio
from collections import deque
from typing import Optional

# Columns the firmware accepts in one write (BLE_NOZZLE_MAX_COLS_PER_WRITE).
# Imported rather than restated: PrintheadBLE already clamps its own
# MTU-derived batch size to it, and a second copy here would be free to drift
# away from the firmware's real limit.
from .ble_client import MAX_BATCH_COLS as MAX_COLS_PER_WRITE

# Backlog ceiling, in columns. Deliberately smaller than the firmware's own
# 128-column FIFO: this queue is upstream of it, so anything sitting here is
# waiting behind a full firmware buffer as well. At the firmware's 300 us tick
# 64 columns is ~19 ms of already-committed output -- about 0.3 mm of travel
# at median hand speed, i.e. still close enough to be printed where it was
# meant to go.
MAX_QUEUE_COLS = 64


class PatternSender:
    """
    Streams nozzle columns to the printhead from a background task.

    Construct from a running event loop (it starts its task immediately) and
    :meth:`close` it at the end of a pass.
    """

    def __init__(self, ble, max_queue_cols: int = MAX_QUEUE_COLS):
        self._ble = ble
        self._queue: "deque[bytes]" = deque()
        self._max = max(1, int(max_queue_cols))
        self._dirty = asyncio.Event()
        self._task = asyncio.ensure_future(self._run())
        # A failed write is dropped, not retried -- by the time a retry went
        # out the cart has moved, so the column would land in the wrong place.
        # Recorded so a caller can notice a link that is failing repeatedly
        # without polling write by write.
        self.last_error: Optional[Exception] = None
        # Columns handed over and columns thrown away for lack of room. Both
        # are ink, so both are countable: `sent` is what the paper should
        # show, `dropped` is what it is missing.
        self.sent = 0
        self.dropped = 0

    def send(self, pattern: bytes, copies: int = 1) -> None:
        """
        Queue ``copies`` fires of ``pattern``.

        ``copies`` is the ink: the firmware fires each queued column once, so
        three copies are three drops. Values below 1 queue nothing rather than
        being clamped up -- a caller that computed "no drops owed this sample"
        means it.
        """
        n = int(copies)
        if n < 1:
            return
        for _ in range(n):
            if len(self._queue) >= self._max:
                self._queue.popleft()      # oldest = most out of date
                self.dropped += 1
            self._queue.append(pattern)
        self._dirty.set()

    @property
    def pending(self) -> int:
        """Columns queued but not yet handed to BLE."""
        return len(self._queue)

    async def _run(self) -> None:
        try:
            while True:
                await self._dirty.wait()
                self._dirty.clear()
                while self._queue:
                    # Pack as many columns as the MTU allows into one write:
                    # the firmware queues them all and fires them one per
                    # tick, so a batch costs one round trip instead of N.
                    grenze = min(self._batch_cols(), MAX_COLS_PER_WRITE,
                                 len(self._queue))
                    chunk = [self._queue.popleft() for _ in range(grenze)]
                    nutzlast = chunk[0] if len(chunk) == 1 else b"".join(chunk)
                    try:
                        await self._ble.write_pattern(nutzlast)
                        self.sent += len(chunk)
                    except Exception as exc:
                        self.last_error = exc
        except asyncio.CancelledError:
            pass

    def _batch_cols(self) -> int:
        """How many columns the transport says fit in one write."""
        return max(1, int(getattr(self._ble, "batch_cols", 1) or 1))

    async def close(self) -> None:
        """
        Stop the background task and wait for it to finish.

        Anything still queued is abandoned, deliberately: a pass is over when
        this is called, and flushing the backlog would fire those columns
        after the cart has stopped, i.e. onto whatever it happens to be
        sitting on. At most :data:`MAX_QUEUE_COLS` columns (~0.3 mm of travel
        at median hand speed) can be lost this way, and only if the link was
        already behind.
        """
        self._task.cancel()
        try:
            await self._task
        except asyncio.CancelledError:
            pass
