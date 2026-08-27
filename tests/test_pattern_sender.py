"""
PatternSender tests (no hardware): the bounded column QUEUE that replaced the
old "latest wins" mailbox when the firmware stopped repeating a held pattern.

The distinction these tests exist to pin down: a queued column is now one
physical fire, so dropping one is dropping ink. Nothing may be coalesced, and
anything the queue does have to throw away has to be counted.

Uses a lightweight fake BLE object (an async write_pattern plus a batch_cols
attribute) rather than PrintheadBLE -- PatternSender only depends on those. A
``_gate`` asyncio.Event lets a test hold a write "in flight" so the
interleaving with further send() calls is deterministic instead of
timing-sensitive.

Run with:  python tests/test_pattern_sender.py
"""

import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.pattern_sender import (                             # noqa: E402
    MAX_COLS_PER_WRITE, PatternSender,
)


class _FakeBLE:
    def __init__(self, batch_cols=1):
        self.sent = []                # one entry per write_pattern call
        self.batch_cols = batch_cols
        self.fail_next = False
        self._gate = asyncio.Event()
        self._gate.set()          # open by default; a test can .clear() it

    async def write_pattern(self, pattern):
        await self._gate.wait()
        if self.fail_next:
            self.fail_next = False
            raise RuntimeError("simulated BLE failure")
        self.sent.append(pattern)


def _p(n: int) -> bytes:
    return bytes([n]) * 19


def _cols(ble):
    """Every column that reached the fake, in order, unbatched."""
    out = []
    for w in ble.sent:
        for i in range(0, len(w), 19):
            out.append(w[i:i + 19])
    return out


async def _settle(n: float = 0.05):
    await asyncio.sleep(n)


def test_single_send_is_delivered():
    async def run():
        ble = _FakeBLE()
        sender = PatternSender(ble)
        sender.send(_p(1))
        await _settle()
        await sender.close()
        assert ble.sent == [_p(1)]
        assert sender.sent == 1 and sender.dropped == 0
    asyncio.run(run())


def test_copies_are_queued_once_each_and_never_merged():
    # The core of the new firing model: `copies` IS the ink. Three copies must
    # reach the firmware as three separate columns, because it fires each one
    # exactly once. (One write may carry all three -- see the batching test --
    # but never fewer than three columns' worth of payload.)
    async def run():
        ble = _FakeBLE()
        sender = PatternSender(ble)
        sender.send(_p(7), copies=3)
        await _settle()
        await sender.close()
        assert _cols(ble) == [_p(7), _p(7), _p(7)]
        assert sender.sent == 3
    asyncio.run(run())


def test_sends_made_while_a_write_is_in_flight_are_NOT_coalesced():
    # The behaviour this class used to have, now a bug: _p(2) is superseded by
    # _p(3) before the link frees up, but under a fire-once firmware _p(2) is
    # a drop of ink that was owed, not a stale snapshot.
    async def run():
        ble = _FakeBLE()
        ble._gate.clear()                 # hold the first write open
        sender = PatternSender(ble)
        sender.send(_p(1))
        await _settle()                   # background task now blocked on _p(1)
        sender.send(_p(2))
        sender.send(_p(3))
        await _settle()
        assert sender.pending == 2        # both still owed, neither discarded
        ble._gate.set()                   # release the first write
        await _settle()
        await sender.close()
        assert _cols(ble) == [_p(1), _p(2), _p(3)]
        assert sender.dropped == 0
    asyncio.run(run())


def test_send_after_previous_delivery_is_not_dropped():
    async def run():
        ble = _FakeBLE()
        sender = PatternSender(ble)
        sender.send(_p(1))
        await _settle()
        sender.send(_p(2))
        await _settle()
        await sender.close()
        assert _cols(ble) == [_p(1), _p(2)]
    asyncio.run(run())


def test_copies_below_one_queue_nothing():
    # "No drops owed this sample" is a real answer from the controller's
    # accumulator, not a value to clamp up to 1.
    async def run():
        ble = _FakeBLE()
        sender = PatternSender(ble)
        sender.send(_p(1), copies=0)
        sender.send(_p(2), copies=-4)
        await _settle()
        await sender.close()
        assert ble.sent == []
        assert sender.sent == 0 and sender.dropped == 0
    asyncio.run(run())


def test_batches_up_to_the_transports_column_count_in_one_write():
    async def run():
        ble = _FakeBLE(batch_cols=4)
        ble._gate.clear()                  # let a backlog build first
        sender = PatternSender(ble)
        sender.send(_p(9), copies=10)
        await _settle()
        ble._gate.set()
        await _settle()
        await sender.close()
        # 10 columns at 4 per write -> 4 + 4 + 2, and every column still there.
        assert [len(w) // 19 for w in ble.sent] == [4, 4, 2]
        assert _cols(ble) == [_p(9)] * 10
        assert sender.sent == 10
    asyncio.run(run())


def test_batch_size_never_exceeds_the_firmwares_per_write_limit():
    # A transport claiming a bigger batch than the firmware accepts must not
    # be believed -- an over-long write is rejected outright, losing the lot.
    async def run():
        ble = _FakeBLE(batch_cols=MAX_COLS_PER_WRITE * 4)
        ble._gate.clear()
        sender = PatternSender(ble)
        sender.send(_p(5), copies=MAX_COLS_PER_WRITE + 3)
        await _settle()
        ble._gate.set()
        await _settle()
        await sender.close()
        assert [len(w) // 19 for w in ble.sent] == [MAX_COLS_PER_WRITE, 3]
    asyncio.run(run())


def test_a_missing_batch_cols_attribute_falls_back_to_one_column_per_write():
    class _Bare:
        def __init__(self):
            self.sent = []

        async def write_pattern(self, pattern):
            self.sent.append(pattern)

    async def run():
        ble = _Bare()
        sender = PatternSender(ble)
        sender.send(_p(1), copies=3)
        await _settle()
        await sender.close()
        assert ble.sent == [_p(1), _p(1), _p(1)]
    asyncio.run(run())


def test_overflow_drops_the_OLDEST_columns_and_counts_them():
    # A full queue means BLE is behind the cart. The oldest column is the one
    # whose position the cart has most definitely already left, so it is the
    # one to lose -- and the loss is real ink, so it has to be countable.
    async def run():
        ble = _FakeBLE()
        ble._gate.clear()                  # nothing drains
        sender = PatternSender(ble, max_queue_cols=4)
        for i in range(1, 7):              # all six queued before any await
            sender.send(_p(i))
        assert sender.pending == 4         # ceiling held
        assert sender.dropped == 2
        await _settle()
        ble._gate.set()
        await _settle()
        await sender.close()
        # The two OLDEST (_p(1), _p(2)) went; the four newest survived.
        assert _cols(ble) == [_p(3), _p(4), _p(5), _p(6)]
    asyncio.run(run())


def test_survives_a_failed_write_and_keeps_sending():
    async def run():
        ble = _FakeBLE()
        ble.fail_next = True
        sender = PatternSender(ble)
        sender.send(_p(1))
        await _settle()
        assert sender.last_error is not None
        assert ble.sent == []               # the failed write was never recorded

        sender.send(_p(2))
        await _settle()
        await sender.close()
        assert _cols(ble) == [_p(2)]        # sender kept working afterwards
    asyncio.run(run())


def test_close_stops_the_background_task():
    async def run():
        ble = _FakeBLE()
        sender = PatternSender(ble)
        sender.send(_p(1))
        await _settle()
        await sender.close()
        assert sender._task.done()

        sender.send(_p(2))                  # nobody is listening anymore
        await _settle()
        assert _cols(ble) == [_p(1)]
    asyncio.run(run())


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All pattern-sender tests passed.")
