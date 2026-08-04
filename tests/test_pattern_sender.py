"""
PatternSender tests (no hardware): the "latest wins" mailbox that replaces
FIFO queuing for freehand page-mode pattern updates.

Uses a lightweight fake BLE object (just an async write_pattern) rather than
PrintheadBLE -- PatternSender only depends on that one method. A ``_gate``
asyncio.Event lets a test hold a write "in flight" so the interleaving with
further send() calls is deterministic instead of timing-sensitive.

Run with:  python tests/test_pattern_sender.py
"""

import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.pattern_sender import PatternSender    # noqa: E402


class _FakeBLE:
    def __init__(self):
        self.sent = []
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
    asyncio.run(run())


def test_coalesces_sends_made_while_a_write_is_in_flight():
    async def run():
        ble = _FakeBLE()
        ble._gate.clear()                 # hold the first write open
        sender = PatternSender(ble)
        sender.send(_p(1))
        await _settle()                   # background task now blocked sending _p(1)
        sender.send(_p(2))                # superseded before it is ever sent
        sender.send(_p(3))                # this is what should go out next
        await _settle()
        ble._gate.set()                   # release the first write
        await _settle()
        await sender.close()
        assert ble.sent == [_p(1), _p(3)]  # _p(2) was coalesced away
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
        assert ble.sent == [_p(1), _p(2)]   # both delivered, nothing coalesced
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
        assert ble.sent == [_p(2)]          # sender kept working afterwards
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
        assert ble.sent == [_p(1)]
    asyncio.run(run())


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All pattern-sender tests passed.")
