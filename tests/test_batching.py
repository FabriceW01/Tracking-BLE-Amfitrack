"""
BLE column batching tests (no hardware).

Verifies that several columns are packed into one write without changing the byte
stream the firmware sees, and that the batch size follows the negotiated MTU.

Run with:  python tests/test_batching.py
"""

import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.ble_client import MAX_BATCH_COLS, PrintheadBLE   # noqa: E402
from printhead.config import BleSettings                        # noqa: E402
from printhead.geometry import NOZZLE_UUID, ROW_BYTES           # noqa: E402


class FakeClient:
    """Captures what would go out over BLE."""

    def __init__(self, mtu_size=247):
        self.mtu_size = mtu_size
        self.writes = []

    async def write_gatt_char(self, uuid, payload, response=False):
        assert uuid == NOZZLE_UUID
        self.writes.append(bytes(payload))


def _ble(mtu=247, batch_cols=0):
    ble = PrintheadBLE(BleSettings(batch_cols=batch_cols))
    ble._client = FakeClient(mtu)
    ble._resolve_batch_size()
    return ble


def _frames(n):
    return [bytes([i % 256]) * ROW_BYTES for i in range(n)]


def test_batch_size_from_mtu():
    # 247 - 3 header bytes = 244 -> 12 columns of 19 bytes.
    assert _ble(mtu=247).batch_cols == 12
    # Default 23-byte MTU only fits a single column.
    assert _ble(mtu=23).batch_cols == 1
    # A backend that cannot report the MTU must stay on the safe side.
    assert _ble(mtu=0).batch_cols == 1
    # Never exceed what the firmware accepts.
    assert _ble(mtu=4096).batch_cols == MAX_BATCH_COLS


def test_explicit_batch_cols_wins():
    assert _ble(mtu=247, batch_cols=1).batch_cols == 1
    assert _ble(mtu=23, batch_cols=8).batch_cols == 8
    assert _ble(mtu=4096, batch_cols=999).batch_cols == MAX_BATCH_COLS


def test_byte_stream_is_unchanged_by_batching():
    frames = _frames(29)

    ble = _ble(mtu=247)                       # 12 per write
    asyncio.run(ble.write_columns(frames))
    batched = ble._client.writes

    one_at_a_time = _ble(mtu=23)              # 1 per write
    asyncio.run(one_at_a_time.write_columns(frames))

    # Fewer packets ...
    assert len(batched) == 3, len(batched)
    assert len(one_at_a_time._client.writes) == 29
    # ... carrying exactly the same bytes in exactly the same order.
    assert b"".join(batched) == b"".join(one_at_a_time._client.writes)
    assert b"".join(batched) == b"".join(frames)
    # Every write is a whole number of columns, none oversized.
    for w in batched:
        assert len(w) % ROW_BYTES == 0 and 0 < len(w) <= 12 * ROW_BYTES


def test_single_column_write_is_plain():
    ble = _ble(mtu=247)
    frames = _frames(1)
    asyncio.run(ble.write_columns(frames))
    assert ble._client.writes == [frames[0]]
    assert len(ble._client.writes[0]) == ROW_BYTES


def test_empty_write_is_a_noop():
    ble = _ble(mtu=247)
    asyncio.run(ble.write_columns([]))
    assert ble._client.writes == []


def test_write_pattern_sends_a_single_plain_frame():
    ble = _ble(mtu=247)                       # batches 12 -- must not apply here
    pattern = bytes([0xAB]) * ROW_BYTES
    asyncio.run(ble.write_pattern(pattern))
    assert ble._client.writes == [pattern]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All batching tests passed.")
