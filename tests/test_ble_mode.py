"""
Print Mode Characteristic tests (no hardware).

Covers the bug where the client never wrote MODE_UUID: the firmware defaults
to (and silently stays in) line mode, so --mode page against real hardware
kept dosing through the line-mode column FIFO instead of the page-mode
fixed-stride held pattern, and vice versa for a line/time-mode run following
a page-mode one. See README_BLE_INTERFACE.md "2) Print Mode Characteristic"
(firmware repo) and ble_client.set_print_mode's docstring.

Two layers:
  * PrintheadBLE.set_print_mode() itself, against a fake bleak client
    (style copied from test_batching.py's FakeClient).
  * PrintController._run_ble(), which must call set_print_mode with the mode
    matching self.tracking.mode right after connecting -- verified against a
    fake PrintheadBLE substituted for controller.PrintheadBLE, since real
    bleak is not available/needed for this. This is the mutation-sensitive
    test: commenting out the set_print_mode call in _run_ble makes it fail
    (verified by hand while writing this file, see the task's mutation-check
    note in the commit message / final report).

Run with:  python tests/test_ble_mode.py
"""

import asyncio
import io
import os
import sys
from contextlib import redirect_stdout

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead import controller as controller_mod                    # noqa: E402
from printhead.ble_client import PrintheadBLE                          # noqa: E402
from printhead.config import BleSettings, RenderSettings, TrackingSettings  # noqa: E402
from printhead.controller import PrintController                       # noqa: E402
from printhead.geometry import (                                       # noqa: E402
    IMAGE_HEIGHT, MODE_UUID, NOZZLE_MODE_LINE, NOZZLE_MODE_PAGE,
)


class FakeClient:
    """Captures what would go out over BLE (style: tests/test_batching.py)."""

    def __init__(self, fail_mode_write=False):
        self.mtu_size = 247
        self.writes = []              # list of (uuid, payload, response)
        self.fail_mode_write = fail_mode_write

    async def write_gatt_char(self, uuid, payload, response=False):
        if self.fail_mode_write and uuid == MODE_UUID:
            raise RuntimeError("simulated ATT failure")
        self.writes.append((uuid, bytes(payload), response))


def _ble(fail_mode_write=False):
    ble = PrintheadBLE(BleSettings())
    ble._client = FakeClient(fail_mode_write=fail_mode_write)
    return ble


# ===================================================== PrintheadBLE.set_print_mode
def test_set_print_mode_page_writes_the_page_byte_with_response():
    ble = _ble()
    ok = asyncio.run(ble.set_print_mode(NOZZLE_MODE_PAGE))
    assert ok is True
    assert ble._client.writes == [(MODE_UUID, b"\x01", True)]


def test_set_print_mode_line_writes_the_line_byte_with_response():
    ble = _ble()
    ok = asyncio.run(ble.set_print_mode(NOZZLE_MODE_LINE))
    assert ok is True
    assert ble._client.writes == [(MODE_UUID, b"\x00", True)]


def test_set_print_mode_required_failure_raises_with_a_real_diagnosis():
    ble = _ble(fail_mode_write=True)
    try:
        asyncio.run(ble.set_print_mode(NOZZLE_MODE_PAGE, required=True))
    except RuntimeError as exc:
        msg = str(exc).lower()
        assert "mode" in msg and "firmware" in msg, msg
        return
    raise AssertionError("expected RuntimeError when a required mode write fails")


def test_set_print_mode_optional_failure_returns_false_without_raising():
    ble = _ble(fail_mode_write=True)
    out = io.StringIO()
    with redirect_stdout(out):
        ok = asyncio.run(ble.set_print_mode(NOZZLE_MODE_LINE, required=False))
    assert ok is False
    assert ble._client.writes == []          # the write itself never landed
    assert out.getvalue().strip() != ""      # but the failure was not silent


# ========================================================== PrintController._run_ble
class _FakeConnectedBLE:
    """
    Stands in for PrintheadBLE inside _run_ble, avoiding a dependency on real
    bleak (not installed / not needed for this). Records set_print_mode calls
    so the test can check *which* mode _run_ble selects, without caring how
    PrintheadBLE itself performs the write (that is covered above).
    """
    instances = []

    def __init__(self, settings):
        self.settings = settings
        self.mode_calls = []
        _FakeConnectedBLE.instances.append(self)

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc, tb):
        return False

    async def start_notifications(self, on_start, on_startpoint=None):
        pass

    async def set_print_mode(self, mode, required=True):
        self.mode_calls.append((mode, required))
        return True

    async def stream_time(self, frames, period, verbose=False):
        pass

    async def write_blank(self):
        pass


def _mode_calls_for(mode):
    """
    Run one full _run_ble() pass for the given tracking mode and return the
    (mode, required) pairs it passed to set_print_mode.

    tracking.enabled=False keeps this reachable without a real/simulated
    tracker: line/page mode then hit their own early-exit path (no page
    calibration / no tracker) which _run_ble's per-pass try/except swallows,
    same as any other pass failure -- exactly the behaviour a real "firmware
    predates the mode characteristic" failure would also go through. auto_start
    + once make it do exactly one pass without waiting on a START button.
    """
    render = RenderSettings(text="mode test")
    ble_settings = BleSettings(auto_start=True, once=True, period=0.001)
    trk = TrackingSettings(mode=mode, enabled=False, timeout_s=0.2)
    if mode == "page":
        ink = np.ones((10, 3), dtype=bool)
    else:
        ink = np.zeros((IMAGE_HEIGHT, 3), dtype=bool)
    ctrl = PrintController(render, ble_settings, trk, ink=ink)

    _FakeConnectedBLE.instances.clear()
    original = controller_mod.PrintheadBLE
    controller_mod.PrintheadBLE = _FakeConnectedBLE
    try:
        out = io.StringIO()
        with redirect_stdout(out):
            asyncio.run(ctrl._run_ble())
    finally:
        controller_mod.PrintheadBLE = original

    assert len(_FakeConnectedBLE.instances) == 1
    return _FakeConnectedBLE.instances[0].mode_calls


def test_run_ble_selects_page_mode_as_a_hard_requirement():
    assert _mode_calls_for("page") == [(NOZZLE_MODE_PAGE, True)]


def test_run_ble_selects_line_mode_best_effort():
    assert _mode_calls_for("line") == [(NOZZLE_MODE_LINE, False)]


def test_run_ble_selects_line_mode_best_effort_for_time_mode_too():
    # time mode also drives the column FIFO (via stream_time), so it needs
    # the same firmware mode as line mode, selected the same tolerant way.
    assert _mode_calls_for("time") == [(NOZZLE_MODE_LINE, False)]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All print-mode tests passed.")
