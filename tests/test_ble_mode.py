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
    # Class-level, not constructor args: _run_ble() constructs this with a
    # single positional arg (PrintheadBLE(self.ble)), so a test that wants
    # non-default behaviour sets these on the CLASS before calling _run_ble()
    # and resets them afterwards -- same "external class-level control" idea
    # already used for `instances` above.
    fail_stream_time = False
    fail_process_stop = False

    def __init__(self, settings):
        self.settings = settings
        self.mode_calls = []
        self.stop_requests = 0
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
        if _FakeConnectedBLE.fail_stream_time:
            raise RuntimeError("simulated failure mid-pass")

    async def write_blank(self):
        pass

    async def request_process_stop(self):
        self.stop_requests += 1
        if _FakeConnectedBLE.fail_process_stop:
            # Mirrors PrintheadBLE.request_process_stop's own contract: it
            # swallows failures and returns False, never raises -- this fake
            # does the same, so a test opting into "the write fails" still
            # exercises _run_ble()'s tolerance of that, not a crash here.
            return False
        return True


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


# =============================================== PrintController._run_ble: process stop
def _run_once(mode):
    """
    Same setup as _mode_calls_for() above, but returns the whole
    _FakeConnectedBLE instance (not just its mode_calls) so a test can also
    inspect stop_requests. Resets the class-level fail_stream_time/
    fail_process_stop flags afterwards regardless of outcome, so one test's
    opt-in never leaks into the next.
    """
    render = RenderSettings(text="stop test")
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
        _FakeConnectedBLE.fail_stream_time = False
        _FakeConnectedBLE.fail_process_stop = False

    assert len(_FakeConnectedBLE.instances) == 1
    return _FakeConnectedBLE.instances[0]


def test_run_ble_requests_a_process_stop_after_every_pass():
    # line/page mode: tracking.enabled=False makes the pass itself raise
    # (no tracker), exercising the except-branch's route into the shared
    # finally. time mode: stream_time succeeds cleanly, exercising the
    # normal-completion route into the SAME finally. Both must still
    # request a stop -- the whole point is that this runs regardless of
    # how the pass ended.
    for mode in ("line", "page", "time"):
        inst = _run_once(mode)
        assert inst.stop_requests == 1, f"mode={mode}: {inst.stop_requests}"


def test_run_ble_requests_a_process_stop_even_when_the_pass_raises():
    # Explicit, not just incidental: line/page above already go through the
    # except-branch, but naming it directly documents the guarantee rather
    # than relying on a reader noticing tracking.enabled=False's side effect.
    inst = _run_once("line")
    assert inst.stop_requests == 1


def test_run_ble_process_stop_failure_does_not_crash_the_loop():
    # request_process_stop() itself never raises (see its docstring) -- this
    # confirms _run_ble() does not additionally wrap it in a way that would
    # turn "the write failed" into an uncaught exception that kills the
    # whole BLE session over a best-effort cleanup step.
    _FakeConnectedBLE.fail_process_stop = True
    inst = _run_once("time")
    assert inst.stop_requests == 1, "the attempt must still be made"


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All print-mode tests passed.")
