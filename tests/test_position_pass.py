"""
Position-loop behaviour tests (no hardware).

Verifies:
  * moving the printhead backward does not reprint already-transmitted columns
    (the frontier / no-reprint logic), and
  * a startpoint-button press mid-pass re-zeros the origin and restarts the
    print from column 0.

Run with:  python tests/test_position_pass.py
"""

import asyncio
import io
import os
import re
import sys
from contextlib import redirect_stdout

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.config import BleSettings, RenderSettings, TrackingSettings  # noqa: E402
from printhead.controller import (  # noqa: E402
    PrintController, _ImmediateEvent, _NullPrinthead,
)


class ScriptedTracker:
    """Returns a predetermined sequence of advance positions along the Y axis."""

    def __init__(self, advances_mm):
        self._seq = list(advances_mm)
        self._i = 0

    def open(self):
        pass

    def close(self):
        pass

    def read_position(self):
        if self._i < len(self._seq):
            value = self._seq[self._i]
            self._i += 1
        else:
            value = self._seq[-1]
        pos = np.zeros(3, dtype=float)
        pos[1] = value                      # advance_axis == "y"
        return pos


def _controller():
    render = RenderSettings(text="reverse test")
    ble = BleSettings()
    # mode="line": these tests exercise _print_line_pass's no-reprint-on-
    # reverse / startpoint-reset logic specifically, not mode selection.
    trk = TrackingSettings(mode="line", advance_axis="y", mm_per_column=0.2,
                           min_move_mm=0.01, smooth_ms=0.0,
                           poll_hz=1000.0, timeout_s=5.0)
    return PrintController(render, ble, trk), trk.mm_per_column


def test_no_reprint_on_reverse():
    ctrl, mmpc = _controller()
    width = ctrl.width
    assert width > 30, "test text must render wider than 30 columns"

    # forward to col 20, back to col 5, forward to col 30, then jump past the end.
    origin = [0.0]                                   # consumed by set-origin
    fwd1 = [c * mmpc for c in range(0, 21)]          # cols 0..20  (21 new)
    back = [c * mmpc for c in range(19, 4, -1)]      # cols 19..5  (no reprint)
    fwd2 = [c * mmpc for c in range(6, 31)]          # cols 6..30  (10 new: 21..30)
    end = [width * mmpc]                             # >= width -> break
    tracker = ScriptedTracker(origin + fwd1 + back + fwd2 + end)

    rec = _NullPrinthead()
    asyncio.run(ctrl._print_line_pass(rec, tracker, _ImmediateEvent()))

    # Columns 0..30 must be printed exactly once each -> 31 writes total.
    assert rec.column_writes == 31, f"expected 31 writes, got {rec.column_writes}"
    assert rec.blank_writes >= 1, "reverse motion should emit a blank frame"


class RampTracker:
    """Position along Y that grows by a fixed step on every read (always moving)."""

    def __init__(self, mm_per_call=0.2):
        self._mm = mm_per_call
        self._n = 0

    def open(self):
        pass

    def close(self):
        pass

    def read_position(self):
        pos = np.zeros(3, dtype=float)
        pos[1] = self._n * self._mm
        self._n += 1
        return pos


class FireOnce:
    """Startpoint stub whose is_set() returns True on exactly the k-th call."""

    def __init__(self, at_check):
        self._checks = 0
        self._at = at_check

    def is_set(self):
        self._checks += 1
        return self._checks == self._at

    def clear(self):
        pass

    async def wait(self):
        return True

    def set(self):
        pass


def _run_pass(ctrl, event):
    rec = _NullPrinthead()
    asyncio.run(ctrl._print_line_pass(rec, RampTracker(), event))
    return rec


def test_startpoint_reset_restarts_from_zero():
    render = RenderSettings(text="Hi")
    trk = TrackingSettings(mode="line", advance_axis="y", mm_per_column=0.2,
                           min_move_mm=0.01, smooth_ms=0.0,
                           poll_hz=1000.0, timeout_s=10.0)
    ctrl = PrintController(render, BleSettings(), trk)
    assert ctrl.width > 50, "test text should render wide enough for a mid-pass reset"

    baseline = _run_pass(ctrl, _ImmediateEvent())         # no reset
    withreset = _run_pass(ctrl, FireOnce(at_check=40))    # startpoint press at iter 40

    # After the mid-pass reset the head reprints from column 0 -> strictly more
    # column writes than a single forward pass.
    assert baseline.column_writes > 0
    assert withreset.column_writes > baseline.column_writes, (
        f"reset should reprint from 0: {withreset.column_writes} "
        f"!> {baseline.column_writes}")
    # The reset emits its own blank frame on top of the final one.
    assert withreset.blank_writes > baseline.blank_writes


# ============================================================= --verbose
def test_verbose_prints_a_live_position_status_line():
    # The --pos equivalent, but usable while an actual line-mode pass is
    # running (see _print_line_pass's docstring) -- unlike --pos itself,
    # which is one of cli.py's standalone diagnostics and cannot be combined
    # with a real print pass at all.
    render = RenderSettings(text="Hi")
    trk = TrackingSettings(mode="line", advance_axis="y", mm_per_column=0.2,
                           min_move_mm=0.01, smooth_ms=0.0,
                           poll_hz=1000.0, timeout_s=5.0)
    ctrl = PrintController(render, BleSettings(verbose=True), trk)

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_line_pass(_NullPrinthead(), RampTracker(),
                                          _ImmediateEvent()))
    text = out.getvalue()
    assert "advance=" in text and "col=" in text, text


def test_verbose_off_by_default_prints_no_status_line():
    render = RenderSettings(text="Hi")
    trk = TrackingSettings(mode="line", advance_axis="y", mm_per_column=0.2,
                           min_move_mm=0.01, smooth_ms=0.0,
                           poll_hz=1000.0, timeout_s=5.0)
    ctrl = PrintController(render, BleSettings(), trk)      # verbose=False

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_line_pass(_NullPrinthead(), RampTracker(),
                                          _ImmediateEvent()))
    assert "advance=" not in out.getvalue()


def test_verbose_status_line_does_not_garble_the_final_message():
    # REGRESSION guard: the status line ends every write with `\r`, not
    # `\n` (see the throttled block in _print_line_pass), so without the
    # trailing print() flush in the finally block, "Finished pass..." would
    # land on top of that partial line instead of a fresh one.
    render = RenderSettings(text="Hi")
    trk = TrackingSettings(mode="line", advance_axis="y", mm_per_column=0.2,
                           min_move_mm=0.01, smooth_ms=0.0,
                           poll_hz=1000.0, timeout_s=5.0)
    ctrl = PrintController(render, BleSettings(verbose=True), trk)

    out = io.StringIO()
    with redirect_stdout(out):
        asyncio.run(ctrl._print_line_pass(_NullPrinthead(), RampTracker(),
                                          _ImmediateEvent()))
    text = out.getvalue()
    assert re.search(r"(^|\n)Finished pass; sent blank frame\.\s*$", text), text


if __name__ == "__main__":
    test_no_reprint_on_reverse()
    print("OK: no-reprint-on-reverse test passed.")
    test_startpoint_reset_restarts_from_zero()
    print("OK: startpoint-reset test passed.")
    test_verbose_prints_a_live_position_status_line()
    print("OK: verbose status line test passed.")
    test_verbose_off_by_default_prints_no_status_line()
    print("OK: verbose off-by-default test passed.")
    test_verbose_status_line_does_not_garble_the_final_message()
    print("OK: verbose final-message flush test passed.")
