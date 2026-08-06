"""
Diagnostics print-mode-selection tests (no hardware).

Covers the bug where ``nozzle_test()`` and ``ble_benchmark()`` open
``PrintheadBLE`` directly, bypassing ``PrintController._run_ble()`` (see
test_ble_mode.py), and so never pinned the firmware to a known mode before
writing. For ``nozzle_test()`` that is not just cosmetic: its first step
writes all nozzles on and sleeps ``on_seconds`` (default 2.0s). If the
firmware is still in page mode from an earlier ``--mode page`` run, that
write becomes a held pattern re-fired every ``PATTERN_STRIDE`` ticks of the
450us print loop instead of firing ``BLE_DROPS_PER_COLUMN`` times and being
done -- roughly 2.0 / 450e-6 / 37 =~ 120 fires vs. the 3 line mode intends,
about 40x the ink, on every nozzle, at once.

This module puts a separate fake ``PrintheadBLE`` in front of
``diagnostics.py`` (an ordered operation log, monkeypatched over
``diagnostics.PrintheadBLE``) rather than extending test_ble_mode.py's
FakeClient/​_FakeConnectedBLE, since it exercises a different pair of
functions (nozzle_test/ble_benchmark, not set_print_mode or _run_ble) and the
thing under test here is strictly the *order* of calls, not their payloads --
kept in its own file to keep that distinction obvious.

Run with:  python tests/test_diagnostics_mode.py
"""

import asyncio
import io
import os
import sys
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead import diagnostics                       # noqa: E402
from printhead.config import BleSettings, TrackingSettings  # noqa: E402
from printhead.geometry import NOZZLE_MODE_LINE          # noqa: E402


class _FakeBLE:
    """
    Stands in for PrintheadBLE, recording every operation (mode select, column
    write, blank write) in call order -- that order is exactly what defect 1's
    fix depends on, so the fake must expose it rather than just a tally.
    """

    instances = []

    def __init__(self, settings):
        self.settings = settings
        self.ops = []
        _FakeBLE.instances.append(self)

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc, tb):
        return False

    async def set_print_mode(self, mode, required=True):
        self.ops.append(("mode", mode, required))
        return True

    async def write_column(self, frame, response=False):
        self.ops.append(("write_column",))

    async def write_blank(self):
        self.ops.append(("write_blank",))


def _run_with_fake_ble(coro_factory):
    """Run coro_factory() with diagnostics.PrintheadBLE swapped for _FakeBLE,
    and return the ops log of the one instance it created."""
    _FakeBLE.instances.clear()
    original = diagnostics.PrintheadBLE
    diagnostics.PrintheadBLE = _FakeBLE
    try:
        out = io.StringIO()
        with redirect_stdout(out):
            asyncio.run(coro_factory())
    finally:
        diagnostics.PrintheadBLE = original
    assert len(_FakeBLE.instances) == 1
    return _FakeBLE.instances[0].ops


def _first_index(ops, tag):
    return next(i for i, op in enumerate(ops) if op[0] == tag)


# ============================================================================
# --nozzle-test
# ============================================================================
def test_nozzle_test_selects_line_mode_before_writing_anything():
    ops = _run_with_fake_ble(
        lambda: diagnostics.nozzle_test(BleSettings(), on_seconds=0.0, sweep_step=0.0))

    assert ops[0] == ("mode", NOZZLE_MODE_LINE, False), ops
    # Ordering, not just presence: the mode write must land before the first
    # nozzle write, otherwise a page-mode hold can still turn the all-on step
    # into a continuous re-fire instead of the intended 3 drops.
    mode_idx = _first_index(ops, "mode")
    write_idx = _first_index(ops, "write_column")
    assert mode_idx < write_idx, ops


# ============================================================================
# --ble-benchmark
# ============================================================================
def test_ble_benchmark_selects_line_mode_before_writing_anything():
    ops = _run_with_fake_ble(
        lambda: diagnostics.ble_benchmark(
            BleSettings(), TrackingSettings(), n_fast=2, n_probe=1))

    assert ops[0] == ("mode", NOZZLE_MODE_LINE, False), ops
    mode_idx = _first_index(ops, "mode")
    write_idx = _first_index(ops, "write_column")
    assert mode_idx < write_idx, ops


# ============================================================================
# START-button hint (defect 1)
# ============================================================================
def test_nozzle_test_emits_the_start_button_hint():
    # Not using _run_with_fake_ble here: it captures stdout internally to
    # keep test output clean and only returns the ops log, not the text --
    # this test needs the printed text itself, so it does its own capture.
    _FakeBLE.instances.clear()
    original = diagnostics.PrintheadBLE
    diagnostics.PrintheadBLE = _FakeBLE
    try:
        out = io.StringIO()
        with redirect_stdout(out):
            asyncio.run(diagnostics.nozzle_test(BleSettings(), on_seconds=0.0, sweep_step=0.0))
    finally:
        diagnostics.PrintheadBLE = original
    text = out.getvalue()

    assert "START button" in text, text
    # And the final message must not overclaim success -- frames were sent,
    # not necessarily fired (the whole point of this defect).
    assert "Nozzle test done." not in text, text
    assert "START button" in text.split("frames sent.")[-1], (
        "the closing message should also point back at the button", text)


if __name__ == "__main__":
    tests = [v for k, v in list(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"All {len(tests)} diagnostics-mode tests passed.")
