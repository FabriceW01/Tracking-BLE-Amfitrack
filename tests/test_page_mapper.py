"""
Milestone 3 tests: PageMapper (tracking.py) and its --pos wiring (no hardware).

PageCalibration's own projection math is already covered in depth by
tests/test_calibration.py; what's new here is PageMapper's thin wiring onto a
loaded calibration, and that diagnostics.monitor_position actually threads a
--page-calibration path through to live page_u/page_v/page_z output.

Run with:  python tests/test_page_mapper.py
"""

import asyncio
import contextlib
import io
import json
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead import diagnostics                          # noqa: E402
from printhead.calibration import PageCalibration           # noqa: E402
from printhead.config import TrackingSettings                # noqa: E402
from printhead.tracking import PageMapper                    # noqa: E402


# ================================================================= PageMapper
def test_page_mapper_delegates_to_the_calibration():
    cal = PageCalibration(origin=np.array([1.0, 2.0, 3.0]),
                          e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    mapper = PageMapper(cal)
    pos = np.array([11.0, 7.0, 3.0])
    assert mapper.project(pos) == cal.project(pos)


def test_page_mapper_reflects_scale_correction():
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]), scale_col=2.0)
    mapper = PageMapper(cal)
    u, v, z = mapper.project(np.array([5.0, 5.0, 0.0]))
    assert abs(u - 10.0) < 1e-9      # 5mm raw * scale_col 2.0
    assert abs(v - 5.0) < 1e-9       # scale_row default 1.0
    assert abs(z) < 1e-9


# ===================================================== --pos / monitor_position
async def _run_monitor_briefly(**kwargs):
    """Run monitor_position(simulate=True, ndjson=True, ...) for a short while,
    then cancel it (like Ctrl+C) and return everything it printed."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        task = asyncio.ensure_future(diagnostics.monitor_position(
            TrackingSettings(), simulate=True, hz=50.0, ndjson=True, **kwargs))
        await asyncio.sleep(0.3)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
    return buf.getvalue()


def _events(output: str):
    # SimulatedTracker.open() prints a plain-text banner even in ndjson mode
    # (pre-existing, unrelated to this change) -- skip lines that aren't JSON,
    # same tolerance ui.server._try_parse_json applies to this same stream.
    events = []
    for line in output.splitlines():
        if not line.strip():
            continue
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return events


def test_monitor_position_reports_page_uvz_when_calibration_given():
    # Identity-ish calibration in the XY plane: SimulatedTracker's default
    # motion (50 mm/s along y) should come straight through as page_v.
    cal = PageCalibration(origin=np.zeros(3), e_col=np.array([1.0, 0.0, 0.0]),
                          e_row=np.array([0.0, 1.0, 0.0]))
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "cal.json")
        cal.save(path)
        output = asyncio.run(_run_monitor_briefly(page_calibration_path=path))

    positions = [e for e in _events(output) if e.get("event") == "position"]
    assert positions, output
    last = positions[-1]
    assert "page_u" in last and "page_v" in last and "page_z" in last
    assert abs(last["page_u"]) < 0.5              # travel is along y only
    assert abs(last["page_z"]) < 0.5
    assert abs(last["page_v"] - last["y"]) < 0.5   # e_row == y-axis, scale 1


def test_monitor_position_omits_page_uvz_without_calibration():
    output = asyncio.run(_run_monitor_briefly())
    positions = [e for e in _events(output) if e.get("event") == "position"]
    assert positions, output
    assert "page_u" not in positions[-1]


def test_monitor_position_reports_an_error_for_a_bad_calibration_path():
    output = asyncio.run(_run_monitor_briefly(
        page_calibration_path="/nonexistent/path/cal.json"))
    events = _events(output)
    assert any(e.get("event") == "error" for e in events), output
    assert not any(e.get("event") == "position" for e in events)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All page-mapper tests passed.")
