"""
Web UI calibration endpoints + coverage-event relay (no browser, no hardware).

The calibration business logic (compute/save/load) is tested as plain
functions -- this project has no pytest/httpx, so ui/server.py factors the
logic out of the @app.post handlers specifically so it's callable directly.
The coverage-event relay is tested by driving a real `main.py --dry-run
--simulate --mode page --progress-json` subprocess through Hub.run_action(),
the same way tests/test_ui_runner.py drives CommandProcess directly.

Run with:  python tests/test_ui_calibration.py
"""

import asyncio
import json
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.calibration import PageCalibration                      # noqa: E402
from printhead.ui.server import (                                      # noqa: E402
    Hub, compute_calibration, load_calibration, save_calibration,
)


def _noisy_line(origin, direction, length_mm, n=30, noise_mm=0.02, seed=0):
    rng = np.random.default_rng(seed)
    t = np.linspace(0.0, length_mm, n)
    pts = np.asarray(origin) + np.outer(t, direction)
    return (pts + rng.normal(0.0, noise_mm, pts.shape)).tolist()


def _page_traces(width_mm=210.0, height_mm=297.0, row_tilt_deg=0.0):
    col_samples = _noisy_line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), width_mm, seed=0)
    tilt = np.radians(row_tilt_deg)
    row_dir = (np.sin(tilt), np.cos(tilt), 0.0)
    row_samples = _noisy_line((0.0, 0.0, 0.0), row_dir, height_mm, seed=1)
    return col_samples, row_samples


# ============================================================ compute_calibration
def test_compute_calibration_returns_a_clean_summary():
    col, row = _page_traces()
    result = compute_calibration(col, row)
    assert result["ok"] is True
    assert result["warning"] is None
    assert result["quality_warning"] is None
    assert abs(result["scale_col"] - 1.0) < 1e-6
    assert abs(result["scale_row"] - 1.0) < 1e-6
    assert {"origin", "e_col", "e_row"} <= result["calibration"].keys()


def test_compute_calibration_flags_a_skewed_trace():
    col, row = _page_traces(row_tilt_deg=30.0)     # 30 deg off perpendicular
    result = compute_calibration(col, row)
    assert result["ok"] is True                    # still computed, just warned
    assert result["warning"] is not None
    assert "deg" in result["warning"]


def test_compute_calibration_returns_quality_metrics():
    col, row = _page_traces()
    result = compute_calibration(col, row)
    assert result["ok"] is True
    q = result["quality"]
    assert abs(q["col_trace_length_mm"] - 210.0) < 1.0
    assert abs(q["row_trace_length_mm"] - 297.0) < 1.0
    assert q["col_sample_count"] == 30
    assert q["row_sample_count"] == 30
    assert q["col_rms_residual_mm"] < 0.1
    assert q["normal_tilt_deg"] < 0.1


def test_compute_calibration_flags_low_quality_separately_from_angle():
    # A short, sparse trace with no angle problem at all -- quality_warning
    # must fire independently of (and be distinguishable from) `warning`.
    col = _noisy_line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 30.0, n=8, noise_mm=0.02)
    row = _noisy_line((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 30.0, n=8, noise_mm=0.02, seed=1)
    result = compute_calibration(col, row)
    assert result["ok"] is True
    assert result["warning"] is None                       # edges are perpendicular
    assert result["quality_warning"] is not None
    assert "column" in result["quality_warning"] or "row" in result["quality_warning"]


def test_compute_calibration_applies_sheet_size_scale():
    col, row = _page_traces(width_mm=200.0, height_mm=297.0)   # raw trace short by 10mm
    result = compute_calibration(col, row, sheet_width_mm=210.0, sheet_height_mm=297.0)
    assert result["ok"] is True
    assert abs(result["scale_col"] - 210.0 / 200.0) < 1e-2


def test_compute_calibration_returns_an_error_for_degenerate_samples():
    result = compute_calibration([[0, 0, 0]], [[0, 0, 0], [1, 0, 0]])
    assert result["ok"] is False
    assert isinstance(result["error"], str) and result["error"]


def test_compute_calibration_returns_an_error_for_malformed_input():
    result = compute_calibration("not samples", [[0, 0, 0], [1, 0, 0]])
    assert result["ok"] is False


# ==================================================================== boresight
def test_compute_calibration_stores_a_given_boresight_quat():
    col, row = _page_traces()
    quat = [0.0, 0.0, 0.1305, 0.9914]
    result = compute_calibration(col, row, boresight_quat=quat)
    assert result["ok"] is True
    assert result["has_boresight"] is True
    assert np.allclose(result["calibration"]["boresight_quat"], quat)


def test_compute_calibration_has_no_boresight_when_not_given():
    col, row = _page_traces()
    result = compute_calibration(col, row)
    assert result["ok"] is True
    assert result["has_boresight"] is False
    assert "boresight_quat" not in result["calibration"]


# =================================================== save_calibration / load
def test_save_and_load_calibration_round_trip():
    col, row = _page_traces()
    computed = compute_calibration(col, row)
    assert computed["ok"]

    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "cal.json")
        saved = save_calibration(computed["calibration"], path)
        assert saved == {"ok": True, "path": path}

        loaded = load_calibration(path)
    assert loaded["ok"] is True
    assert abs(loaded["angle_error_deg"] - computed["angle_error_deg"]) < 1e-9
    assert np.allclose(loaded["calibration"]["origin"], computed["calibration"]["origin"])
    assert abs(loaded["quality"]["col_trace_length_mm"]
              - computed["quality"]["col_trace_length_mm"]) < 1e-6


def test_load_calibration_reports_none_quality_for_a_pre_feature_file():
    # A calibration JSON saved before the fit-quality feature existed (the
    # operator has one) -- load_calibration must not crash, and its quality
    # fields must come back None rather than fabricated.
    old_style = {"origin": [0.0, 0.0, 0.0], "e_col": [1.0, 0.0, 0.0],
                "e_row": [0.0, 1.0, 0.0], "scale_col": 1.0, "scale_row": 1.0,
                "angle_error_deg": 0.0}
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "old_cal.json")
        with open(path, "w") as f:
            json.dump(old_style, f)
        loaded = load_calibration(path)
    assert loaded["ok"] is True
    assert loaded["quality"]["col_trace_length_mm"] is None
    assert loaded["quality"]["normal_tilt_deg"] is None


def test_load_calibration_returns_an_error_for_a_missing_file():
    result = load_calibration("/nonexistent/path/cal.json")
    assert result["ok"] is False
    assert isinstance(result["error"], str) and result["error"]


def test_save_calibration_returns_an_error_for_an_invalid_dict():
    result = save_calibration({"not": "a calibration"}, "/tmp/should_not_be_written.json")
    assert result["ok"] is False


# ============================================================== coverage relay
class _FakeWS:
    def __init__(self):
        self.messages = []

    async def send_json(self, msg):
        self.messages.append(msg)


def test_coverage_events_are_relayed_with_the_coverage_event_type():
    col, row = _page_traces()
    computed = compute_calibration(col, row)
    assert computed["ok"]

    async def run():
        with tempfile.TemporaryDirectory() as tmp:
            cal_path = os.path.join(tmp, "cal.json")
            PageCalibration.from_dict(computed["calibration"]).save(cal_path)

            hub = Hub()
            fake = _FakeWS()
            hub.clients.add(fake)
            r = await hub.run_action([
                "Hi", "--dry-run", "--simulate", "--mode", "page",
                "--page-calibration", cal_path, "--progress-json",
                "--timeout", "0.15", "--drops-per-pixel", "2",
            ])
            assert r["ok"], r

            for _ in range(200):                 # up to ~10s
                if hub.action is not None and not hub.action.running:
                    break
                await asyncio.sleep(0.05)

        return fake.messages

    messages = asyncio.run(run())
    coverage_msgs = [m for m in messages if m.get("type") == "coverage_event"]
    assert coverage_msgs, messages
    assert any(m["event"] == "coverage_start" for m in coverage_msgs)
    assert any(m["event"] == "coverage" for m in coverage_msgs)
    # position/log events must not be mistaken for coverage events, and vice versa.
    assert not any(m.get("type") == "position" for m in coverage_msgs)


def test_plain_log_lines_are_not_misrouted_as_coverage_events():
    async def run():
        hub = Hub()
        fake = _FakeWS()
        hub.clients.add(fake)
        r = await hub.run_action(["--pattern", "solid", "--dry-run", "--simulate",
                                  "--pattern-length-mm", "10"])
        assert r["ok"], r
        for _ in range(200):
            if hub.action is not None and not hub.action.running:
                break
            await asyncio.sleep(0.05)
        return fake.messages

    messages = asyncio.run(run())
    assert not any(m.get("type") == "coverage_event" for m in messages)
    assert any(m.get("type") == "log" for m in messages)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All UI-calibration tests passed.")
