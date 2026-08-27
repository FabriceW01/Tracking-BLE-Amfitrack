"""
Timing-profiler tests (no hardware).

Run with:  python tests/test_profiling.py
"""

import io
import os
import sys
import time
from contextlib import redirect_stdout

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead import cli                                            # noqa: E402
from printhead.profiling import DEFAULT_BLE_WRITE_CEILING_PER_S, PassProfiler  # noqa: E402


def _run_profiler(mm_per_column, latency_s, speed, n=50, csv_path=None):
    prof = PassProfiler(mm_per_column, live=False, csv_path=csv_path)
    prof.start()
    for i in range(n):
        prof.record_write(i, i * mm_per_column, latency_s, speed)
    out = io.StringIO()
    with redirect_stdout(out):
        prof.finish()
    return prof, out.getvalue()


def test_slow_ble_reports_lag():
    # 10 ms per write -> ~100 cols/s sustained; 50 mm/s demands 250 cols/s.
    prof, report = _run_profiler(mm_per_column=0.2, latency_s=0.010, speed=50.0)
    assert prof.n_cols == 50
    assert abs(prof._sustained_rate() - 100.0) < 1.0, prof._sustained_rate()
    assert "outran the BLE" in report, report
    assert "columns lagged" in report


def test_fast_ble_keeps_up():
    # 0.1 ms per write -> ~10000 cols/s sustained; 50 mm/s demands 250 cols/s.
    _, report = _run_profiler(mm_per_column=0.2, latency_s=0.0001, speed=50.0)
    assert "kept up" in report, report


def test_max_safe_speed_scales_with_mm_per_column():
    # sustained ~100 cols/s; at 0.5 mm/col that is ~50 mm/s.
    _, report = _run_profiler(mm_per_column=0.5, latency_s=0.010, speed=5.0)
    assert "~50.0 mm/s" in report, report


def test_csv_log_written(tmp_path=None):
    path = os.path.join(
        os.environ.get("TMPDIR", "/tmp"), "printhead_profile_test.csv")
    try:
        _run_profiler(mm_per_column=0.2, latency_s=0.001, speed=10.0, n=5,
                      csv_path=path)
        with open(path) as fh:
            lines = fh.read().strip().splitlines()
        # Line mode has no orientation input -- pins the header unchanged and,
        # explicitly, that the page-mode-only quaternion columns did not leak in.
        assert lines[0] == "t_s,column,advance_mm,write_latency_ms,speed_mm_s"
        assert "qx" not in lines[0] and "qw" not in lines[0]
        assert len(lines) == 1 + 5           # header + 5 rows
    finally:
        if os.path.exists(path):
            os.remove(path)


def test_empty_profiler_is_safe():
    prof = PassProfiler(0.2, live=False)
    prof.start()
    out = io.StringIO()
    with redirect_stdout(out):
        prof.finish()
    assert "no column writes" in out.getvalue()


# ---- page mode --------------------------------------------------------
def _run_page_profiler(mm_per_column, n=20, gap_s=0.01, speed=10.0, csv_path=None,
                       ble_write_ceiling=DEFAULT_BLE_WRITE_CEILING_PER_S,
                       columns=1, batch_cols=1):
    prof = PassProfiler(mm_per_column, live=False, csv_path=csv_path, mode="page",
                        ble_write_ceiling=ble_write_ceiling,
                        batch_cols=batch_cols)
    prof.start()
    for i in range(n):
        prof.record_page_sample(u_mm=i * mm_per_column, v_mm=0.0, speed_mm_s=speed,
                                columns=columns)
        time.sleep(gap_s)
    out = io.StringIO()
    with redirect_stdout(out):
        prof.finish()
    return prof, out.getvalue()


def test_page_mode_reports_column_rate_within_ceiling():
    # ~100 columns/s (10ms gaps), well under the default 270/s ceiling.
    prof, report = _run_page_profiler(mm_per_column=1.0, n=20, gap_s=0.01)
    assert prof.page_events == 20
    assert prof.page_columns == 20
    assert "stayed within the known BLE ceiling" in report, report


def test_page_mode_counts_columns_not_sends():
    # The unit that matters is the column: the firmware fires each one it
    # receives exactly once, so three copies in one send are three drops.
    # Counting sends would under-report the load by that factor.
    prof, report = _run_page_profiler(mm_per_column=1.0, n=10, gap_s=0.01,
                                      columns=3)
    assert prof.page_events == 10
    assert prof.page_columns == 30
    assert "columns queued       : 30" in report, report


def test_page_mode_ceiling_scales_with_the_columns_packed_per_write():
    # The BLE ceiling is in WRITES; batching several columns into each one
    # raises the column ceiling proportionally. Reporting columns against a
    # writes ceiling would cry wolf at any decent poll rate.
    _, schmal = _run_page_profiler(mm_per_column=1.0, n=5, gap_s=0.01,
                                   ble_write_ceiling=100.0, batch_cols=1)
    _, breit = _run_page_profiler(mm_per_column=1.0, n=5, gap_s=0.01,
                                  ble_write_ceiling=100.0, batch_cols=12)
    assert "~100 cols/s" in schmal, schmal
    assert "~1200 cols/s" in breit, breit


def test_page_mode_flags_columns_outrunning_a_low_ceiling():
    _, report = _run_page_profiler(mm_per_column=1.0, n=20, gap_s=0.005,
                                   ble_write_ceiling=1.0)
    assert "faster than BLE can" in report, report
    # Says what the cost IS: under a fire-once firmware this is lost ink,
    # not a delayed snapshot.
    assert "ink" in report and "dropped" in report, report


def test_page_mode_csv_log_written():
    path = os.path.join(
        os.environ.get("TMPDIR", "/tmp"), "printhead_page_profile_test.csv")
    try:
        _run_page_profiler(mm_per_column=1.0, n=5, gap_s=0.002, csv_path=path)
        with open(path) as fh:
            lines = fh.read().strip().splitlines()
        assert lines[0] == \
            "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,qx,qy,qz,qw"
        assert len(lines) == 1 + 5
    finally:
        if os.path.exists(path):
            os.remove(path)


def test_page_mode_csv_header_has_quat_columns_even_without_samples():
    # Header shape is guaranteed unconditionally, so a downstream CSV parser
    # never needs to special-case a pass with zero orientation samples.
    path = os.path.join(
        os.environ.get("TMPDIR", "/tmp"), "printhead_page_profile_test_empty.csv")
    try:
        prof = PassProfiler(1.0, live=False, csv_path=path, mode="page")
        prof.start()
        prof.finish()
        with open(path) as fh:
            lines = fh.read().strip().splitlines()
        assert lines[0] == \
            "t_s,row,col,u_mm,v_mm,speed_mm_s,cols_per_s,qx,qy,qz,qw"
    finally:
        if os.path.exists(path):
            os.remove(path)


def test_page_mode_csv_logs_quaternion_when_present():
    path = os.path.join(
        os.environ.get("TMPDIR", "/tmp"), "printhead_page_profile_test_quat.csv")
    try:
        prof = PassProfiler(1.0, live=False, csv_path=path, mode="page")
        prof.start()
        quat = np.array([0.12345, -0.6789, 0.0001, 0.98765])
        prof.record_page_sample(u_mm=1.0, v_mm=2.0, speed_mm_s=5.0, quat=quat)
        prof.finish()
        with open(path) as fh:
            lines = fh.read().strip().splitlines()
        header = lines[0].split(",")
        row = lines[1].split(",")
        assert header[-4:] == ["qx", "qy", "qz", "qw"]
        assert row[-4:] == ["0.1235", "-0.6789", "0.0001", "0.9877"], row
    finally:
        if os.path.exists(path):
            os.remove(path)


def test_page_mode_csv_blank_quaternion_when_absent():
    # quat=None (the default) must not raise, and must leave the four fields
    # blank -- NOT "0,0,0,0", which would look like a real degenerate
    # quaternion and could mislead an offline analysis.
    path = os.path.join(
        os.environ.get("TMPDIR", "/tmp"), "printhead_page_profile_test_noquat.csv")
    try:
        prof = PassProfiler(1.0, live=False, csv_path=path, mode="page")
        prof.start()
        prof.record_page_sample(u_mm=1.0, v_mm=2.0, speed_mm_s=5.0)
        prof.finish()
        with open(path) as fh:
            lines = fh.read().strip().splitlines()
        # Splitting on ',' with trailing empty fields keeps them visible.
        row = lines[1].split(",")
        assert row[-4:] == ["", "", "", ""], row
    finally:
        if os.path.exists(path):
            os.remove(path)


def test_empty_page_profiler_is_safe():
    prof = PassProfiler(1.0, live=False, mode="page")
    prof.start()
    out = io.StringIO()
    with redirect_stdout(out):
        prof.finish()
    assert "no columns recorded" in out.getvalue()


# ---- CLI wiring -----------------------------------------------------------
def test_cli_ble_benchmark_needs_no_text():
    args = cli.parse_args(["--ble-benchmark"])
    assert args.ble_benchmark and args.text is None


def test_cli_profile_flags():
    # --mode line: this test is about --profile/--profile-csv, not mode
    # selection, and passes no --page-calibration.
    args = cli.parse_args(["Hi", "--dry-run", "--mode", "line",
                           "--profile", "--profile-csv", "x.csv"])
    assert args.profile and args.profile_csv == "x.csv"


if __name__ == "__main__":
    tests = [v for k, v in list(globals().items())
             if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"All {len(tests)} profiling tests passed.")
