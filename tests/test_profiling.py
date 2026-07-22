"""
Timing-profiler tests (no hardware).

Run with:  python tests/test_profiling.py
"""

import io
import os
import sys
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead import cli                        # noqa: E402
from printhead.profiling import PassProfiler     # noqa: E402


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
        assert lines[0] == "t_s,column,advance_mm,write_latency_ms,speed_mm_s"
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


# ---- CLI wiring -----------------------------------------------------------
def test_cli_ble_benchmark_needs_no_text():
    args = cli.parse_args(["--ble-benchmark"])
    assert args.ble_benchmark and args.text is None


def test_cli_profile_flags():
    args = cli.parse_args(["Hi", "--dry-run", "--profile", "--profile-csv", "x.csv"])
    assert args.profile and args.profile_csv == "x.csv"


if __name__ == "__main__":
    tests = [v for k, v in list(globals().items())
             if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print(f"OK: {t.__name__}")
    print(f"All {len(tests)} profiling tests passed.")
