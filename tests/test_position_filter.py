"""
Position low-pass filter tests (no hardware).

Run with:  python tests/test_position_filter.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from printhead.tracking import PositionFilter        # noqa: E402


def test_off_is_passthrough():
    f = PositionFilter(0.0)
    for i in range(5):
        p = np.array([i, 2 * i, -i], dtype=float)
        assert np.array_equal(f.update(p, i * 0.005), p)


def test_reduces_noise_variance():
    rng = np.random.default_rng(0)
    # a slow ramp along Y with heavy noise on top
    n = 400
    dt = 0.005                                   # 200 Hz
    true_y = np.linspace(0.0, 20.0, n)
    noisy = true_y + rng.normal(0.0, 1.0, n)     # sigma = 1 mm noise

    f = PositionFilter(0.03)                      # 30 ms time constant
    out = []
    for i in range(n):
        p = np.array([0.0, noisy[i], 0.0])
        out.append(f.update(p, i * dt)[1])
    out = np.array(out)

    # Residual to the underlying ramp is much smaller after filtering.
    raw_err = np.std(noisy - true_y)
    filt_err = np.std(out - true_y)
    assert filt_err < raw_err * 0.5, (filt_err, raw_err)

    # Sample-to-sample jitter (what makes the column jump) is strongly reduced.
    raw_jit = np.std(np.diff(noisy))
    filt_jit = np.std(np.diff(out))
    assert filt_jit < raw_jit * 0.4, (filt_jit, raw_jit)


def test_larger_tau_smooths_more():
    rng = np.random.default_rng(1)
    noisy = rng.normal(0.0, 1.0, 300)
    def jitter(tau):
        f = PositionFilter(tau)
        out = [f.update(np.array([0.0, v, 0.0]), i * 0.005)[1]
               for i, v in enumerate(noisy)]
        return np.std(np.diff(out))
    assert jitter(0.05) < jitter(0.01) < jitter(0.0)


def test_reset_forgets_state():
    f = PositionFilter(0.05)
    for i in range(10):
        f.update(np.array([0.0, 100.0, 0.0]), i * 0.005)
    f.reset()
    first = f.update(np.array([0.0, 5.0, 0.0]), 1.0)
    assert first[1] == 5.0            # first sample after reset passes through


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print(f"OK: {name}")
    print("All position-filter tests passed.")
