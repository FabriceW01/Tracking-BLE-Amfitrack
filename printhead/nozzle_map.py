"""
Nozzle block remapping
=======================

If the physical nozzles are wired in repeating blocks whose firing order does
not match their physical (vertical) order, this reorders the ink rows before
framing so the printed image still comes out visually correct.
"""

from __future__ import annotations

from typing import List

import numpy as np


def parse_order(spec: str, block_size: int) -> List[int]:
    """
    Parse a comma list like ``"2,3,4,1,5"`` into a validated 0-indexed
    permutation of ``range(block_size)``.
    """
    try:
        values = [int(x) for x in spec.split(",")]
    except ValueError:
        raise ValueError(
            f"--nozzle-order must be a comma-separated list of integers, got {spec!r}")
    if len(values) != block_size:
        raise ValueError(
            f"--nozzle-order needs exactly {block_size} values (one per block "
            f"position), got {len(values)}")
    zero_based = [v - 1 for v in values]
    if sorted(zero_based) != list(range(block_size)):
        raise ValueError(
            f"--nozzle-order must be a permutation of 1..{block_size}, got {spec!r}")
    return zero_based


def remap_rows(ink: np.ndarray, block_size: int, order: List[int]) -> np.ndarray:
    """
    Reorder ``ink`` rows in repeating blocks of ``block_size``:
    ``new[block_start + i] = old[block_start + order[i]]``.

    A trailing partial block (e.g. 164 rows not divisible by ``block_size``) is
    left unchanged, since the permutation does not fully apply to it.
    """
    h = ink.shape[0]
    out_idx = np.arange(h)
    for start in range(0, h, block_size):
        if h - start < block_size:
            break                              # trailing partial block: leave as-is
        for i, src in enumerate(order):
            out_idx[start + i] = start + src
    return ink[out_idx, :]
