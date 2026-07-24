"""
Send recorder / print reconstruction
====================================

Records every nozzle frame actually written over BLE during a position pass,
together with the head position at that moment, and reconstructs an image of
what physically ends up on paper.

This is the key difference between the clean nozzle-test and a real print: the
firmware prints the *latest* frame it received until the next one arrives, so a
frame is deposited across the physical span from where it was sent until the next
frame is sent. If the head moves faster than columns can be sent (or columns are
gap-filled in a burst at one position), several columns collapse onto the same
spot -> horizontal detail is lost. The reconstruction makes that visible: it maps
each sent frame to the head position it was sent at, exactly as the printhead
sees it, and stacks it against the intended image for comparison.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np
from PIL import Image, ImageDraw

from .geometry import IMAGE_HEIGHT
from .rendering import load_font


def _decode(frame: bytes) -> np.ndarray:
    """Decode a nozzle frame back to a (IMAGE_HEIGHT,) boolean column."""
    col = np.zeros(IMAGE_HEIGHT, dtype=bool)
    for j in range(IMAGE_HEIGHT):
        if frame[j >> 3] >> (j & 7) & 1:
            col[j] = True
    return col


class SendRecorder:
    """Collects (advance_mm, frame) for every write and renders the result."""

    def __init__(self, mm_per_column: float):
        self.mm_per_column = mm_per_column
        self.events: List[Tuple[float, bytes]] = []

    def record(self, advance_mm: float, frame: bytes) -> None:
        self.events.append((float(advance_mm), bytes(frame)))

    # ------------------------------------------------------------- reconstruct
    def reconstruct(self, min_width: int = 0) -> np.ndarray:
        """Position-mapped image of what was actually deposited: each sent frame
        painted from the head position it was sent at until the next frame's
        position ("latest frame wins"). Bursts sent at one position collapse."""
        if not self.events:
            return np.zeros((IMAGE_HEIGHT, max(1, min_width)), dtype=bool)
        mmpc = self.mm_per_column
        xs = [int(round(a / mmpc)) for a, _ in self.events]
        off = -min(0, min(xs))                       # shift so the first x >= 0
        xs = [x + off for x in xs]
        width = max(max(xs) + 1, min_width)

        recon = np.zeros((IMAGE_HEIGHT, width), dtype=bool)
        for k, (_, frame) in enumerate(self.events):
            xa = xs[k]
            xb = xs[k + 1] if k + 1 < len(xs) else xa + 1
            if xb <= xa:                             # superseded within a burst
                continue
            xa = max(0, min(width, xa))
            xb = max(0, min(width, xb))
            if xb > xa:
                recon[:, xa:xb] = _decode(frame)[:, None]
        return recon

    # ------------------------------------------------------------------ render
    def render(self, path: str, intended_ink: Optional[np.ndarray] = None) -> bool:
        """Write a PNG comparing the intended image to the reconstruction.
        Returns False if nothing was recorded."""
        if not self.events:
            return False
        intended_w = intended_ink.shape[1] if intended_ink is not None else 0
        recon = self.reconstruct(min_width=intended_w)
        width = recon.shape[1]

        panels = []
        if intended_ink is not None:
            intended = np.zeros((IMAGE_HEIGHT, width), dtype=bool)
            w2 = min(width, intended_ink.shape[1])
            intended[:, :w2] = intended_ink[:, :w2].astype(bool)
            panels.append(("INTENDED (preview)", intended))
        panels.append((f"SENT over BLE @ head position  "
                       f"({len(self.events)} writes, {self.mm_per_column:.3f} mm/col)",
                       recon))
        _save_panels(panels, path, width)
        return True


def _save_panels(panels, path: str, width: int) -> None:
    label_h = 18
    gap = 12
    total_h = sum(label_h + p.shape[0] + gap for _, p in panels)
    canvas = Image.new("L", (width, total_h), 255)
    draw = ImageDraw.Draw(canvas)
    font = load_font(None, 13)

    y = 0
    for label, mask in panels:
        draw.text((3, y + 2), label, font=font, fill=0)
        y += label_h
        img = Image.fromarray(np.where(mask, 0, 255).astype(np.uint8), mode="L")
        canvas.paste(img, (0, y))
        y += mask.shape[0] + gap
    canvas.save(path)
