"""
Send recorder / print reconstruction
====================================

Records every nozzle frame actually written over BLE during a position pass,
together with the head position at that moment, and reconstructs an image of
what physically ends up on paper.

This models what the firmware does with the columns it is handed: it queues them
and prints each one exactly once, in order, for a bounded number of fires. So a
frame occupies one column slot starting at the position it was sent from -- it is
neither smeared across the gap to the next frame nor overwritten by a column sent
right behind it.

What the reconstruction therefore exposes is what the *client* got wrong:
  * columns the client never sent (head fed faster than columns were emitted)
    show up as gaps;
  * columns sent from the same spot in a gap-fill burst are laid out side by side,
    so a burst appears shifted relative to where the head actually was;
  * a blank consumes a slot and deposits nothing.

It is stacked against the intended image for comparison.
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
        """Position-mapped image of what was actually deposited.

        The firmware queues the columns it receives and prints each exactly once,
        so every sent frame gets its own slot starting at the position it was sent
        from. Frames sent from the same position (a gap-fill burst) are laid out
        consecutively rather than overwriting each other."""
        if not self.events:
            return np.zeros((IMAGE_HEIGHT, max(1, min_width)), dtype=bool)
        mmpc = self.mm_per_column
        xs = [int(round(a / mmpc)) for a, _ in self.events]
        off = -min(0, min(xs))                       # shift so the first x >= 0
        xs = [x + off for x in xs]

        # Lay the queue out: a slot is never reused, so a burst spills to the right.
        slots = []
        next_x = 0
        for x in xs:
            x = max(x, next_x)
            slots.append(x)
            next_x = x + 1
        width = max(max(slots) + 1, min_width)

        recon = np.zeros((IMAGE_HEIGHT, width), dtype=bool)
        for (_, frame), x in zip(self.events, slots):
            if x < width:
                recon[:, x] = _decode(frame)         # a blank simply deposits nothing
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


_SENSOR_PATH_RGB = (30, 100, 220)     # blue: raw sensor centre
_NOZZLE_PATH_RGB = (230, 90, 20)      # orange: nozzle-bar centre
_PATH_START_RGB = (30, 160, 60)       # green dot: pass start
_PATH_END_RGB = (40, 40, 40)          # dark dot: pass end


def render_coverage(printed: np.ndarray, ink: np.ndarray, path: str,
                    sensor_path: Optional[List[Tuple[int, int]]] = None,
                    nozzle_path: Optional[List[Tuple[int, int]]] = None) -> bool:
    """
    Write a PNG comparing the intended page-mode image to what
    ``CoverageEngine`` actually covered. Returns False if nothing was
    printed.

    Unlike :meth:`SendRecorder.reconstruct` (line mode), there is nothing to
    reconstruct here: ``printed`` already IS a true-position record of what
    got inked, built live by ``CoverageEngine.step()`` sample by sample,
    rather than modelled after the fact from a send log against an assumed
    queue-slot layout. A third MISSED panel (ink wanted but never printed)
    is included since that is the whole open question after a freehand pass
    -- did the cart cover everything, and if not, where.

    ``sensor_path``/``nozzle_path``, if given, are lists of ``(row, col)``
    pixel coordinates -- one point per live sample, same convention
    ``CoverageEngine.step()`` uses internally (``row = round(v_mm /
    NOZZLE_PITCH_MM)``, ``col = round(u_mm / mm_per_column)``) -- for the
    RAW SENSOR position and the NOZZLE-BAR-CENTRE position respectively
    (see ``controller._print_freehand_pass``, which records both once per
    sample precisely so this can draw them). When either is given, a 4th
    "PATH" panel is added, letting an operator trace where the cart
    actually went against what got covered -- e.g. spotting that a MISSED
    patch was simply never driven over, versus driven over too fast for
    ``--dose-hold-s`` to complete. Points outside the page are drawn
    anyway (PIL clips automatically) rather than dropped, since a path
    leaving/re-entering the page is itself useful information. Omitting
    both keeps this call byte-for-byte identical to the pre-path-tracking
    behaviour (3 grayscale panels, unchanged), for any caller that doesn't
    have the trajectory available.
    """
    printed = np.asarray(printed, dtype=bool)
    ink = np.asarray(ink, dtype=bool)
    if not printed.any():
        return False
    missed = ink & ~printed
    panels = [
        ("INTENDED", ink),
        (f"COVERED ({int(printed.sum())}/{int(ink.sum())} ink pixels)", printed),
        (f"MISSED ({int(missed.sum())} ink pixels)", missed),
    ]
    extra_rgb_panels = []
    if sensor_path or nozzle_path:
        n_pts = max(len(sensor_path or ()), len(nozzle_path or ()))
        # Kept short deliberately: this label sits on a canvas as narrow as
        # the printed pattern itself (see test_render_coverage_path_panel_
        # label_still_fits_a_narrow_image), unlike a fixed-width UI label.
        label = f"PATH ({n_pts} pts) blue=sensor orange=nozzles"
        extra_rgb_panels.append(
            (label, _render_path_panel(ink.shape, sensor_path, nozzle_path)))
    _save_panels(panels, path, ink.shape[1], extra_rgb_panels=extra_rgb_panels)
    return True


def _render_path_panel(shape: Tuple[int, int],
                       sensor_path: Optional[List[Tuple[int, int]]],
                       nozzle_path: Optional[List[Tuple[int, int]]]) -> Image.Image:
    """A white RGB panel of ``shape`` (height, width) with each given path
    drawn as a thin polyline plus a start/end marker dot, so direction of
    travel is visible (a plain line alone doesn't show which end is which).
    """
    height, width = shape
    arr = np.full((height, width, 3), 255, dtype=np.uint8)
    img = Image.fromarray(arr, mode="RGB")
    draw = ImageDraw.Draw(img)

    def _draw(pts: Optional[List[Tuple[int, int]]], colour) -> None:
        if not pts:
            return
        # (row, col) -> PIL's (x, y) = (col, row).
        xy = [(c, r) for r, c in pts]
        if len(xy) >= 2:
            draw.line(xy, fill=colour, width=1)
        _dot(draw, xy[0], _PATH_START_RGB)
        _dot(draw, xy[-1], _PATH_END_RGB)

    _draw(sensor_path, _SENSOR_PATH_RGB)
    _draw(nozzle_path, _NOZZLE_PATH_RGB)
    return img


def _dot(draw: ImageDraw.ImageDraw, xy: Tuple[int, int], colour, r: int = 3) -> None:
    x, y = xy
    draw.ellipse([x - r, y - r, x + r, y + r], fill=colour)


def _save_panels(panels, path: str, width: int, extra_rgb_panels=None) -> None:
    """``panels`` are ``(label, boolean_mask)`` pairs rendered in grayscale
    (black-on-white), same as before path overlays existed. ``extra_rgb_panels``
    are ``(label, PIL.Image in RGB mode)`` pairs appended below them, for
    panels (like the path overlay) that need colour -- the whole canvas is
    built as RGB either way so a grayscale panel (via .convert) and a colour
    one can share one image; a pure grayscale panel still LOOKS identical to
    the pre-colour version, R==G==B."""
    label_h = 18
    gap = 12
    extra_rgb_panels = extra_rgb_panels or []
    total_h = (sum(label_h + p.shape[0] + gap for _, p in panels)
              + sum(label_h + img.height + gap for _, img in extra_rgb_panels))
    canvas = Image.new("RGB", (width, total_h), (255, 255, 255))
    draw = ImageDraw.Draw(canvas)
    font = load_font(None, 13)

    y = 0
    for label, mask in panels:
        draw.text((3, y + 2), label, font=font, fill=(0, 0, 0))
        y += label_h
        img = Image.fromarray(np.where(mask, 0, 255).astype(np.uint8), mode="L")
        canvas.paste(img.convert("RGB"), (0, y))
        y += mask.shape[0] + gap
    for label, img in extra_rgb_panels:
        draw.text((3, y + 2), label, font=font, fill=(0, 0, 0))
        y += label_h
        canvas.paste(img, (0, y))
        y += img.height + gap
    canvas.save(path)
