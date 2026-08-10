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
_PATH_START_RGB = (30, 160, 60)       # green dot: pass start (no sample_times)
_PATH_END_RGB = (40, 40, 40)          # dark dot: pass end
_MARKER_TEXT_RGB = (20, 20, 20)

DEFAULT_RECORD_SCALE = 3              # upscale factor for the whole PNG
DEFAULT_MARKER_INTERVAL_S = 2.0       # seconds between numbered path markers


def render_coverage(printed: np.ndarray, ink: np.ndarray, path: str,
                    sensor_path: Optional[List[Tuple[int, int]]] = None,
                    nozzle_path: Optional[List[Tuple[int, int]]] = None,
                    sample_times: Optional[List[float]] = None,
                    scale: int = DEFAULT_RECORD_SCALE,
                    marker_interval_s: float = DEFAULT_MARKER_INTERVAL_S) -> bool:
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
    both keeps this call's PATH panel omitted, for any caller that doesn't
    have the trajectory available.

    ``sample_times``, if given, is the elapsed pass time (seconds) at each
    of those same points -- same index/length as both path lists. Every
    ``marker_interval_s`` seconds (default 2.0), a bigger, numbered marker
    (1, 2, 3, ... starting at the very first point) is drawn at the nearest
    actual sample on BOTH paths at once, so a MISSED patch or an unexpected
    detour can be pinned to roughly when it happened, not just where.
    Without ``sample_times`` the path still draws, just with a plain
    (unlabelled) start/end dot instead, same as before markers existed.

    ``scale`` upscales the whole PNG (all four panels, nearest-neighbour for
    the boolean mask panels so each block still represents exactly one
    real nozzle-row/column cell, native drawing resolution for the PATH
    panel so its lines/dots/text stay crisp rather than blocky) -- makes the
    path easier to follow on a print job whose native column/row resolution
    would otherwise render it tiny. ``scale=1`` reproduces the pre-scaling
    pixel dimensions exactly.
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
        if sample_times:
            label += f", numbered every {marker_interval_s:g}s"
        extra_rgb_panels.append(
            (label, _render_path_panel(ink.shape, sensor_path, nozzle_path,
                                       sample_times, scale, marker_interval_s)))
    _save_panels(panels, path, ink.shape[1], extra_rgb_panels=extra_rgb_panels,
                scale=scale)
    return True


def _marker_indices(sample_times: List[float],
                    interval_s: float) -> List[Tuple[int, int]]:
    """``(index, marker_number)`` pairs: marker 1 at index 0 (t=0), then one
    every ``interval_s`` seconds of elapsed pass time, each placed at
    whichever recorded sample is closest to that target time (poll samples
    essentially never land on an exact 2.000s boundary). ``sample_times``
    must be non-decreasing (true of elapsed wall-clock time from a single
    pass). Never returns two markers on the same index -- a pass shorter
    than ``interval_s`` gets only marker 1."""
    if not sample_times:
        return []
    times = np.asarray(sample_times, dtype=float)
    n_markers = int(times[-1] // interval_s) + 1
    out = []
    last_idx = -1
    for k in range(n_markers):
        target = k * interval_s
        idx = int(np.searchsorted(times, target))
        if idx > 0 and (idx >= len(times)
                        or (target - times[idx - 1]) <= (times[idx] - target)):
            idx -= 1
        idx = min(idx, len(times) - 1)
        if idx != last_idx:
            out.append((idx, k + 1))
            last_idx = idx
    return out


def _render_path_panel(shape: Tuple[int, int],
                       sensor_path: Optional[List[Tuple[int, int]]],
                       nozzle_path: Optional[List[Tuple[int, int]]],
                       sample_times: Optional[List[float]],
                       scale: int, marker_interval_s: float) -> Image.Image:
    """An RGB panel, ``scale`` times the size of ``shape`` (height, width),
    with each given path drawn as a polyline. With ``sample_times``, bigger
    numbered markers replace the plain start/end dots (see
    ``render_coverage``'s docstring); without, a plain start (green) / end
    (dark) dot marks direction of travel, same as before markers existed.
    """
    height, width = shape
    arr = np.full((height * scale, width * scale, 3), 255, dtype=np.uint8)
    img = Image.fromarray(arr, mode="RGB")
    draw = ImageDraw.Draw(img)
    font = load_font(None, 13)
    markers = _marker_indices(sample_times, marker_interval_s) if sample_times else []

    def _draw(pts: Optional[List[Tuple[int, int]]], colour) -> None:
        if not pts:
            return
        # (row, col) -> PIL's (x, y) = (col, row), scaled up to match the
        # mask panels' own nearest-neighbour upscale.
        xy = [(c * scale, r * scale) for r, c in pts]
        if len(xy) >= 2:
            draw.line(xy, fill=colour, width=max(1, scale // 2))
        if markers:
            for idx, number in markers:
                if idx < len(xy):
                    _marker(draw, xy[idx], colour, str(number), font)
        else:
            _dot(draw, xy[0], _PATH_START_RGB, r=3 * scale)
            _dot(draw, xy[-1], _PATH_END_RGB, r=3 * scale)

    _draw(sensor_path, _SENSOR_PATH_RGB)
    _draw(nozzle_path, _NOZZLE_PATH_RGB)
    return img


def _dot(draw: ImageDraw.ImageDraw, xy: Tuple[int, int], colour, r: int = 3) -> None:
    x, y = xy
    draw.ellipse([x - r, y - r, x + r, y + r], fill=colour)


def _marker(draw: ImageDraw.ImageDraw, xy: Tuple[int, int], colour,
           text: str, font, r: int = 7) -> None:
    """A bigger filled dot in the path's own colour (so it's obvious which
    path a marker belongs to where paths cross) with its number just above
    and to the right, in a neutral dark colour for legibility against
    either path's colour."""
    x, y = xy
    draw.ellipse([x - r, y - r, x + r, y + r], fill=colour, outline=(255, 255, 255))
    draw.text((x + r + 2, y - r - 2), text, font=font, fill=_MARKER_TEXT_RGB)


def _save_panels(panels, path: str, width: int, extra_rgb_panels=None,
                 scale: int = 1) -> None:
    """``panels`` are ``(label, boolean_mask)`` pairs rendered in grayscale
    (black-on-white), upscaled ``scale``x with nearest-neighbour (so each
    block still represents exactly one real nozzle-row/column cell -- no
    blur/anti-aliasing implying false sub-cell precision). ``extra_rgb_panels``
    are ``(label, PIL.Image in RGB mode)`` pairs appended below them, for
    panels (like the path overlay) that need colour and are already built at
    the target (scaled) resolution by the caller -- the whole canvas is RGB
    either way so a grayscale panel (via .convert) and a colour one can
    share one image; a pure grayscale panel still LOOKS identical to the
    pre-colour version, R==G==B. ``scale=1`` (the default, used by
    SendRecorder.render's line-mode call) reproduces the pre-scaling pixel
    dimensions exactly."""
    label_h = 18
    gap = 12
    width *= scale
    extra_rgb_panels = extra_rgb_panels or []
    total_h = (sum(label_h + p.shape[0] * scale + gap for _, p in panels)
              + sum(label_h + img.height + gap for _, img in extra_rgb_panels))
    canvas = Image.new("RGB", (width, total_h), (255, 255, 255))
    draw = ImageDraw.Draw(canvas)
    font = load_font(None, 13)

    y = 0
    for label, mask in panels:
        draw.text((3, y + 2), label, font=font, fill=(0, 0, 0))
        y += label_h
        img = Image.fromarray(np.where(mask, 0, 255).astype(np.uint8), mode="L")
        if scale != 1:
            img = img.resize((mask.shape[1] * scale, mask.shape[0] * scale),
                             Image.NEAREST)
        canvas.paste(img.convert("RGB"), (0, y))
        y += mask.shape[0] * scale + gap
    for label, img in extra_rgb_panels:
        draw.text((3, y + 2), label, font=font, fill=(0, 0, 0))
        y += label_h
        canvas.paste(img, (0, y))
        y += img.height + gap
    canvas.save(path)
