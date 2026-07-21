"""
Text -> bitmap -> nozzle frames
===============================

Renders a text string into a black/white mask whose height is EXACTLY
``IMAGE_HEIGHT`` (164 px); the width depends on the text. Each image column then
becomes one 21-byte BLE frame.

Frame format
------------
Each frame is 21 bytes = 168 nozzle bits, LSB-first: bit ``p`` (byte ``p // 8``,
bit ``p % 8``) fires nozzle ``p``. A set bit = nozzle active = black pixel.
Only nozzles ``FIRST_NOZZLE..LAST_NOZZLE`` are connected, so image row ``y``
maps to nozzle ``p = FIRST_NOZZLE + y``.
"""

from __future__ import annotations

import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

from .config import RenderSettings
from .geometry import FIRST_NOZZLE, IMAGE_HEIGHT, LAST_NOZZLE, NUM_NOZZLES, ROW_BYTES


# ============================================================================
# Fonts
# ============================================================================
def load_font(font_path, size):
    """Return a scalable TrueType font, trying a few common locations."""
    candidates = []
    if font_path:
        candidates.append(font_path)
    candidates += [
        "DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/Library/Fonts/Arial.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "C:\\Windows\\Fonts\\arial.ttf",
        "arial.ttf",
        "LiberationSans-Regular.ttf",
    ]
    for candidate in candidates:
        try:
            return ImageFont.truetype(candidate, size)
        except Exception:
            continue
    print("WARN: no TrueType font found, falling back to PIL default.",
          file=sys.stderr)
    try:
        return ImageFont.load_default(size)   # Pillow >= 10.1
    except TypeError:
        return ImageFont.load_default()


# ============================================================================
# Text -> ink mask
# ============================================================================
def render_text(settings: RenderSettings) -> np.ndarray:
    """
    Render ``settings.text`` and return a boolean ndarray of shape
    ``(IMAGE_HEIGHT, W)`` where ``True`` == "fire nozzle" (black pixel).
    """
    font = load_font(settings.font, settings.render_size)

    # Measure the text.
    probe = ImageDraw.Draw(Image.new("L", (8, 8), 255))
    bbox = probe.textbbox((0, 0), settings.text, font=font)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    if tw <= 0 or th <= 0:
        raise ValueError("Text renders to an empty image.")

    # Draw black text on white.
    img = Image.new("L", (tw + 4, th + 4), 255)
    ImageDraw.Draw(img).text((-bbox[0] + 2, -bbox[1] + 2), settings.text,
                             font=font, fill=0)

    # Crop to the actual ink (invert so background becomes 0 for getbbox()).
    ink_box = ImageOps.invert(img).getbbox()
    if ink_box:
        img = img.crop(ink_box)

    # Scale so the text fills the height (minus optional margin), keep aspect.
    margin = settings.margin
    target_h = IMAGE_HEIGHT - 2 * margin
    if target_h < 1:
        target_h, margin = IMAGE_HEIGHT, 0
    w, h = img.size
    target_w = max(1, round(w * target_h / h))
    img = img.resize((target_w, target_h), Image.LANCZOS)

    # Pad vertically to exactly IMAGE_HEIGHT px.
    canvas = Image.new("L", (target_w, IMAGE_HEIGHT), 255)
    canvas.paste(img, (0, margin))

    # Threshold to a hard boolean mask (True = black = fire).
    ink = np.asarray(canvas) < settings.threshold
    if settings.invert:
        ink = ~ink
    if settings.flip_y:
        ink = ink[::-1, :]
    if settings.mirror_x:
        ink = ink[:, ::-1]
    return np.ascontiguousarray(ink)


# ============================================================================
# Ink mask -> nozzle frames
# ============================================================================
def frames_from_ink(ink: np.ndarray) -> list[bytes]:
    """
    Turn a ``(IMAGE_HEIGHT, W)`` boolean mask into a list of ``W`` 21-byte frames.

    Vectorised with ``numpy.packbits`` instead of a per-pixel Python loop: build a
    ``(W, NUM_NOZZLES)`` bit matrix, place the 164 image rows at nozzle offset
    ``FIRST_NOZZLE``, then pack LSB-first. This is bit-for-bit identical to setting
    ``frame[p // 8] |= 1 << (p % 8)`` for every fired nozzle ``p``.
    """
    h, w = ink.shape
    if h != IMAGE_HEIGHT:
        raise ValueError(f"Image height must be {IMAGE_HEIGHT}, got {h}.")

    bits = np.zeros((w, NUM_NOZZLES), dtype=np.uint8)
    # Column x, nozzle p = FIRST_NOZZLE + y  <->  bits[x, p] = ink[y, x]
    bits[:, FIRST_NOZZLE:LAST_NOZZLE + 1] = ink.T
    packed = np.packbits(bits, axis=1, bitorder="little")   # (W, ROW_BYTES)
    assert packed.shape == (w, ROW_BYTES)
    return [bytes(row) for row in packed]


# ============================================================================
# Preview
# ============================================================================
def save_preview(ink: np.ndarray, path: str) -> None:
    """Save exactly what will be printed (fire = black) as a PNG."""
    Image.fromarray(np.where(ink, 0, 255).astype(np.uint8), mode="L").save(path)
