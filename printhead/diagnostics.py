"""
Debug / diagnostics
===================

Standalone bring-up checks, each wired to its own CLI flag. They reuse the
normal building blocks (tracker, framing, BLE client) but run independently of
a print pass: connect, report/act, then exit. Every check degrades gracefully
with a friendly message when the hardware or a vendor library is missing.
"""

from __future__ import annotations

import asyncio
import json
import math
import time
from typing import Optional

import numpy as np

from .ble_client import PrintheadBLE
from .calibration import MIN_SAMPLE_COUNT, MIN_TRACE_LENGTH_MM, PageCalibration
from .config import BleSettings, NozzleMapSettings, TrackingSettings
from .geometry import (
    BLANK_FRAME,
    IMAGE_HEIGHT,
    NOZZLE_MODE_LINE,
    SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM,
    SENSOR_TO_NOZZLE_COL_MM,
)
from .nozzle_map import remap_rows
from .rendering import frames_from_ink
from .tracking import _AXIS_INDEX, PageMapper, PositionFilter, make_tracker


# ============================================================================
# --pos : live Amfitrack position readout
# ============================================================================
async def monitor_position(tracking: TrackingSettings, simulate: bool,
                           hz: float = 15.0, ndjson: bool = False,
                           page_calibration_path: Optional[str] = None,
                           sensor_offset_row_mm: float = SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM,
                           sensor_offset_col_mm: float = SENSOR_TO_NOZZLE_COL_MM,
                           boresight_deg: float = 0.0,
                           simple_boresight: Optional[np.ndarray] = None) -> None:
    """Continuously print the sensor position (x/y/z), the travel-axis value and
    the resulting column, until Ctrl+C. Doubles as an axis / mm-per-column aid.

    ``ndjson=True`` prints one newline-terminated JSON object per sample instead
    of the live single-line readout, so tools (the web UI) can parse the stream.

    ``page_calibration_path``, if given, loads a ``PageCalibration`` (see
    ``calibration.py``) and additionally reports the live page-plane
    ``(page_u, page_v, page_z)`` for that calibration -- lets a page-mode
    calibration be sanity-checked (known hand motion -> plausible u/v) before
    anything is printed with it. A bad/missing path aborts the same way a
    failed tracker connection does, since the caller asked for it explicitly.

    ``simple_boresight``, with ``tracking.page_frame == "simple"``, pins the
    yaw reference instead of auto-capturing whichever pose the first live
    sample happens to be in (see ``PageCalibration.simple_frame``). This is
    what makes this diagnostic a two-step capture-and-verify workflow: run it
    once with no ``simple_boresight`` to read off the raw ``quat=[...]``
    while the cart is held genuinely flat, then run it again (or print) with
    that exact quaternion pinned here to confirm ``yaw_deg``/``roll_deg``/
    ``pitch_deg`` now read ~0 for that same pose before trusting it.

    ``sensor_offset_row_mm``/``sensor_offset_col_mm``/``boresight_deg`` are
    forwarded into the same :class:`~printhead.tracking.PageMapper` a real
    pass builds (see ``PrintController._print_freehand_pass``), so this
    diagnostic reports the exact same ``(page_u, page_v)`` -- and, when a
    boresight has been captured, the exact same live yaw -- a real pass
    would use. Reporting the live yaw (``yaw_deg`` below) is exactly what
    lets a boresight be verified before printing with it: held in the
    reference pose (nozzle bar along the traced row edge), ``yaw_deg``
    should read close to 0. ``roll_deg``/``pitch_deg`` (see
    ``rotation.cart_rotation_angles``) are reported alongside it for the
    same live-monitoring purpose, but are diagnostic only -- unlike yaw,
    neither feeds any position correction (see that function's docstring)."""
    tracker = make_tracker(tracking, simulate)
    try:
        tracker.open()
    except Exception as exc:
        if ndjson:
            print(json.dumps({"event": "error", "message": str(exc)}), flush=True)
        else:
            print(f"Cannot open Amfitrack tracker: {exc}")
        return

    page_mapper = None
    if tracking.page_frame == "simple":
        # Calibration-free frame (see PageCalibration.simple_frame): page
        # axes = tracker x/y, yaw about tracker z. Unlike a print pass, the
        # origin is NOT zeroed here -- this diagnostic is for watching raw
        # tracker-frame u/v/yaw, and re-zeroing to wherever --pos happened to
        # start would only obscure that. page_u/page_v therefore read as
        # absolute tracker x/y (plus the sensor->nozzle offset).
        page_mapper = PageMapper(
            PageCalibration.simple_frame(boresight_quat=simple_boresight),
            sensor_offset_row_mm=sensor_offset_row_mm,
            sensor_offset_col_mm=sensor_offset_col_mm,
            boresight_offset_rad=math.radians(boresight_deg))
    elif page_calibration_path is not None:
        try:
            page_mapper = PageMapper(PageCalibration.load(page_calibration_path),
                                     sensor_offset_row_mm=sensor_offset_row_mm,
                                     sensor_offset_col_mm=sensor_offset_col_mm,
                                     boresight_offset_rad=math.radians(boresight_deg))
        except Exception as exc:
            if ndjson:
                print(json.dumps({"event": "error",
                                  "message": f"Cannot load page calibration: {exc}"}),
                      flush=True)
            else:
                print(f"Cannot load page calibration '{page_calibration_path}': {exc}")
            tracker.close()
            return

    axis = _AXIS_INDEX[tracking.advance_axis]
    origin = None
    pos_filter = PositionFilter(tracking.smooth_ms / 1000.0)
    if ndjson:
        print(json.dumps({"event": "connected", "axis": tracking.advance_axis,
                          "mm_per_column": tracking.mm_per_column}), flush=True)
    else:
        print(f"Live Amfitrack position (axis '{tracking.advance_axis}', "
              f"{tracking.mm_per_column:.3f} mm/col). Ctrl+C to stop.")
    try:
        while True:
            pos, quat = tracker.read_pose()
            if pos is not None:
                pos = pos_filter.update(pos, time.monotonic())
                if origin is None:
                    origin = pos.copy()
                advance = tracking.axis_sign * float(pos[axis] - origin[axis])
                col = int(round(advance / tracking.mm_per_column))
                # quat (qx,qy,qz,qw) -- see AmfitrackTracker._extract_pose -- is
                # included only when the connected hardware/SDK actually reports it,
                # so this line looks exactly as before on setups that don't.
                # page_mapper.project() applies the same rotation correction
                # (or lack of it -- see PageMapper.project's docstring) as a
                # real freehand pass, and caches this sample's yaw on
                # page_mapper.last_yaw_rad as a side effect (read below) --
                # see controller._print_freehand_pass for the identical
                # "compute once, reuse" pattern.
                # Simple frame with no pinned --simple-boresight: adopt the
                # first orientation sample as the yaw reference, mirroring
                # the auto-capture a real pass falls back to -- so yaw_deg
                # reads 0 for the pose the cart is in now and then shows the
                # turn from it. Guarded on `is None`, so a boresight already
                # pinned via simple_boresight above is never overwritten --
                # that pinned value is the whole point of the second
                # verification run (see this function's docstring).
                if (page_mapper is not None and quat is not None
                        and tracking.page_frame == "simple"
                        and page_mapper.calibration.boresight_quat is None):
                    page_mapper.capture_boresight(quat)
                page_uvz = page_mapper.project(pos, quat) if page_mapper is not None else None
                yaw_deg = (math.degrees(page_mapper.last_yaw_rad)
                          if page_mapper is not None else None)
                # Diagnostic-only tilt readout (see rotation.cart_rotation_angles /
                # tracking.PageMapper.project) -- reported alongside yaw_deg but,
                # like it, never fed back into any correction; last_roll_rad/
                # last_pitch_rad are valid floats (0.0 default) whenever a
                # page_mapper is active at all, same as last_yaw_rad.
                roll_deg = (math.degrees(page_mapper.last_roll_rad)
                           if page_mapper is not None else None)
                pitch_deg = (math.degrees(page_mapper.last_pitch_rad)
                            if page_mapper is not None else None)
                if ndjson:
                    event = {
                        "event": "position",
                        "x": round(float(pos[0]), 3), "y": round(float(pos[1]), 3),
                        "z": round(float(pos[2]), 3),
                        "advance": round(advance, 3), "col": col}
                    if quat is not None:
                        event.update(qx=round(float(quat[0]), 4), qy=round(float(quat[1]), 4),
                                     qz=round(float(quat[2]), 4), qw=round(float(quat[3]), 4))
                    if page_uvz is not None:
                        event.update(page_u=round(page_uvz[0], 3),
                                     page_v=round(page_uvz[1], 3),
                                     page_z=round(page_uvz[2], 3),
                                     yaw_deg=round(yaw_deg, 3),
                                     roll_deg=round(roll_deg, 3),
                                     pitch_deg=round(pitch_deg, 3))
                    print(json.dumps(event), flush=True)
                else:
                    line = (f"x={pos[0]:9.2f}  y={pos[1]:9.2f}  z={pos[2]:9.2f} mm  |  "
                           f"advance={advance:9.2f} mm  |  col={col:5d}")
                    if quat is not None:
                        line += (f"  |  quat=[{quat[0]:+.2f} {quat[1]:+.2f} "
                                f"{quat[2]:+.2f} {quat[3]:+.2f}]")
                    if page_uvz is not None:
                        line += (f"  |  page u={page_uvz[0]:8.2f}  v={page_uvz[1]:8.2f}  "
                                f"z={page_uvz[2]:6.2f} mm  yaw={yaw_deg:+6.2f} deg  "
                                f"roll={roll_deg:+6.2f} deg  pitch={pitch_deg:+6.2f} deg")
                    print(line, end="\r", flush=True)
            await asyncio.sleep(1.0 / hz)
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        if not ndjson:
            print()                   # leave the live line intact
        tracker.close()
        if ndjson:
            print(json.dumps({"event": "stopped"}), flush=True)
        else:
            print("Stopped position monitor.")


# ============================================================================
# --calibration-check : measure yaw drift while only TRANSLATING
# ============================================================================
# Verdict thresholds (degrees of yaw SPAN -- max minus min -- over the whole
# sweep). With the cart genuinely flat and only sliding, not rotating, yaw
# should barely move at all -- any span at all is either measurement noise,
# or tilt (roll/pitch, see rotation.cart_rotation_angles) leaking into yaw
# through a page normal that is not quite right. The operator's own current
# (already decent, see calibration.py's threshold comment: 0.63 deg normal
# tilt) calibration measures 2-3 deg of span on a normal A4-sized sweep and
# considers that acceptable day to day -- so this stays quiet up to
# CALIBRATION_CHECK_YAW_SPAN_FINE_DEG, treats up to
# CALIBRATION_CHECK_YAW_SPAN_WARN_DEG (roughly what the operator already
# tolerates) as borderline, and only calls anything BEYOND that a real
# problem.
CALIBRATION_CHECK_YAW_SPAN_FINE_DEG = 2.0
CALIBRATION_CHECK_YAW_SPAN_WARN_DEG = 4.0

# ...but a yaw span only means anything if the cart actually MOVED, and moved
# over enough samples for the span to be more than one noisy reading. Below
# either of these, the verdict is INCONCLUSIVE rather than OK.
#
# CORRECTION: the verdict first keyed on yaw span alone, so a run with ZERO
# samples -- tracker never delivered a pose, or Ctrl+C hit immediately --
# reported "OK: yaw span 0.00 deg ... consistent with a good calibration".
# A health check that declares the calibration healthy after measuring
# nothing at all is worse than no health check: it is a false all-clear on
# exactly the question the operator ran it to answer. The same held for a
# two-centimetre wiggle, whose near-zero yaw span proves nothing either.
# _calibration_check_summary already computed u_travel_mm/v_travel_mm and
# its own docstring called them "the headline 'did the operator actually
# slide it far enough to mean anything' sanity check" -- the verdict simply
# never consulted them. It does now.
#
# Deliberately the SAME numbers as calibration.MIN_TRACE_LENGTH_MM /
# MIN_SAMPLE_COUNT, imported rather than redeclared: "enough travel and
# enough samples for a straight-line fit over the page to mean something"
# is the same question in both places, already backed by the measurement
# table in calibration.py, and two independently drifting copies of that
# judgement would be worse than one.
CALIBRATION_CHECK_MIN_TRAVEL_MM = MIN_TRACE_LENGTH_MM
CALIBRATION_CHECK_MIN_SAMPLES = MIN_SAMPLE_COUNT


def _calibration_check_summary(u_samples, v_samples, yaw_deg_samples,
                               roll_deg_samples, pitch_deg_samples) -> dict:
    """
    Pure computation from a completed (or in-progress) ``calibration_check``
    run's collected per-sample page ``u``/``v`` (mm) and ``yaw``/``roll``/
    ``pitch`` (degrees) readings -- factored out of the async streaming loop
    below exactly so it is directly unit-testable against a scripted sample
    sequence, without needing a tracker or an event loop at all (mirrors
    ``ui.server.compute_calibration`` being factored out of its ``@app.post``
    handler for the same "testable as a plain function" reason).

    Returns a dict of:

      * ``sample_count`` -- how many samples went into this summary.
      * ``u_travel_mm``/``v_travel_mm`` -- how far the cart's page position
        actually spanned (max - min) along each page axis during the sweep.
        The headline "did the operator actually slide it far enough to mean
        anything" sanity check -- a large yaw span over a two-inch wiggle
        proves much less than the same span over a full A4 sheet.
      * ``yaw_min_deg``/``yaw_max_deg``/``yaw_span_deg`` -- ``yaw_span_deg``
        (max - min) is the HEADLINE number: with the cart provably flat and
        only translating, this should stay near 0.
      * ``roll_span_deg``/``pitch_span_deg`` -- same span idea for tilt, the
        quantity that leaks INTO yaw through an imperfect page normal (see
        ``rotation.cart_rotation_angles``'s and ``calibration.py``'s own
        docstrings) -- reported so a large yaw span can be cross-checked
        against whether tilt was actually present to leak from.
      * ``yaw_u_correlation``/``yaw_v_correlation`` -- Pearson correlation
        coefficient of yaw against page position along each axis, or
        ``None`` if there are fewer than 2 samples or either series has ~0
        variance (a correlation coefficient is undefined there, not 0).
        This is what separates ordinary sample-to-sample NOISE (yaw jitters
        around with no relationship to where the cart is) from SYSTEMATIC
        drift with position (yaw trends consistently as the cart moves one
        way) -- on real data from this rig, measured tilt correlated +0.69
        with v while the cart was provably flat, i.e. not noise.
      * ``verdict`` -- a human-readable one-paragraph verdict against the
        thresholds above, including which follow-up experiment distinguishes
        a bad calibration frame from tracker field distortion (see below).
    """
    u = np.asarray(u_samples, dtype=float)
    v = np.asarray(v_samples, dtype=float)
    yaw = np.asarray(yaw_deg_samples, dtype=float)
    roll = np.asarray(roll_deg_samples, dtype=float)
    pitch = np.asarray(pitch_deg_samples, dtype=float)

    def _span(a: np.ndarray) -> float:
        return float(a.max() - a.min()) if a.size else 0.0

    def _correlation(a: np.ndarray, b: np.ndarray) -> Optional[float]:
        if a.size < 2 or float(np.std(a)) < 1e-9 or float(np.std(b)) < 1e-9:
            return None
        return float(np.corrcoef(a, b)[0, 1])

    yaw_span_deg = _span(yaw)
    summary = {
        "sample_count": int(u.size),
        "u_travel_mm": _span(u),
        "v_travel_mm": _span(v),
        "yaw_min_deg": float(yaw.min()) if yaw.size else 0.0,
        "yaw_max_deg": float(yaw.max()) if yaw.size else 0.0,
        "yaw_span_deg": yaw_span_deg,
        "roll_span_deg": _span(roll),
        "pitch_span_deg": _span(pitch),
        "yaw_u_correlation": _correlation(yaw, u),
        "yaw_v_correlation": _correlation(yaw, v),
    }

    # Did the sweep measure enough to judge at all? Checked BEFORE the yaw
    # thresholds, because a sweep that never happened trivially satisfies
    # all of them (see CALIBRATION_CHECK_MIN_TRAVEL_MM for the false
    # all-clear this prevents). Travel is the bounding-box diagonal of the
    # (u, v) sweep -- a sweep that ran 60mm along u alone counts just as
    # much as one that ran diagonally, which matches how the operator
    # actually slides the cart.
    travel_mm = math.hypot(summary["u_travel_mm"], summary["v_travel_mm"])
    if (summary["sample_count"] < CALIBRATION_CHECK_MIN_SAMPLES
            or travel_mm < CALIBRATION_CHECK_MIN_TRAVEL_MM):
        summary["verdict"] = (
            f"INCONCLUSIVE: only {summary['sample_count']} sample(s) over "
            f"{travel_mm:.1f} mm of travel -- too little to judge (need at "
            f"least {CALIBRATION_CHECK_MIN_SAMPLES} samples and "
            f"{CALIBRATION_CHECK_MIN_TRAVEL_MM:.0f} mm). Slide the cart flat "
            f"across the page, ideally its full width, WITHOUT rotating it, "
            f"then stop with Ctrl+C. This says nothing about the calibration "
            f"either way -- it is not a pass.")
        return summary

    if yaw_span_deg <= CALIBRATION_CHECK_YAW_SPAN_FINE_DEG:
        verdict = (
            f"OK: yaw span {yaw_span_deg:.2f} deg is at or under the "
            f"~{CALIBRATION_CHECK_YAW_SPAN_FINE_DEG:.0f} deg 'fine' mark for "
            f"a flat, non-rotating sweep -- consistent with a good calibration.")
    elif yaw_span_deg <= CALIBRATION_CHECK_YAW_SPAN_WARN_DEG:
        verdict = (
            f"BORDERLINE: yaw span {yaw_span_deg:.2f} deg is above the "
            f"~{CALIBRATION_CHECK_YAW_SPAN_FINE_DEG:.0f} deg 'fine' mark, but "
            f"close to the 2-3 deg this rig's own calibration currently shows "
            f"and the operator already finds acceptable -- worth a look, not "
            f"necessarily broken.")
    else:
        verdict = (
            f"BAD: yaw span {yaw_span_deg:.2f} deg is well beyond what a "
            f"flat, non-rotating sweep should show. Likely either (a) a bad "
            f"calibration page-normal (retrace the edges -- see calibration."
            f"py's CalibrationQualityWarning for whether the trace itself was "
            f"short/noisy/sparse), or (b) tracker field distortion (a real "
            f"physical effect, independent of calibration). To tell them "
            f"apart: re-run this check at the SAME physical spot with a "
            f"freshly, carefully re-traced calibration -- if the drift "
            f"disappears, it was the calibration; if it persists even with a "
            f"known-good one, repeat the same sweep at a DIFFERENT position/"
            f"height over the tracker base station -- a drift pattern that "
            f"moves with absolute tracker position rather than with the "
            f"calibration is field distortion, not something re-tracing can "
            f"fix.")
    summary["verdict"] = verdict
    return summary


async def calibration_check(tracking: TrackingSettings, simulate: bool,
                            hz: float = 15.0, ndjson: bool = False,
                            page_calibration_path: Optional[str] = None,
                            sensor_offset_row_mm: float = SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM,
                            sensor_offset_col_mm: float = SENSOR_TO_NOZZLE_COL_MM,
                            boresight_deg: float = 0.0,
                            simple_boresight: Optional[np.ndarray] = None,
                            tracker=None) -> None:
    """
    Calibration health check: measures the operator's exact reported
    symptom -- yaw drifting while the cart only TRANSLATES.

    Streams live pose like ``--pos``/``monitor_position`` does (same
    ``ndjson`` NDJSON convention -- one ``{"event": "position", ...}``
    object per line, so this reuses the web UI's existing live position
    handling verbatim rather than needing a second one), while the operator
    slides the cart flat over the page WITHOUT rotating it. On Ctrl+C, it
    stops streaming and prints a summary of what was collected (see
    ``_calibration_check_summary``, which does the actual statistics and is
    unit-tested directly against a scripted sample sequence).

    Needs a real page frame to measure drift against -- ``page_calibration_path``
    or ``tracking.page_frame == "simple"`` -- unlike ``monitor_position``,
    which tolerates neither and just omits the page fields. There is nothing
    to check without one.

    ``tracker``, if given, is used instead of building one via
    ``tracking.make_tracker``/``simulate`` -- lets tests inject a scripted
    pose sequence (mirrors ``tests/test_freehand_pass.py``'s
    ``ScriptedTracker`` / ``tests/test_position_pass.py``'s pattern) with a
    real quaternion series, which ``SimulatedTracker`` itself never fakes
    (see its own docstring) and so cannot exercise the yaw-drift statistics
    this function exists to compute. Defaults to ``None``, i.e. the normal
    hardware/``--simulate`` path ``monitor_position`` also uses.

    The other arguments (``sensor_offset_row_mm``/``sensor_offset_col_mm``/
    ``boresight_deg``/``simple_boresight``) are forwarded into the
    :class:`~printhead.tracking.PageMapper` exactly like ``monitor_position``
    -- the page-frame construction below deliberately mirrors that
    function's rather than sharing a helper with it, so this addition
    cannot change ``monitor_position``'s own already-tested behaviour.
    """
    if tracker is None:
        tracker = make_tracker(tracking, simulate)
    try:
        tracker.open()
    except Exception as exc:
        if ndjson:
            print(json.dumps({"event": "error", "message": str(exc)}), flush=True)
        else:
            print(f"Cannot open Amfitrack tracker: {exc}")
        return

    if tracking.page_frame == "simple":
        page_mapper = PageMapper(
            PageCalibration.simple_frame(boresight_quat=simple_boresight),
            sensor_offset_row_mm=sensor_offset_row_mm,
            sensor_offset_col_mm=sensor_offset_col_mm,
            boresight_offset_rad=math.radians(boresight_deg))
    elif page_calibration_path is not None:
        try:
            page_mapper = PageMapper(PageCalibration.load(page_calibration_path),
                                     sensor_offset_row_mm=sensor_offset_row_mm,
                                     sensor_offset_col_mm=sensor_offset_col_mm,
                                     boresight_offset_rad=math.radians(boresight_deg))
        except Exception as exc:
            if ndjson:
                print(json.dumps({"event": "error",
                                  "message": f"Cannot load page calibration: {exc}"}),
                      flush=True)
            else:
                print(f"Cannot load page calibration '{page_calibration_path}': {exc}")
            tracker.close()
            return
    else:
        message = ("--calibration-check needs a page frame to measure drift "
                   "against: pass --page-calibration PATH or --page-frame simple")
        if ndjson:
            print(json.dumps({"event": "error", "message": message}), flush=True)
        else:
            print(message)
        tracker.close()
        return

    pos_filter = PositionFilter(tracking.smooth_ms / 1000.0)
    u_samples: list = []
    v_samples: list = []
    yaw_samples: list = []
    roll_samples: list = []
    pitch_samples: list = []

    if ndjson:
        print(json.dumps({"event": "connected"}), flush=True)
    else:
        print("Calibration health check: slide the cart FLAT over the page, "
             "WITHOUT rotating it. Ctrl+C to stop and print the summary.")
    try:
        while True:
            pos, quat = tracker.read_pose()
            if pos is not None:
                pos = pos_filter.update(pos, time.monotonic())
                # Simple frame with no pinned --simple-boresight: adopt the
                # first orientation sample as the yaw reference, exactly
                # mirroring monitor_position's identical block above --
                # without this, PageMapper.project() has no boresight_quat
                # at all and last_yaw_rad/last_roll_rad/last_pitch_rad stay
                # at their 0.0 default forever, making this diagnostic
                # report a perfect (and meaningless) yaw span of 0 no matter
                # how the cart actually moves.
                if (tracking.page_frame == "simple" and quat is not None
                        and page_mapper.calibration.boresight_quat is None):
                    page_mapper.capture_boresight(quat)
                u, v, z = page_mapper.project(pos, quat)
                yaw_deg = math.degrees(page_mapper.last_yaw_rad)
                roll_deg = math.degrees(page_mapper.last_roll_rad)
                pitch_deg = math.degrees(page_mapper.last_pitch_rad)
                u_samples.append(u)
                v_samples.append(v)
                yaw_samples.append(yaw_deg)
                roll_samples.append(roll_deg)
                pitch_samples.append(pitch_deg)
                if ndjson:
                    event = {
                        "event": "position",
                        "x": round(float(pos[0]), 3), "y": round(float(pos[1]), 3),
                        "z": round(float(pos[2]), 3),
                        "page_u": round(u, 3), "page_v": round(v, 3), "page_z": round(z, 3),
                        "yaw_deg": round(yaw_deg, 3), "roll_deg": round(roll_deg, 3),
                        "pitch_deg": round(pitch_deg, 3)}
                    print(json.dumps(event), flush=True)
                else:
                    print(f"page u={u:8.2f}  v={v:8.2f} mm  |  "
                         f"yaw={yaw_deg:+6.2f}  roll={roll_deg:+6.2f}  "
                         f"pitch={pitch_deg:+6.2f} deg", end="\r", flush=True)
            await asyncio.sleep(1.0 / hz)
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        if not ndjson:
            print()                   # leave the live line intact
        tracker.close()
        summary = _calibration_check_summary(
            u_samples, v_samples, yaw_samples, roll_samples, pitch_samples)
        if ndjson:
            print(json.dumps({"event": "calibration_check_summary", **summary}), flush=True)
        else:
            corr_u = summary["yaw_u_correlation"]
            corr_v = summary["yaw_v_correlation"]
            corr_line = (f"  yaw correlation: vs u = {corr_u:+.2f}  vs v = {corr_v:+.2f}"
                        if corr_u is not None and corr_v is not None
                        else "  yaw correlation: n/a (not enough motion/variation collected)")
            print("---- calibration health check summary ----")
            print(f"  samples: {summary['sample_count']}")
            print(f"  travelled: u={summary['u_travel_mm']:.1f}mm  "
                  f"v={summary['v_travel_mm']:.1f}mm")
            print(f"  yaw: min={summary['yaw_min_deg']:+.2f}  "
                  f"max={summary['yaw_max_deg']:+.2f}  "
                  f"span={summary['yaw_span_deg']:.2f} deg")
            print(f"  roll span: {summary['roll_span_deg']:.2f} deg   "
                  f"pitch span: {summary['pitch_span_deg']:.2f} deg")
            print(corr_line)
            print(f"  verdict: {summary['verdict']}")
        if ndjson:
            print(json.dumps({"event": "stopped"}), flush=True)
        else:
            print("Stopped calibration check.")


# ============================================================================
# --list-nodes : enumerate Amfitrack USB nodes
# ============================================================================
def list_nodes(tracking: TrackingSettings) -> None:
    """Connect to the dongle and list every node so the 'Sensor' match is visible."""
    try:
        import amfiprot
    except ImportError:
        print("amfiprot is not installed (pip install amfiprot amfiprot-amfitrack).")
        return

    s = tracking
    try:
        conn = amfiprot.USBConnection(s.vendor_id, s.product_id)
    except Exception:
        try:
            conn = amfiprot.USBConnection(s.vendor_id, s.product_id_source)
        except Exception as exc:
            print(f"Cannot open USB dongle "
                  f"(vendor 0x{s.vendor_id:04X}): {exc}")
            return

    try:
        nodes = conn.find_nodes()
        print(f"Found {len(nodes)} node(s):")
        for node in nodes:
            name = getattr(node, "name", "?")
            marker = "  <- sensor" if "Sensor" in str(name) else ""
            print(f"  name={name!r}  uuid={getattr(node, 'uuid', '?')}  "
                  f"tx_id={getattr(node, 'tx_id', '?')}{marker}")
        if not any("Sensor" in str(getattr(n, 'name', '')) for n in nodes):
            print("No node name contains 'Sensor' -> the tracker would find none.")
    finally:
        for method in ("stop", "close"):
            try:
                getattr(conn, method)()
            except Exception:
                pass


# ============================================================================
# --scan-ble : list nearby BLE devices
# ============================================================================
async def scan_ble(ble: BleSettings) -> None:
    """Scan and print BLE devices (address + name) to find the printhead."""
    try:
        from bleak import BleakScanner
    except ImportError:
        print("bleak is not installed (pip install bleak).")
        return

    print(f"Scanning BLE for {ble.scan_timeout:.0f}s ...")
    try:
        devices = await BleakScanner.discover(timeout=ble.scan_timeout)
    except Exception as exc:
        print(f"BLE scan failed: {exc}")
        return

    if not devices:
        print("No BLE devices found.")
        return
    for dev in devices:
        name = dev.name or "(no name)"
        marker = "  <- printhead" if dev.name == ble.device_name else ""
        print(f"  {dev.address}  {name}{marker}")


def _print_start_button_hint() -> None:
    """
    Firmware only drains its BLE receive FIFO into the nozzle output queue
    while ``process_running == 1`` -- and that flag is set *exclusively* by
    a physical button press in the firmware's ``mainloop()`` (see
    ``main.c``: ``button_poll_press_event()`` -> ``i2s_parallel_start()``).
    BLE writes from here always succeed and land in the FIFO regardless, so
    without the button this command reports success while physically
    nothing fires and 0 mA flows -- confirmed against a real serial log.
    There is no BLE-visible signal wired up to gate on here (see the
    module docstring / README for why), so a message that cannot be missed
    is the fix.
    """
    print("=" * 72)
    print("IMPORTANT: the printhead only fires while its print process is")
    print("running, and that is only ever started by a PHYSICAL PRESS of the")
    print("START button on the device itself -- this command cannot do that")
    print("for you. Press and hold the device's START button now, for the")
    print("duration of this test, or nothing will physically happen even")
    print("though every BLE write below reports success.")
    print("=" * 72)


# ============================================================================
# --nozzle-test : fire a diagnostic pattern on the cartridge
# ============================================================================
async def nozzle_test(ble: BleSettings, nozzle_map: Optional[NozzleMapSettings] = None,
                      on_seconds: float = 2.0, sweep_step: float = 0.02) -> None:
    """All nozzles on briefly, then a single nozzle swept down all 152 rows.

    If ``nozzle_map`` is given, it is applied first, so the sweep lets you
    visually confirm a block remap fixes the physical firing order."""
    all_on_ink = np.ones((IMAGE_HEIGHT, 1), dtype=bool)
    sweep_ink = np.eye(IMAGE_HEIGHT, dtype=bool)      # 152 single-nozzle frames
    if nozzle_map is not None and nozzle_map.block_size:
        all_on_ink = remap_rows(all_on_ink, nozzle_map.block_size, nozzle_map.order)
        sweep_ink = remap_rows(sweep_ink, nozzle_map.block_size, nozzle_map.order)
    all_on = frames_from_ink(all_on_ink)[0]
    sweep = frames_from_ink(sweep_ink)

    try:
        async with PrintheadBLE(ble) as client:
            # This tool bypasses _run_ble(), so nothing else pins the firmware to
            # line mode here. If it is still in page mode from an earlier --mode
            # page run, the "all on" write below would not fire 3 times like line
            # mode intends -- page mode queues each written column and fires it
            # exactly ONCE, so the 2.0s "all nozzles on" step would be a single
            # 300 us flash instead. required=False: this must still run against
            # older firmware without MODE_UUID, where line mode is the only
            # behaviour anyway.
            #
            # Note this is the opposite failure from the one this line used to
            # guard against: the older page-mode firmware HELD the pattern and
            # re-fired it every PATTERN_STRIDE ticks, ~120 times over 2.0s,
            # dumping ~40x the intended ink. Either way the fix is the same --
            # pin line mode first -- so the call stays.
            await client.set_print_mode(NOZZLE_MODE_LINE, required=False)
            _print_start_button_hint()
            print(f"All {IMAGE_HEIGHT} nozzles ON for {on_seconds:.1f}s ...")
            await client.write_column(all_on)
            await asyncio.sleep(on_seconds)

            print("Sweeping a single nozzle down all rows ...")
            for frame in sweep:
                await client.write_column(frame)
                await asyncio.sleep(sweep_step)
            await client.write_blank()
        # Not "done" / success -- the frames were sent over BLE, that's all
        # this can confirm from here. If nothing visibly fired or no current
        # was drawn, the START button was most likely not pressed/held.
        print("Nozzle test: all frames sent. If nothing fired and no current "
              "was drawn, the physical START button on the device was most "
              "likely not pressed (or not held) throughout the test.")
    except Exception as exc:
        print(f"Nozzle test failed (BLE): {exc}")


# ============================================================================
# --ble-benchmark : measure the BLE column throughput / latency ceiling
# ============================================================================
async def ble_benchmark(ble: BleSettings, tracking: TrackingSettings,
                        n_fast: int = 400, n_probe: int = 60) -> None:
    """
    Measure how fast columns can actually be pushed over BLE. This is the ceiling
    that makes position printing speed-dependent: if the head crosses columns
    faster than this, they lag no matter how good the position is.

      * throughput: ``n_fast`` write-without-response frames as fast as possible.
      * latency:    ``n_probe`` write-*with-response* frames -> true GATT
        round-trip (~ the connection interval), i.e. real delivery latency.

    Blank frames are used so nothing is actually printed.
    """
    loop = asyncio.get_event_loop()
    mmpc = tracking.mm_per_column
    try:
        async with PrintheadBLE(ble) as client:
            # Measure in a known mode, so the reported cols/s means something.
            await client.set_print_mode(NOZZLE_MODE_LINE, required=False)
            _print_start_button_hint()
            print(f"Throughput: sending {n_fast} frames (no response) ...")
            t0 = loop.time()
            for _ in range(n_fast):
                await client.write_column(BLANK_FRAME)
            dt = loop.time() - t0
            thr = n_fast / dt if dt > 0 else 0.0

            print(f"Latency: {n_probe} frames (with response) ...")
            lat = []
            for _ in range(n_probe):
                t = loop.time()
                await client.write_column(BLANK_FRAME, response=True)
                lat.append((loop.time() - t) * 1000.0)
            await client.write_blank()

            lat.sort()
            avg = sum(lat) / len(lat)
            p95 = lat[min(len(lat) - 1, int(0.95 * len(lat)))]
            max_speed = thr * mmpc

            print("---- BLE benchmark ----")
            print(f"  no-response throughput : {thr:.0f} cols/s "
                  f"({1000.0 / thr:.1f} ms/col)" if thr else "  throughput: n/a")
            print(f"  with-response latency  : avg {avg:.1f} ms  "
                  f"p95 {p95:.1f} ms  max {lat[-1]:.1f} ms")
            print(f"  => at {mmpc:.3f} mm/col, columns keep up to ~{max_speed:.1f} "
                  f"mm/s. Above that, position printing will lag / depend on speed.")
    except Exception as exc:
        print(f"BLE benchmark failed: {exc}")
