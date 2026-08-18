"""
Amfitrack positioning
======================

Turns the electromagnetic 6-DOF pose from an Amfitrack sensor into either the
scalar "how far has the printhead travelled" value the legacy 1D pipeline
needs to pick a column, or a 2-D page-plane position for freehand printing.

Three pieces:
  * :class:`AmfitrackTracker` - reads raw ``(x, y, z)`` position (mm), and
    orientation quaternion when available, from the USB dongle via
    ``amfiprot`` / ``amfiprot_amfitrack``. :class:`SimulatedTracker` is a
    drop-in replacement that fakes motion so the closed loop can be tested
    without hardware.
  * :class:`AdvanceMapper` - the 1D pipeline's mapper: converts a 3-D position
    into travel distance along the print direction. The rig's measured travel
    axis is X (the default), but the sensor may be mounted rotated on a given
    rig, so this either picks a fixed axis or auto-calibrates the direction.
  * :class:`PageMapper` - the 2-D counterpart for freehand page printing:
    projects a position onto a calibrated page plane (see ``calibration.py``)
    instead of a single travel scalar, and then corrects for the tracked
    sensor not being physically at the nozzle bar (see ``geometry.py``'s
    ``SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM``/``SENSOR_TO_NOZZLE_COL_MM``).
"""

from __future__ import annotations

import math
import time
from typing import Callable, Optional

import numpy as np

from .calibration import PageCalibration
from .config import TrackingSettings
from .geometry import (
    NOZZLE_BAR_SPAN_MM,
    SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM,
    SENSOR_TO_NOZZLE_COL_MM,
)
from .rotation import cart_rotation_angles, twist_about_axis, yaw_about_normal

_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def _wrap_pi(angle_rad: float) -> float:
    """Wrap to ``(-pi, +pi]`` -- used when subtracting a boresight's own
    twist from a live one, since the difference of two already-wrapped
    angles can land outside the range (see PageMapper.project)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


# ============================================================================
# Position low-pass filter
# ============================================================================
class PositionFilter:
    """
    First-order low-pass (EMA) on the 3-D position, parameterised by a time
    constant so it is independent of the polling rate.

    The Amfitrack signal is noisy; unfiltered it makes the derived column jump
    around, which trips the "stopped"/"reversed" checks and leaves irregular gaps
    and uneven line widths in the print. Smoothing the position first fixes that.
    Larger ``tau_s`` = smoother but more lag (a roughly constant position offset).
    """

    def __init__(self, tau_s: float):
        self.tau = float(tau_s)
        self._state = None
        self._t = None

    def reset(self) -> None:
        self._state = None
        self._t = None

    def update(self, pos, t):
        pos = np.asarray(pos, dtype=float)
        if self.tau <= 0.0 or self._state is None:
            self._state = pos.copy()
            self._t = t
            return self._state
        dt = t - self._t
        self._t = t
        if dt <= 0.0:
            return self._state
        alpha = dt / (self.tau + dt)
        self._state = self._state + alpha * (pos - self._state)
        return self._state


# ============================================================================
# Position -> travel distance
# ============================================================================
class AdvanceMapper:
    """
    Maps a 3-D position (mm) to a scalar "advance" (mm along the travel axis),
    relative to an origin set at the start of a pass.

    Fixed-axis mode (default: Y): ``advance = axis_sign * (pos[axis] - origin[axis])``.
    Auto-calibrate mode: the travel direction is learned from the first
    ``calib_distance_mm`` of motion, then ``advance = dot(pos - origin, dir)``.
    """

    def __init__(self, settings: TrackingSettings):
        self.settings = settings
        self._axis = _AXIS_INDEX[settings.advance_axis]
        self._origin: Optional[np.ndarray] = None
        self._direction: Optional[np.ndarray] = None  # unit vector (auto mode)

    def set_origin(self, pos) -> None:
        self._origin = np.asarray(pos, dtype=float)
        self._direction = None

    @property
    def calibrated(self) -> bool:
        return not self.settings.auto_calibrate or self._direction is not None

    def advance(self, pos) -> Optional[float]:
        """
        Return travel distance in mm, or ``None`` while auto-calibration is still
        collecting the initial motion (caller should hold the current column).
        """
        if self._origin is None:
            raise RuntimeError("set_origin() must be called before advance().")
        pos = np.asarray(pos, dtype=float)

        if not self.settings.auto_calibrate:
            return self.settings.axis_sign * float(pos[self._axis] - self._origin[self._axis])

        if self._direction is None:
            delta = pos - self._origin
            dist = float(np.linalg.norm(delta))
            if dist < self.settings.calib_distance_mm:
                return None                       # not enough motion yet
            self._direction = delta / dist
            print(f"[calib] travel direction locked to "
                  f"[{self._direction[0]:+.2f} {self._direction[1]:+.2f} "
                  f"{self._direction[2]:+.2f}]")
        return self.settings.axis_sign * float(np.dot(pos - self._origin, self._direction))


class PageMapper:
    """
    Maps a 3-D sensor position (mm) onto the calibrated 2-D page plane, in
    the nozzle-0-referenced ``(u, v)`` frame :class:`~printhead.coverage.
    CoverageEngine` expects -- the page-mode counterpart to
    :class:`AdvanceMapper`'s single scalar "advance". This does two things:

      1. Projects through a :class:`~printhead.calibration.PageCalibration`
         fit ahead of time (trace two page edges, see ``calibration.py``)
         rather than locking anything at pass-start: the calibration's own
         origin (a traced page corner) already anchors ``(u, v)``, so one
         calibration stays valid across many passes as long as the paper and
         cart mount haven't moved. This step alone gives the *sensor's* own
         page-plane position, not the nozzle bar's.
      2. Corrects for the tracked sensor not being physically at the nozzle
         bar: the cart carries the sensor and the 152-nozzle bar a fixed
         offset apart (measured, see ``geometry.py``'s
         ``SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM``/``SENSOR_TO_NOZZLE_COL_MM``),
         so without this the reported ``(u, v)`` would be off by exactly that
         offset from where ink is actually deposited.

    That sensor->nozzle offset is a vector in the CART's own frame, not the
    page's -- it stays pointing "from the sensor towards the bar" no matter
    which way the cart is rotated. Treating it as a constant page-frame
    shift (the pre-rotation-correction behaviour) is only correct at zero
    yaw; measured from a real pass (``pass5.csv``), cart yaw about the page
    normal spans 75.6 deg, which turns a 62.36mm offset into up to ~76mm of
    position error. ``project()`` therefore rotates the offset by the
    cart's current yaw -- see ``rotation.yaw_about_normal`` -- whenever a
    ``PageCalibration.boresight_quat`` reference pose is available; with no
    boresight captured (every calibration saved before this feature existed)
    it falls back to exactly the old constant-shift behaviour, since there
    is no reference pose to measure yaw against.
    """

    def __init__(self, calibration: PageCalibration,
                 sensor_offset_row_mm: float = SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM,
                 sensor_offset_col_mm: float = SENSOR_TO_NOZZLE_COL_MM,
                 boresight_offset_rad: float = 0.0):
        self.calibration = calibration
        # Convert the measured bar-CENTRE-referenced offset to the nozzle-0-
        # referenced one CoverageEngine actually needs: CoverageEngine places
        # nozzle p at row base_row + p for p in 0..NUM_NOZZLES-1, so the bar's
        # centre (nozzle index (NUM_NOZZLES-1)/2) sits exactly
        # NOZZLE_BAR_SPAN_MM/2 further along +v than nozzle 0 -- exact, not
        # approximate, because NOZZLE_BAR_SPAN_MM is defined in geometry.py as
        # exactly (NUM_NOZZLES - 1) * NOZZLE_PITCH_MM. Deliberately NOT
        # NOZZLE_BAR_WIDTH_MM/2: that constant is the bar's OUTER edge-to-edge
        # width (152 cells), which is half a pitch too wide for a nozzle-0-
        # to-centre distance -- see geometry.py's comment above
        # NOZZLE_BAR_SPAN_MM for why the two are no longer interchangeable.
        # See geometry.py's SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM for the
        # measurement itself.
        self._row_offset_mm = sensor_offset_row_mm - NOZZLE_BAR_SPAN_MM / 2.0
        self._col_offset_mm = sensor_offset_col_mm
        # Additive fine-tune on top of the yaw computed from the captured
        # boresight (see cli.py's --boresight-deg): lets a print that comes
        # out rotated be corrected without re-capturing the boresight, same
        # "adjust rather than rebuild" idea as sensor_offset_row_mm/-col_mm
        # above. No effect when boresight_quat is None (there is no captured
        # yaw to fine-tune in the first place).
        self.boresight_offset_rad = boresight_offset_rad
        # Yaw about the page normal (radians), most recently computed by
        # project() below -- 0.0 (assume the boresight pose) until either a
        # live orientation sample arrives or forever, if the calibration has
        # no boresight_quat at all. Exposed as a plain attribute (mirrors
        # CoverageEngine.last_in_bounds) rather than a getter method so a
        # caller computes it exactly ONCE per sample, inside project(), and
        # reuses this value for CoverageEngine.step()'s yaw_rad instead of
        # calling yaw_about_normal a second time (see controller.py's
        # _print_freehand_pass).
        self.last_yaw_rad = 0.0
        # Roll/pitch about the page normal's in-plane axes (radians), same
        # "0.0 until a live sample arrives, or forever with no boresight"
        # semantics as last_yaw_rad above -- see cart_rotation_angles.
        # DIAGNOSTIC ONLY: unlike last_yaw_rad, these two never feed the
        # sensor->nozzle offset rotation below or CoverageEngine.step(), see
        # project()'s docstring and cart_rotation_angles's own docstring for
        # why (measured tilt is small, correcting it is a deliberate
        # non-goal). Exposed as plain attributes for the same "compute once
        # in project(), reuse from here" reason as last_yaw_rad.
        self.last_roll_rad = 0.0
        self.last_pitch_rad = 0.0

    def set_origin(self, pos) -> None:
        """
        Re-zero the page frame's origin at ``pos`` (a world-space position).

        Only meaningful for the calibration-free simple frame (see
        ``PageCalibration.simple_frame``), whose origin starts at the
        tracker's world zero -- somewhere on the table, not on the paper --
        so without this every sample would land far outside the target image
        and nothing would print. The controller calls it once at pass start,
        which makes "wherever the cart is when you press START" the page's
        ``(0, 0)``, exactly like ``AdvanceMapper.set_origin`` does for the 1D
        line mode.

        A traced calibration must NOT be re-zeroed this way: its origin is a
        real, measured page corner, and one calibration is meant to stay
        valid across many passes (see the class docstring). The controller
        therefore only calls this for the simple frame.
        """
        self.calibration.origin = np.asarray(pos, dtype=float).copy()

    def capture_boresight(self, quat) -> None:
        """
        Adopt ``quat`` as the reference pose yaw is measured against.

        Used by the calibration-free simple frame, which has no traced
        boresight: the pose held at START becomes "0 deg", so a subsequent
        flat turn of the cart reads out as exactly that turn. Without this
        the frame would have to assume the sensor is mounted square with the
        tracker axes -- it generally is not (measured: 120 deg off), and
        assuming so corrupts yaw badly and non-linearly. See
        ``PageCalibration.simple_frame`` for the measured before/after.

        Calling it with ``None`` is a no-op, leaving ``boresight_quat``
        unset so ``project()`` applies no rotation correction at all
        (the same safe fallback as a pre-boresight traced calibration)
        rather than silently referencing a wrong pose.
        """
        if quat is not None:
            self.calibration.boresight_quat = np.asarray(quat, dtype=float).copy()

    def zero_at_nozzle(self, pos, quat=None) -> None:
        """
        Re-zero the page frame so that ``project(pos, quat)`` returns
        ``(0, 0)`` -- i.e. put the page's origin under the NOZZLE BAR, not
        under the sensor.

        ``set_origin(pos)`` alone is not enough, and gets this visibly wrong:
        it puts the origin at the *sensor*, but ``project()`` deliberately
        reports where the nozzle bar is, a fixed
        ``SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM`` (~62mm) away. Zeroing at the
        sensor therefore leaves the bar at ``v ~ 54.8mm`` (at the +62.36mm
        sign this constant originally shipped with -- ``62.36 -
        NOZZLE_BAR_SPAN_MM / 2``; magnitude only changes with the
        constant's current sign, the failure mode does not) -- far outside a
        15.2mm-tall page -- so every sample reads out of bounds and NOTHING
        prints. Seen for real on the first simulated simple-frame pass.

        Shifting the origin by ``d`` along ``e_col`` moves ``u`` by
        ``-d * scale_col`` (see ``PageCalibration.project``), so cancelling a
        residual ``u0`` needs a shift of ``+u0 / scale_col``; likewise for
        ``v``. The frame's own axes are used rather than world axes so this
        stays correct for a rotated (traced) frame too, even though only the
        simple frame currently calls it.

        ``quat`` matters because the sensor->nozzle offset is rotated by the
        cart's current yaw: this zeroes at the bar's position *for the pose
        held at START*, which is exactly the pose the operator is aiming
        with.
        """
        self.set_origin(pos)
        u0, v0, _ = self.project(pos, quat)
        cal = self.calibration
        cal.origin = (cal.origin
                      + (u0 / cal.scale_col) * cal.e_col
                      + (v0 / cal.scale_row) * cal.e_row)

    def project(self, pos, quat=None) -> "tuple[float, float, float]":
        """
        World position (mm) -> nozzle-0-referenced page-plane
        ``(u_mm, v_mm, z_mm)``. Feed it an already-filtered position (see
        :class:`PositionFilter`) -- this does no smoothing of its own, only
        the fixed geometric projection plus the (possibly yaw-rotated)
        sensor->nozzle offset.

        ``quat`` is the live orientation sample for this tick (``(qx, qy,
        qz, qw)``, see ``AmfitrackTracker.read_pose``), or ``None`` if this
        tick's packet carried no orientation. Three cases:

          * no ``boresight_quat`` on this calibration at all -> no rotation
            correction, ever (``self.last_yaw_rad`` stays 0.0 forever): this
            is every calibration saved before boresight capture existed, and
            deliberately does NOT fall back to guessing a reference pose
            from whatever orientation the cart happened to have at pass
            start -- see the class docstring / README for why.
          * boresight present and ``quat`` given -> compute the current yaw
            (``rotation.yaw_about_normal``) and use it.
          * boresight present but ``quat is None`` this tick (a dropped
            orientation packet) -> reuse the last computed yaw rather than
            snapping to 0. An intermittent quaternion dropout must not make
            the correction flicker between corrected and uncorrected pixel
            placement sample to sample.
        """
        u, v, z = self.calibration.project(pos)
        if self.calibration.absolute_twist_yaw and quat is not None:
            # Simple frame: the cart's ABSOLUTE twist about each page axis,
            # no reference pose subtracted -- the hardware owner's own
            # known-good readout (see rotation.twist_about_axis, a faithful
            # port of their amfitrack_live_pose.py). This needs no boresight
            # at all, which is the point: in this frame a captured reference
            # has repeatedly been the weak link (blind first-sample capture
            # grabbing whatever pose the cart was in; the rig's saved
            # boresight measuring ~110 deg off flat), and an absolute
            # reading has no such failure mode -- the same physical
            # orientation always reads the same, run to run.
            #
            # The page axes ARE the tracker axes here (simple_frame sets
            # e_col = x, e_row = y, so the normal is z), so twisting about
            # them is exactly the operator's axis-(0,0,1)/(1,0,0)/(0,1,0)
            # calls. Written against the calibration's own axes rather than
            # hardcoded x/y/z so it stays honest if simple_frame's axes
            # ever change.
            #
            # A captured/pinned boresight, when present, only shifts the
            # ZERO POINT: its own twist about the same axis is subtracted,
            # so the pose it was captured at still reads 0 -- preserving
            # --simple-boresight's meaning without reintroducing the
            # boresight-relative math it used to rely on. With no boresight
            # the reading is simply absolute.
            normal = np.cross(np.asarray(self.calibration.e_col, dtype=float),
                              np.asarray(self.calibration.e_row, dtype=float))
            bore = self.calibration.boresight_quat
            yaw = twist_about_axis(quat, normal)
            if bore is not None:
                yaw = _wrap_pi(yaw - twist_about_axis(bore, normal))
            self.last_yaw_rad = yaw + self.boresight_offset_rad

            # Roll/pitch deliberately NOT switched to the same per-axis
            # twist. Three independent single-axis twists are not an
            # orthogonal decomposition of one rotation: read that way, a
            # PURE turn about the normal already makes the other two axes
            # report nonzero (measured: a flat 15 deg turn from the rig's
            # real mounting pose reads 15 deg of "roll"). That is fine for
            # the operator's live readout, where each axis is looked at on
            # its own, but here roll/pitch are specifically the "is the cart
            # tilted?" indicator, and a yaw-driven reading would make them
            # useless for that. They therefore keep the boresight-relative
            # swing-twist below, and stay 0.0 when no boresight exists --
            # unchanged behaviour. Only the Z-AXIS ROTATION was asked to
            # move to the absolute twist, and only that moved.
            if bore is not None:
                self.last_roll_rad, self.last_pitch_rad, _ = cart_rotation_angles(
                    quat, bore, self.calibration.e_col, self.calibration.e_row
                )
        elif self.calibration.boresight_quat is not None and quat is not None:
            self.last_yaw_rad = yaw_about_normal(
                quat, self.calibration.boresight_quat,
                self.calibration.e_col, self.calibration.e_row
            ) + self.boresight_offset_rad
            # Diagnostic-only roll/pitch (see cart_rotation_angles's
            # docstring) -- last_yaw_rad above, from the untouched
            # yaw_about_normal path, stays the single source of truth for
            # yaw; this function's own yaw component is discarded rather
            # than replacing it, even though the two are mathematically
            # identical. boresight_offset_rad (--boresight-deg) is a
            # yaw-only fine-tune and must NOT be added to roll/pitch.
            self.last_roll_rad, self.last_pitch_rad, _ = cart_rotation_angles(
                quat, self.calibration.boresight_quat,
                self.calibration.e_col, self.calibration.e_row
            )

        # cos(0.0) == 1.0 and sin(0.0) == 0.0 exactly in IEEE 754, so at
        # yaw == 0.0 (no boresight, ever, or a boresight pose sample) this
        # reduces bit-for-bit to the pre-rotation "u + col_offset_mm,
        # v + row_offset_mm" formula -- no separate zero-yaw code path
        # needed to keep that identical.
        cos_y = math.cos(self.last_yaw_rad)
        sin_y = math.sin(self.last_yaw_rad)
        du = self._col_offset_mm * cos_y - self._row_offset_mm * sin_y
        dv = self._col_offset_mm * sin_y + self._row_offset_mm * cos_y
        return u + du, v + dv, z


# ============================================================================
# Real hardware
# ============================================================================
class AmfitrackTracker:
    """Reads position from the Amfitrack USB dongle (amfiprot protocol)."""

    def __init__(self, settings: TrackingSettings):
        self.settings = settings
        self._conn = None
        self._devices = []
        self._last: Optional[np.ndarray] = None
        self._last_quat: Optional[np.ndarray] = None

    def open(self) -> None:
        # Imported lazily so the package works (dry-run / simulate) without the
        # vendor libraries installed.
        import amfiprot
        import amfiprot_amfitrack as amfitrack

        s = self.settings
        # Open the USB dongle: the sensor product id first, the source id as a
        # fallback (mirrors the known-working AmfiPoseProvider).
        try:
            conn = amfiprot.USBConnection(s.vendor_id, s.product_id)
        except Exception:
            conn = amfiprot.USBConnection(s.vendor_id, s.product_id_source)

        nodes = conn.find_nodes()
        # Attach to every node whose name contains "Sensor" (optionally narrowed
        # to a single tx_id via --sensor-id).
        self._devices = []
        for node in nodes:
            if "Sensor" not in getattr(node, "name", ""):
                continue
            if s.sensor_id is not None and getattr(node, "tx_id", None) != s.sensor_id:
                continue
            print(f"Amfitrack sensor: {node.name} {getattr(node, 'uuid', '')}")
            self._devices.append(amfitrack.Device(node))

        if not self._devices:
            raise RuntimeError(
                "No Amfitrack Sensor node found (no node.name containing 'Sensor').")

        conn.start()
        self._conn = conn

    def read_position(self) -> Optional[np.ndarray]:
        """Return the latest ``(x, y, z)`` in mm, or ``None`` if no new sample."""
        pos, _ = self._read_latest()
        return pos

    def read_pose(self) -> "tuple[Optional[np.ndarray], Optional[np.ndarray]]":
        """
        Return the latest ``(position, quaternion)``, either ``None`` if that
        part was absent from every packet drained this call.

        ``quaternion`` is ``(qx, qy, qz, qw)`` orientation, or ``None`` if the
        connected SDK/firmware never reports it -- see ``_extract_pose``. This
        field has been confirmed on real hardware (same ``payload.emf.quat_*``
        access path), but is not yet relied on anywhere in the print pipeline.
        """
        return self._read_latest()

    def _read_latest(self) -> "tuple[Optional[np.ndarray], Optional[np.ndarray]]":
        pos = None
        quat = None
        for dev in self._devices:
            try:
                while dev.packet_available():
                    p, q = self._extract_pose(dev.get_packet().payload)
                    if p is not None:
                        pos = p
                    if q is not None:
                        quat = q
            except Exception:
                continue
        if pos is not None:
            self._last = pos
        if quat is not None:
            self._last_quat = quat
        return pos, quat

    # ---- single adapter point for SDK-version differences ------------------
    @staticmethod
    def _extract_pose(payload) -> "tuple[Optional[np.ndarray], Optional[np.ndarray]]":
        """
        Pull an ``(x, y, z)`` position in **mm**, and an orientation quaternion
        if the payload carries one, out of an amfiprot packet.

        The primary position layout is the confirmed-working
        ``payload.emf.pos_{x,y,z}``. A few other layouts are tried as a
        fallback for differing SDK versions; adjust HERE if your SDK reports
        the position differently.

        Quaternion support (``payload.emf.quat_{x,y,z,w}``) is confirmed
        working on real hardware: the identical ``payload.emf`` object this
        reads position from also carries ``quat_x/y/z/w``, verified against a
        reference implementation of this same SDK (the ``AmfiPoseProvider``
        this class's connection logic already mirrors, see ``open()``).
        Surfaced via ``read_pose()`` / ``--pos``; usable for freehand
        page-mode cart-orientation correction.
        """
        emf = getattr(payload, "emf", payload)
        pos = None

        # 1) confirmed working: emf.pos_x / pos_y / pos_z (mm)
        if all(hasattr(emf, a) for a in ("pos_x", "pos_y", "pos_z")):
            pos = np.array([emf.pos_x, emf.pos_y, emf.pos_z], dtype=float)
        else:
            # 2) nested .position with .x/.y/.z (mm)
            nested = getattr(emf, "position", None)
            if nested is not None and hasattr(nested, "x"):
                pos = np.array([nested.x, nested.y, nested.z], dtype=float)
            # 3) flat .x/.y/.z on the emf payload (mm)
            elif all(hasattr(emf, a) for a in ("x", "y", "z")):
                pos = np.array([emf.x, emf.y, emf.z], dtype=float)
            else:
                # 4) C-SDK style names in metres -> convert to mm
                metre_names = ("position_x_in_m", "position_y_in_m", "position_z_in_m")
                if all(hasattr(emf, n) for n in metre_names):
                    pos = np.array([getattr(emf, n) for n in metre_names],
                                   dtype=float) * 1000.0

        quat = None
        if all(hasattr(emf, a) for a in ("quat_x", "quat_y", "quat_z", "quat_w")):
            quat = np.array([emf.quat_x, emf.quat_y, emf.quat_z, emf.quat_w],
                            dtype=float)

        return pos, quat                           # either may be None

    def close(self) -> None:
        if self._conn is not None:
            for method in ("stop", "close"):
                try:
                    getattr(self._conn, method)()
                except Exception:
                    pass
        self._conn = None
        self._devices = []


# ============================================================================
# Hardware-free simulator
# ============================================================================
# Elapsed seconds since the pass started -> a fake (x, y, z) position in mm.
PathFn = Callable[[float], np.ndarray]


def waypoints_path(points, speed_mm_s: float, loop: bool = False) -> PathFn:
    """
    Build a :data:`PathFn` that walks piecewise-linear through ``points`` (an
    ``(N, 3)``-like sequence of mm positions) at a constant speed, starting at
    ``points[0]``.

    Used to make :class:`SimulatedTracker` fake a genuine 2D/3D scribble --
    loops, revisits, vertical sweeps -- instead of straight-line travel along
    one axis, which is what exercising page-mode calibration/coverage without
    hardware needs and a single-axis fake cannot produce.

    ``loop=True`` repeats the path indefinitely once the last point is
    reached; otherwise the tracker holds at the final point.
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < 2:
        raise ValueError("waypoints_path needs at least two (x, y, z) points.")
    seg_vec = np.diff(pts, axis=0)                        # (N-1, 3)
    seg_len = np.linalg.norm(seg_vec, axis=1)              # (N-1,)
    if np.any(seg_len <= 0):
        raise ValueError("waypoints_path: consecutive points must not coincide.")
    seg_dir = seg_vec / seg_len[:, None]
    seg_end_dist = np.cumsum(seg_len)                      # cumulative arc length
    total_len = float(seg_end_dist[-1])

    def path(t: float) -> np.ndarray:
        dist = speed_mm_s * t
        if loop and total_len > 0:
            dist = dist % total_len
        if dist >= total_len:
            return pts[-1].copy()
        i = int(np.searchsorted(seg_end_dist, dist, side="right"))
        start_dist = seg_end_dist[i - 1] if i > 0 else 0.0
        return pts[i] + seg_dir[i] * (dist - start_dist)

    return path


class SimulatedTracker:
    """
    Fakes printhead motion so ``--simulate`` can exercise the position -> column
    loop without hardware.

    Default behaviour (unchanged): constant speed along the configured travel
    axis -- the 1D ``--mode line`` use case.

    Pass ``path_fn`` (see :func:`waypoints_path`) to instead walk an arbitrary
    2D/3D path. Page-mode calibration and coverage need this: they must handle
    loops, revisits and vertical sweeps that a single-axis fake cannot exercise.
    """

    def __init__(self, settings: TrackingSettings, speed_mm_s: float = 50.0,
                 path_fn: Optional[PathFn] = None):
        self.settings = settings
        self.speed_mm_s = speed_mm_s
        self._axis = _AXIS_INDEX[settings.advance_axis]
        self._path_fn = path_fn
        self._t0: Optional[float] = None

    def open(self) -> None:
        self._t0 = time.monotonic()
        if self._path_fn is None:
            print(f"SimulatedTracker: {self.speed_mm_s:.0f} mm/s along "
                  f"{self.settings.advance_axis}-axis.")
        else:
            print("SimulatedTracker: following a custom 2D/3D path.")

    def read_position(self) -> np.ndarray:
        if self._t0 is None:
            self._t0 = time.monotonic()
        elapsed = time.monotonic() - self._t0
        if self._path_fn is not None:
            return np.asarray(self._path_fn(elapsed), dtype=float)
        travelled = self.speed_mm_s * elapsed
        pos = np.zeros(3, dtype=float)
        pos[self._axis] = self.settings.axis_sign * travelled
        return pos

    def read_pose(self) -> "tuple[np.ndarray, Optional[np.ndarray]]":
        """Same interface as :meth:`AmfitrackTracker.read_pose`, so callers
        (e.g. ``diagnostics.monitor_position``) don't need to branch on tracker
        type. The simulator never fakes orientation, so ``quaternion`` is
        always ``None``."""
        return self.read_position(), None

    def close(self) -> None:
        self._t0 = None


def make_tracker(settings: TrackingSettings, simulate: bool,
                 path_fn: Optional[PathFn] = None):
    """Factory: real dongle tracker or the hardware-free simulator.

    ``path_fn`` (see :func:`waypoints_path`) is only used when
    ``simulate=True``; ignored for the real tracker."""
    if simulate:
        return SimulatedTracker(settings, path_fn=path_fn)
    return AmfitrackTracker(settings)
