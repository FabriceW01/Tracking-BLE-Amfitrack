"""
Amfitrack positioning
======================

Turns the electromagnetic 6-DOF pose from an Amfitrack sensor into the scalar
"how far has the printhead travelled" value the controller needs to pick a
column.

Two pieces:
  * :class:`AmfitrackTracker` - reads raw ``(x, y, z)`` position (mm) from the
    USB dongle via ``amfiprot`` / ``amfiprot_amfitrack``.  :class:`SimulatedTracker`
    is a drop-in replacement that fakes motion so the closed loop can be tested
    without hardware.
  * :class:`AdvanceMapper` - converts a 3-D position into travel distance along
    the print direction, handling the *rotated* sensor (travel in Y/Z instead of
    X/Y) either by picking a fixed axis or by auto-calibrating the direction.
"""

from __future__ import annotations

import time
from typing import Callable, Optional

import numpy as np

from .config import TrackingSettings

_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


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
