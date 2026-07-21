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
from typing import Optional

import numpy as np

from .config import TrackingSettings

_AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


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
        pos = None
        for dev in self._devices:
            try:
                while dev.packet_available():
                    candidate = self._extract_position(dev.get_packet().payload)
                    if candidate is not None:
                        pos = candidate
            except Exception:
                continue
        if pos is not None:
            self._last = pos
        return pos

    # ---- single adapter point for SDK-version differences ------------------
    @staticmethod
    def _extract_position(payload) -> Optional[np.ndarray]:
        """
        Pull an ``(x, y, z)`` position in **mm** out of an amfiprot payload.

        The primary layout is the confirmed-working ``payload.emf.pos_{x,y,z}``.
        A few other layouts are tried as a fallback for differing SDK versions;
        adjust HERE if your SDK reports the position differently.
        """
        emf = getattr(payload, "emf", payload)

        # 1) confirmed working: emf.pos_x / pos_y / pos_z (mm)
        if all(hasattr(emf, a) for a in ("pos_x", "pos_y", "pos_z")):
            return np.array([emf.pos_x, emf.pos_y, emf.pos_z], dtype=float)

        # 2) nested .position with .x/.y/.z (mm)
        pos = getattr(emf, "position", None)
        if pos is not None and hasattr(pos, "x"):
            return np.array([pos.x, pos.y, pos.z], dtype=float)

        # 3) flat .x/.y/.z on the emf payload (mm)
        if all(hasattr(emf, a) for a in ("x", "y", "z")):
            return np.array([emf.x, emf.y, emf.z], dtype=float)

        # 4) C-SDK style names in metres -> convert to mm
        metre_names = ("position_x_in_m", "position_y_in_m", "position_z_in_m")
        if all(hasattr(emf, n) for n in metre_names):
            return np.array([getattr(emf, n) for n in metre_names], dtype=float) * 1000.0

        return None                               # not a position-bearing packet

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
class SimulatedTracker:
    """
    Fakes a printhead moving at constant speed along the configured travel axis,
    so ``--simulate`` can exercise the position -> column loop without hardware.
    """

    def __init__(self, settings: TrackingSettings, speed_mm_s: float = 50.0):
        self.settings = settings
        self.speed_mm_s = speed_mm_s
        self._axis = _AXIS_INDEX[settings.advance_axis]
        self._t0: Optional[float] = None

    def open(self) -> None:
        self._t0 = time.monotonic()
        print(f"SimulatedTracker: {self.speed_mm_s:.0f} mm/s along "
              f"{self.settings.advance_axis}-axis.")

    def read_position(self) -> np.ndarray:
        if self._t0 is None:
            self._t0 = time.monotonic()
        travelled = self.speed_mm_s * (time.monotonic() - self._t0)
        pos = np.zeros(3, dtype=float)
        pos[self._axis] = self.settings.axis_sign * travelled
        return pos

    def close(self) -> None:
        self._t0 = None


def make_tracker(settings: TrackingSettings, simulate: bool):
    """Factory: real dongle tracker or the hardware-free simulator."""
    return SimulatedTracker(settings) if simulate else AmfitrackTracker(settings)
