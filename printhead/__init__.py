"""
printhead
=========

Control the nozzles of an HP302 cartridge over BLE (the "PrintheadBLE" ESP32),
optionally driving column selection from an Amfitrack electromagnetic position
sensor.

Public surface:
    from printhead.config import RenderSettings, BleSettings, TrackingSettings
    from printhead.controller import PrintController
    from printhead.cli import main
"""

from .config import BleSettings, RenderSettings, TrackingSettings
from .controller import PrintController

__all__ = [
    "RenderSettings",
    "BleSettings",
    "TrackingSettings",
    "PrintController",
]

__version__ = "1.0.0"
