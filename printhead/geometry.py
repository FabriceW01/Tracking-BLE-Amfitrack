"""
Printhead geometry and BLE identifiers
======================================

These constants describe the ESP32 "PrintheadBLE" server and the physical HP302
cartridge layout. They MUST match the firmware (see README_BLE_INTERFACE.md and
main.c). Everything downstream (rendering, framing, streaming) depends on them.
"""

# ----------------------------------------------------------------------------
# BLE identifiers (from README_BLE_INTERFACE.md)
# ----------------------------------------------------------------------------
DEVICE_NAME = "PrintheadBLE"
SERVICE_UUID = "d0567401-5a22-c59f-5243-8c0fa18e257b"
NOZZLE_UUID = "41a9348e-2f6b-8db1-934d-743c6f17649a"   # Write / WriteNoRsp, 19 bytes
START_BTN_UUID = "b473a21f-6e58-6380-2647-abd7cd4a904e"  # Read / Notify, 1 byte 0/1
STARTPOINT_UUID = "cc1087f5-1d92-6ca4-b84f-3e5880e6713d"  # Read / Notify, 1 byte 0/1
# Runtime line/page switch (must match the firmware's power-on default of line
# mode -- see README_BLE_INTERFACE.md "2) Print Mode Characteristic").
MODE_UUID = "f5ad7c1f-f6e1-4dd7-bbb7-d8b9286a88c6"   # Read / Write, 1 byte
NOZZLE_MODE_LINE = 0
NOZZLE_MODE_PAGE = 1
# Speed warning: client writes 1 when the cart is moving too fast to print
# reliably, 0 otherwise. Advisory only -- drives the firmware's repurposed
# HEALTH LED, has no effect on dosing (see README_BLE_INTERFACE.md "3) Speed
# Warning Characteristic" in the firmware repo). Must match the firmware.
SPEED_WARN_UUID = "58c05253-945f-48fc-a26c-989c785d6678"   # Read / Write, 1 byte
# Process stop: client writes 1 once a print pass has ended (any mode, any
# reason -- success, timeout, or an exception), so the firmware can stop I2S
# output without needing a second physical START press. The firmware's own
# START button is a hard toggle with no other way to learn a pass ended on
# its own; without this write, the toggle desyncs from the client's pass
# lifecycle -- see README_BLE_INTERFACE.md "6) Process Stop Characteristic"
# in the firmware repo, and PrintController._run_ble()'s per-pass cleanup
# (the one call site) for the full symptom this fixes. Must match the
# firmware.
PROCESS_STOP_UUID = "a2e1c9d4-7f3b-4a8e-9c1d-5b6f8e2a0d47"   # Write, 1 byte

# ----------------------------------------------------------------------------
# Printhead geometry (must match the firmware)
# ----------------------------------------------------------------------------
# The nozzle frame is 19 bytes (was 21): the top byte and the bottom byte of the
# old 21-byte frame are dropped so the payload fits within the default BLE ATT
# MTU (23 bytes -> 20 usable). A frame with >20 payload bytes cannot be sent as a
# single Write-Without-Response, which silently truncated the print into ~21
# coarse blocks instead of the full nozzle resolution.
#
# Frame bit j (byte j // 8, bit j % 8, LSB-first) drives PHYSICAL nozzle
# NOZZLE_OFFSET + j. The firmware reconstructs the old layout by zero-padding one
# byte at each end (see the BLE-server change prompt). Physical nozzles 0..7 and
# 160..167 are therefore no longer used.
ROW_BYTES = 19                                # BLE_NOZZLE_ROW_BYTES (was 21)
NUM_NOZZLES = ROW_BYTES * 8                   # 152 bits carried by the frame
FIRST_NOZZLE = 0                              # image row 0 -> frame bit 0
LAST_NOZZLE = NUM_NOZZLES - 1                 # == 151
IMAGE_HEIGHT = LAST_NOZZLE - FIRST_NOZZLE + 1  # == 152 usable rows
NOZZLE_OFFSET = 8                             # frame bit j -> physical nozzle j + 8

# A frame with no nozzle firing; used to stop printing / start clean.
BLANK_FRAME = bytes(ROW_BYTES)

# ----------------------------------------------------------------------------
# Physical nozzle-bar spacing (freehand page-mode; user-measured, confirmed)
# ----------------------------------------------------------------------------
# SUPERSEDED measurement: an earlier pass measured the bar at 15.2mm across
# the 152 nozzle cells edge-to-edge, giving NOZZLE_PITCH_MM = 15.2 / 152 ==
# 0.1mm exactly -- a suspiciously clean number that was itself used as
# evidence for the "152 cells, not 151 gaps" interpretation below (see git
# history for that reasoning in full). A later, more careful re-measurement
# directly against the physical bar gives 13.2mm across the SAME 152 cells
# instead; this file now uses that value. Same cell interpretation as
# before -- kept for consistency with the earlier measurement's methodology,
# NOT because the new number happens to divide cleanly (it doesn't: neither
# 13.2 / 152 nor 13.2 / 151 is a round number, so this measurement carries no
# such self-check the way the 15.2mm one did).
#
# This file still carries BOTH derived quantities as separate, named
# constants rather than picking one and hoping every caller means the same
# thing:
#
#   NOZZLE_PITCH_MM     -- the PRIMARY measurement (mm/nozzle). Everything
#                          else here is derived from it, not the other way
#                          around.
#   NOZZLE_BAR_WIDTH_MM -- the total INKED width: 152 cells of
#                          NOZZLE_PITCH_MM each, side by side (13.2mm). This
#                          is the bar's OUTER edge-to-edge extent -- do NOT
#                          halve it to get "nozzle 0 -> bar centre" (that is
#                          off by half a pitch -- easy to get wrong since the
#                          two constants are so close). Use
#                          NOZZLE_BAR_SPAN_MM below for that instead.
#   NOZZLE_BAR_SPAN_MM  -- the CENTRE-TO-CENTRE distance from nozzle 0 to
#                          nozzle NUM_NOZZLES - 1 (151 gaps, ~13.11mm). The
#                          bar's centre sits at nozzle index
#                          (NUM_NOZZLES - 1) / 2, i.e. exactly
#                          NOZZLE_BAR_SPAN_MM / 2 from nozzle 0 -- THIS is
#                          the one to halve for "nozzle 0 -> bar centre" (see
#                          tracking.PageMapper.__init__, the one place that
#                          conversion happens).
NOZZLE_PITCH_MM = 13.2 / 152          # == 0.086842...mm/nozzle (was 0.1mm)
NOZZLE_BAR_WIDTH_MM = NUM_NOZZLES * NOZZLE_PITCH_MM
NOZZLE_BAR_SPAN_MM = (NUM_NOZZLES - 1) * NOZZLE_PITCH_MM

# ----------------------------------------------------------------------------
# Sensor-to-nozzle-bar mechanical offset (freehand page-mode; user-measured)
# ----------------------------------------------------------------------------
# The tracked Amfitrack sensor is not physically located at the printhead: the
# cart carries the sensor at one spot and the 152-nozzle bar at another, a
# fixed few centimetres apart. This is a mechanical property of the cart
# itself -- independent of any particular page/paper -- which is why it lives
# here next to NOZZLE_BAR_WIDTH_MM/NOZZLE_PITCH_MM rather than in a saved
# PageCalibration (a PageCalibration describes where a specific page is, not
# where the nozzles are relative to the sensor; re-calibrating a new page must
# never require re-entering this number).
#
# Measured ("Die Mitte der Nozzle-Reihe ist 62,36 mm verschoben von der
# Y-Koordinate des Amfitrack"): the CENTRE of the 152-nozzle bar sits 62.36 mm
# from the sensor, along the row axis (the axis perpendicular to travel, i.e.
# along the nozzle bar itself -- which world axis that corresponds to is
# mounting-dependent, not a property of this constant, so not claimed here).
# Sign convention: positive means the nozzle bar sits further in the +row
# direction than the sensor. If a test print comes out offset in the wrong
# direction, the fix is to NEGATE this value -- nothing else needs to change.
#
# NEGATED from the original +62.36 measurement: a real print on the rig
# came out shifted the wrong way, confirming (per the sign-convention note
# above) that the bar sits in the -row direction from the sensor on this
# mounting, not +row.
#
# Deliberately named around the bar CENTRE (not nozzle 0) to make the
# reference point unambiguous: tracking.PageMapper is the one place that
# converts this centre-referenced measurement into the nozzle-0-referenced
# offset CoverageEngine actually needs (see its docstring for that math).
SENSOR_TO_NOZZLE_BAR_CENTER_ROW_MM = -62.36

# Column-axis (travel direction) counterpart. Believed to be 0 -- no
# measurement has shown otherwise -- but kept as a named, overridable
# constant rather than assumed silently, in case a future measurement finds
# a real offset here too.
SENSOR_TO_NOZZLE_COL_MM = 0.0
