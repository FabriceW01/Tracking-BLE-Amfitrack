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
