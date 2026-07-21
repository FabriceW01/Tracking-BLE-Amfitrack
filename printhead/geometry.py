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
NOZZLE_UUID = "41a9348e-2f6b-8db1-934d-743c6f17649a"   # Write / WriteNoRsp, 21 bytes
START_BTN_UUID = "b473a21f-6e58-6380-2647-abd7cd4a904e"  # Read / Notify, 1 byte 0/1
STARTPOINT_UUID = "cc1087f5-1d92-6ca4-b84f-3e5880e6713d"  # Read / Notify, 1 byte 0/1

# ----------------------------------------------------------------------------
# Printhead geometry (must match the firmware)
# ----------------------------------------------------------------------------
ROW_BYTES = 21                                # BLE_NOZZLE_ROW_BYTES
NUM_NOZZLES = 168                             # 21 bytes * 8 bits
FIRST_NOZZLE = 2                              # BLACK_TEST_FIRST_NOZZLE
LAST_NOZZLE = 165                             # BLACK_TEST_LAST_NOZZLE
IMAGE_HEIGHT = LAST_NOZZLE - FIRST_NOZZLE + 1  # == 164 usable rows

# A frame with no nozzle firing; used to stop printing / start clean.
BLANK_FRAME = bytes(ROW_BYTES)
