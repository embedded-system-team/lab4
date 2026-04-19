#!/usr/bin/env python3
"""
BLE GATT Client for STM32 AccSensor
RPi (Central) connects to STM32 (Peripheral), subscribes to acceleration
notifications, and can set the sampling frequency via Char_B.

Usage:
    python3 ble_client.py                    # auto-scan for AccSensor
    python3 ble_client.py <MAC_ADDRESS>      # connect directly by address

Requirements:
    pip install bleak

Commands (while running):
    0  → set 12.5 Hz
    1  → set 26 Hz
    2  → set 52 Hz
    3  → set 104 Hz
    r  → read current frequency from STM32
    q  → quit and disconnect
"""

import asyncio
import struct
import sys
import logging
from bleak import BleakClient, BleakScanner

# ---------------------------------------------------------------------------
# UUIDs - must match gatt_db.c definitions
# ---------------------------------------------------------------------------
ACC_SERVICE_UUID = "a0001000-74ee-43ce-b6a1-0002a5d5c51b"
CHAR_A_UUID      = "a0001001-74ee-43ce-b6a1-0002a5d5c51b"  # Accel Data (NOTIFY)
CHAR_B_UUID      = "a0001002-74ee-43ce-b6a1-0002a5d5c51b"  # Sampling Freq (WRITE)

DEVICE_NAME      = "AccSensor"

# ODR index → Hz
ODR_MAP = {0: 12.5, 1: 26.0, 2: 52.0, 3: 104.0}

# ---------------------------------------------------------------------------
# Notification callback
# ---------------------------------------------------------------------------
def notification_handler(sender, data: bytearray):
    """
    Called every time STM32 sends a Char_A notification.
    Data format: [X_L X_H Y_L Y_H Z_L Z_H] int16_t little-endian, unit: mg
    """
    if len(data) < 6:
        print(f"\r[WARN] Unexpected data length: {len(data)}")
        return

    x, y, z = struct.unpack_from("<hhh", data, 0)
    # \r clears the current input line before printing,
    # keeping the display readable while input() is waiting
    print(f"\rAccel  X={x:6d} mg  Y={y:6d} mg  Z={z:6d} mg")

# ---------------------------------------------------------------------------
# GATT helpers
# ---------------------------------------------------------------------------
async def set_sampling_freq(client: BleakClient, odr_idx: int):
    if odr_idx not in ODR_MAP:
        print(f"[ERROR] Invalid ODR index {odr_idx}. Valid: 0-3")
        return
    payload = bytes([odr_idx])
    await client.write_gatt_char(CHAR_B_UUID, payload, response=True)
    print(f"[OK] Sampling frequency → {ODR_MAP[odr_idx]} Hz (index={odr_idx})")

async def read_sampling_freq(client: BleakClient):
    data = await client.read_gatt_char(CHAR_B_UUID)
    odr_idx = data[0]
    freq = ODR_MAP.get(odr_idx, "unknown")
    print(f"[INFO] Current sampling frequency: {freq} Hz (index={odr_idx})")
    return odr_idx

# ---------------------------------------------------------------------------
# Interactive command loop
# ---------------------------------------------------------------------------
HELP = (
    "\n"
    "  0  → set 12.5 Hz\n"
    "  1  → set 26 Hz\n"
    "  2  → set 52 Hz\n"
    "  3  → set 104 Hz\n"
    "  r  → read current frequency\n"
    "  q  → quit and disconnect\n"
)

async def command_loop(client: BleakClient):
    print(HELP)
    loop = asyncio.get_event_loop()

    while True:
        # run_in_executor lets asyncio keep processing notifications
        # while we wait for user input
        try:
            cmd = await loop.run_in_executor(None, input, "cmd> ")
        except EOFError:
            # Handle Ctrl-D gracefully
            break

        cmd = cmd.strip().lower()

        if cmd == "q":
            print("\nDisconnecting...")
            break
        elif cmd == "r":
            await read_sampling_freq(client)
        elif cmd in ("0", "1", "2", "3"):
            await set_sampling_freq(client, int(cmd))
        elif cmd in ("h", "help", "?"):
            print(HELP)
        elif cmd == "":
            pass  # ignore empty input
        else:
            print(f"[WARN] Unknown command '{cmd}'. Type 'h' for help.")

# ---------------------------------------------------------------------------
# Client session
# ---------------------------------------------------------------------------
async def run_client(client: BleakClient):
    print(f"Connected: {client.is_connected}")

    # Verify our custom service is present
    service_found = any(
        s.uuid.lower() == ACC_SERVICE_UUID for s in client.services
    )
    if not service_found:
        print(f"[ERROR] AccSensor service not found on device.")
        print("Available services:")
        for s in client.services:
            print(f"  {s.uuid}")
        return

    print(f"[OK] AccSensor GATT service found.")

    # Read current ODR
    await read_sampling_freq(client)

    # Subscribe to Char_A notifications
    await client.start_notify(CHAR_A_UUID, notification_handler)
    print("[OK] Subscribed to acceleration notifications.")

    # Enter interactive command loop
    await command_loop(client)

    # Unsubscribe cleanly before disconnecting
    await client.stop_notify(CHAR_A_UUID)
    print("Unsubscribed. Goodbye.")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
async def main():
    address_arg = sys.argv[1] if len(sys.argv) > 1 else None

    if address_arg:
        print(f"Connecting to {address_arg}...")
        async with BleakClient(address_arg) as client:
            await run_client(client)
    else:
        print(f"Scanning for '{DEVICE_NAME}'...")
        device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
        if device is None:
            raise RuntimeError(
                f"Device '{DEVICE_NAME}' not found. "
                "Make sure STM32 is powered and advertising."
            )
        print(f"Found: {device.name} [{device.address}]")

        # Pass device object to avoid BlueZ cache invalidation on Linux
        async with BleakClient(device) as client:
            await run_client(client)

# ---------------------------------------------------------------------------
if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING)
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except RuntimeError as e:
        print(f"[ERROR] {e}")
        sys.exit(1)