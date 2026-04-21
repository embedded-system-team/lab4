#!/usr/bin/env python3
"""
BLE GATT Client for STM32 AccSensor
Subscribes to:
  Char_A (A0001001): acceleration data notifications
  Char_C (A0001003): significant motion event notifications
Can write:
  Char_B (A0001002): sampling frequency ODR index

Usage:
    python3 ble_client.py                 # auto-scan
    python3 ble_client.py <MAC_ADDRESS>   # direct connect

Commands:
    0-3  set ODR (0=12.5Hz 1=26Hz 2=52Hz 3=104Hz)
    r    read current ODR from STM32
    q    quit
"""

import asyncio
import struct
import sys
import logging
from datetime import datetime
from bleak import BleakClient, BleakScanner

# ---------------------------------------------------------------------------
# UUIDs
# ---------------------------------------------------------------------------
ACC_SERVICE_UUID = "a0001000-74ee-43ce-b6a1-0002a5d5c51b"
CHAR_A_UUID      = "a0001001-74ee-43ce-b6a1-0002a5d5c51b"  # Accel Data NOTIFY
CHAR_B_UUID      = "a0001002-74ee-43ce-b6a1-0002a5d5c51b"  # Sampling Freq WRITE
CHAR_C_UUID      = "a0001003-74ee-43ce-b6a1-0002a5d5c51b"  # Motion Event NOTIFY

DEVICE_NAME = "AccSensor"
ODR_MAP     = {0: 12.5, 1: 26.0, 2: 52.0, 3: 104.0}

# ---------------------------------------------------------------------------
# Notification callbacks
# ---------------------------------------------------------------------------
def accel_notification_handler(sender, data: bytearray):
    """Char_A: 6 bytes [X_L X_H Y_L Y_H Z_L Z_H] int16_t LE, unit: mg"""
    if len(data) < 6:
        print(f"\r[WARN] Accel data length unexpected: {len(data)}")
        return
    x, y, z = struct.unpack_from("<hhh", data, 0)
    print(f"\rAccel  X={x:6d} mg  Y={y:6d} mg  Z={z:6d} mg")

def motion_notification_handler(sender, data: bytearray):
    """Char_C: 1 byte, 0x01 = significant motion detected"""
    ts = datetime.now().strftime("%H:%M:%S")
    print(f"\r*** [{ts}] SIGNIFICANT MOTION DETECTED! ***")

# ---------------------------------------------------------------------------
# GATT helpers
# ---------------------------------------------------------------------------
async def set_sampling_freq(client: BleakClient, odr_idx: int):
    if odr_idx not in ODR_MAP:
        print(f"[ERROR] Invalid ODR index {odr_idx}. Valid: 0-3")
        return
    await client.write_gatt_char(CHAR_B_UUID, bytes([odr_idx]), response=True)
    print(f"[OK] Sampling frequency → {ODR_MAP[odr_idx]} Hz (index={odr_idx})")

async def read_sampling_freq(client: BleakClient):
    data = await client.read_gatt_char(CHAR_B_UUID)
    odr_idx = data[0]
    print(f"[INFO] Current sampling frequency: {ODR_MAP.get(odr_idx,'?')} Hz (index={odr_idx})")
    return odr_idx

# ---------------------------------------------------------------------------
# Interactive command loop
# ---------------------------------------------------------------------------
HELP = (
    "\nCommands:\n"
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
        try:
            cmd = await loop.run_in_executor(None, input, "cmd> ")
        except EOFError:
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
            pass
        else:
            print(f"[WARN] Unknown command '{cmd}'. Type 'h' for help.")

# ---------------------------------------------------------------------------
# Client session
# ---------------------------------------------------------------------------
async def run_client(client: BleakClient):
    print(f"Connected: {client.is_connected}")

    # Verify custom service
    service_found = any(
        s.uuid.lower() == ACC_SERVICE_UUID for s in client.services
    )
    if not service_found:
        print("[ERROR] AccSensor service not found.")
        return

    print("[OK] AccSensor GATT service found.")

    # Read current ODR
    await read_sampling_freq(client)

    # Subscribe to Char_A (accel data)
    await client.start_notify(CHAR_A_UUID, accel_notification_handler)
    print("[OK] Subscribed to acceleration notifications (Char_A).")

    # Subscribe to Char_C (motion event)
    await client.start_notify(CHAR_C_UUID, motion_notification_handler)
    print("[OK] Subscribed to significant motion notifications (Char_C).")
    print("     Move the board to trigger a significant motion event.\n")

    # Interactive loop
    await command_loop(client)

    # Unsubscribe cleanly
    await client.stop_notify(CHAR_A_UUID)
    await client.stop_notify(CHAR_C_UUID)
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