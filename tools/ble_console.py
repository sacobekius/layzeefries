#!/usr/bin/env python3
"""BLE UART (Nordic UART Service) terminal voor de layzeefries Nano ESP32.

Installeren:
    pip install bleak

Gebruik:
    python3 ble_console.py [device-naam]

Toont doorlopend de console-output (heartbeat, temperatuur, modus, ...) die
de firmware over BLE stuurt. Typen + Enter stuurt de regel naar het bord
(de firmware doet er momenteel nog niets mee, maar de kanalen bestaan al).
"""
import asyncio
import sys

from bleak import BleakClient, BleakScanner

NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # schrijven naar het bord
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notificaties vanaf het bord

DEFAULT_NAAM = "layzeefries"


def on_notify(_, data: bytearray):
    sys.stdout.write(data.decode(errors="replace"))
    sys.stdout.flush()


async def stuur_input(client: BleakClient):
    loop = asyncio.get_event_loop()
    while True:
        regel = await loop.run_in_executor(None, sys.stdin.readline)
        if not regel:
            break
        await client.write_gatt_char(NUS_RX_UUID, regel.encode())


async def zoek_device(naam: str, timeout: float = 10.0):
    # BleakScanner.find_device_by_name() bleek de naam soms te missen (macOS
    # CoreBluetooth-backend), ook als het device wel degelijk adverteert.
    # discover() + zelf filteren op naam is betrouwbaarder gebleken.
    devices = await BleakScanner.discover(timeout=timeout)
    return next((d for d in devices if d.name == naam), None)


async def main():
    naam = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_NAAM
    print(f"Zoeken naar BLE-device '{naam}'...")
    device = await zoek_device(naam)
    if device is None:
        print(f"Geen device gevonden met naam '{naam}'. Staat het bord aan en is BLE niet al door iets anders (bv. LightBlue) verbonden?")
        return

    async with BleakClient(device) as client:
        print(f"Verbonden met {device.address}")
        await client.start_notify(NUS_TX_UUID, on_notify)
        print("Luisteren naar status-output (Ctrl+C om te stoppen)...")
        try:
            await stuur_input(client)
        except KeyboardInterrupt:
            pass
        await client.stop_notify(NUS_TX_UUID)


if __name__ == "__main__":
    asyncio.run(main())
