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
    # Niet via loop.run_in_executor(None, sys.stdin.readline): dat blocking
    # readline()-aanroep draait in een losse thread die niet op te breken is
    # door een Ctrl+C (dat kan alleen bytecode in de hoofdthread onderbreken)
    # — het programma blijft dan aan die thread hangen tot-ie alsnog een
    # regel/EOF krijgt, en de geforceerde join() bij interpreter-afsluiten
    # gaf de rommelige dubbele traceback. loop.add_reader() legt stdin
    # rechtstreeks in de event loop, geen aparte thread nodig, dus Ctrl+C
    # werkt meteen zoals bij de rest van dit script.
    loop = asyncio.get_event_loop()
    regels: asyncio.Queue = asyncio.Queue()

    def _stdin_gereed():
        regel = sys.stdin.readline()
        if not regel:
            loop.remove_reader(sys.stdin)
        regels.put_nowait(regel)

    loop.add_reader(sys.stdin, _stdin_gereed)
    try:
        while True:
            regel = await regels.get()
            if not regel:
                break
            await client.write_gatt_char(NUS_RX_UUID, regel.encode())
    finally:
        loop.remove_reader(sys.stdin)


async def zoek_device(naam: str, timeout: float = 10.0):
    # BleakScanner.find_device_by_name() bleek de naam soms te missen (macOS
    # CoreBluetooth-backend), ook als het device wel degelijk adverteert.
    # discover() + zelf filteren op naam is betrouwbaarder gebleken.
    devices = await BleakScanner.discover(timeout=timeout)
    return next((d for d in devices if d.name == naam), None)


async def main():
    naam = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_NAAM

    # Buitenste lus: een herstart van het bord (of gewoon BLE dat wegvalt)
    # breekt de verbinding, maar dat was voorheen onzichtbaar voor dit
    # script — geen disconnected_callback, dus stuur_input() bleef gewoon op
    # stdin wachten terwijl er nooit meer een notificatie binnenkwam. Nu:
    # bij een disconnect meteen opnieuw zoeken/verbinden, zonder dat je het
    # script handmatig hoeft te herstarten.
    while True:
        print(f"Zoeken naar BLE-device '{naam}'...")
        device = await zoek_device(naam)
        if device is None:
            print(f"Geen device gevonden met naam '{naam}'. Staat het bord aan en is BLE niet al door iets anders (bv. LightBlue) verbonden?")
            return

        verbroken = asyncio.Event()

        def on_disconnect(_client):
            print("\nVerbinding verbroken (bv. herstart van het bord) — opnieuw zoeken...")
            verbroken.set()

        async with BleakClient(device, disconnected_callback=on_disconnect) as client:
            print(f"Verbonden met {device.address}")
            await client.start_notify(NUS_TX_UUID, on_notify)
            print("Luisteren naar status-output (Ctrl+C om te stoppen)...")

            stuur_taak = asyncio.create_task(stuur_input(client))
            verbroken_taak = asyncio.create_task(verbroken.wait())
            try:
                done, pending = await asyncio.wait(
                    {stuur_taak, verbroken_taak}, return_when=asyncio.FIRST_COMPLETED)
            except KeyboardInterrupt:
                stuur_taak.cancel()
                verbroken_taak.cancel()
                return
            for taak in pending:
                taak.cancel()

            if stuur_taak in done:
                stuur_taak.exception()  # eventuele KeyboardInterrupt hier ophalen, anders "never retrieved"-waarschuwing
                # stuur_input() zelf gestopt (Ctrl+C/EOF op stdin) — dat is
                # een bewuste gebruikersactie, niet opnieuw gaan verbinden.
                if client.is_connected:
                    await client.stop_notify(NUS_TX_UUID)
                return
            # anders: verbroken_taak — terug naar boven voor een nieuwe poging


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        # main() vangt Ctrl+C tijdens het luisteren zelf al af (zie
        # stuur_input()); dit vangt alleen nog een Ctrl+C vóór/na dat punt
        # (bv. tijdens zoek_device()) stil op, zodat asyncio.run() geen
        # traceback naar buiten laat lekken.
        print("\nGestopt.")
