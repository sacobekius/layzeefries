#ifndef LAYZEEFRIES_BLESERIAL_H
#define LAYZEEFRIES_BLESERIAL_H

#include <Arduino.h>

/* Seriele communicatie over een BLE Nordic UART Service (NUS).
 * De Nano ESP32 (ESP32-S3) heeft geen classic Bluetooth/SPP, alleen BLE.
 */
class BleSerial : public Stream {
public:
    void begin(const char *deviceNaam);
    bool isVerbonden() const { return _verbonden; }

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    using Print::write;

    int available() override;
    int read() override;
    int peek() override;

    // Moet elke loop()-iteratie aangeroepen worden: verstuurt gebufferde
    // uitgaande data met een niet-blokkerende rate-limit (i.p.v. delay()).
    void tick();

    // Alleen voor gebruik door de BLE-callbacks, niet extern aanroepen.
    void _setVerbonden(bool verbonden);
    void _setGeabonneerd(bool geabonneerd);
    void _ontvangData(const uint8_t *data, size_t len);

private:
    static constexpr size_t RXBUF_GROOTTE = 128;
    static constexpr size_t TXBUF_GROOTTE = 1024;
    static constexpr size_t NOTIFY_CHUNK = 20;
    static constexpr unsigned long NOTIFY_INTERVAL_MS = 20;

    void *_txKenmerk = nullptr;
    volatile bool _verbonden = false;
    // "verbonden" != "geabonneerd op notificaties": dat laatste gebeurt pas
    // als de client de CCCD schrijft (bv. start_notify()), een moment na de
    // connectie. tick() mag pas draineren als dit echt waar is, anders wordt
    // data "verstuurd" (en dus uit de buffer verwijderd) terwijl er nog
    // niemand luistert.
    volatile bool _geabonneerd = false;

    uint8_t _rxBuf[RXBUF_GROOTTE];
    volatile size_t _rxKop = 0;
    volatile size_t _rxStaart = 0;

    uint8_t _txBuf[TXBUF_GROOTTE];
    size_t _txKop = 0;
    size_t _txStaart = 0;
    unsigned long _volgendeVerzendMoment = 0;
};

extern BleSerial bleSerial;

#endif //LAYZEEFRIES_BLESERIAL_H
