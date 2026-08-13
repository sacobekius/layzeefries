#include "bleserial.h"
#include <NimBLEDevice.h>

BleSerial bleSerial;

// Nordic UART Service UUIDs
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class BleSerialServerCallbacks : public NimBLEServerCallbacks {
public:
    explicit BleSerialServerCallbacks(BleSerial *eigenaar) : _eigenaar(eigenaar) {}

    void onConnect(NimBLEServer *server, NimBLEConnInfo &connInfo) override {
        _eigenaar->_setVerbonden(true);
    }

    void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connInfo, int reason) override {
        _eigenaar->_setVerbonden(false);
        _eigenaar->_setGeabonneerd(false);  // volgende client moet opnieuw abonneren
        server->getAdvertising()->start();  // weer adverteren voor een volgende verbinding
    }

private:
    BleSerial *_eigenaar;
};

class BleSerialRxCallbacks : public NimBLECharacteristicCallbacks {
public:
    explicit BleSerialRxCallbacks(BleSerial *eigenaar) : _eigenaar(eigenaar) {}

    void onWrite(NimBLECharacteristic *kenmerk, NimBLEConnInfo &connInfo) override {
        NimBLEAttValue waarde = kenmerk->getValue();
        if (waarde.length() > 0)
            _eigenaar->_ontvangData(waarde.data(), waarde.length());
    }

private:
    BleSerial *_eigenaar;
};

class BleSerialTxCallbacks : public NimBLECharacteristicCallbacks {
public:
    explicit BleSerialTxCallbacks(BleSerial *eigenaar) : _eigenaar(eigenaar) {}

    void onSubscribe(NimBLECharacteristic *kenmerk, NimBLEConnInfo &connInfo, uint16_t subValue) override {
        _eigenaar->_setGeabonneerd(subValue != 0);
    }

private:
    BleSerial *_eigenaar;
};

void BleSerial::begin(const char *deviceNaam)
{
    NimBLEDevice::init(deviceNaam);
    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(new BleSerialServerCallbacks(this));

    NimBLEService *service = server->createService(NUS_SERVICE_UUID);

    // NimBLE voegt de CCCD (notify-descriptor) automatisch toe, geen BLE2902 nodig.
    auto *tx = service->createCharacteristic(NUS_TX_UUID, NIMBLE_PROPERTY::NOTIFY);
    tx->setCallbacks(new BleSerialTxCallbacks(this));

    auto *rx = service->createCharacteristic(NUS_RX_UUID, NIMBLE_PROPERTY::WRITE);
    rx->setCallbacks(new BleSerialRxCallbacks(this));

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(NUS_SERVICE_UUID);
    advertising->start();

    _txKenmerk = tx;
}

size_t BleSerial::write(uint8_t c)
{
    return write(&c, 1);
}

size_t BleSerial::write(const uint8_t *buffer, size_t size)
{
    // Altijd bufferen, ook vóór bleSerial.begin() en zonder (nog) verbonden
    // client: schrijven raakt alleen _txBuf aan (gewoon een array-lid, altijd
    // aanwezig) en heeft _txKenmerk niet nodig — dat is enkel voor tick(), die
    // pas echt verstuurt zodra dat geldig is. Zo blijft het begin van de trace
    // (WiFi-verbinden, sensordetectie, ...) altijd bewaard, ongeacht volgorde
    // in setup(), totdat iemand verbindt. write() (en dus console.print(),
    // overal in de code) blijft daarbij instant, zonder blocking delay().
    for (size_t i = 0; i < size; i++) {
        size_t volgendeKop = (_txKop + 1) % TXBUF_GROOTTE;
        if (volgendeKop == _txStaart)
            break;  // buffer vol, rest van dit schrijfverzoek wordt genegeerd
        _txBuf[_txKop] = buffer[i];
        _txKop = volgendeKop;
    }
    return size;
}

void BleSerial::tick()
{
    if (!_geabonneerd || _txKenmerk == nullptr || _txKop == _txStaart)
        return;  // (nog) niet geabonneerd op notificaties, of niets te versturen

    unsigned long now = millis();
    if (now < _volgendeVerzendMoment)
        return;  // nog even wachten op het verbindingsinterval, geen delay() nodig

    uint8_t chunk[NOTIFY_CHUNK];
    size_t lengte = 0;
    while (lengte < NOTIFY_CHUNK && _txStaart != _txKop) {
        chunk[lengte++] = _txBuf[_txStaart];
        _txStaart = (_txStaart + 1) % TXBUF_GROOTTE;
    }

    auto *tx = (NimBLECharacteristic *) _txKenmerk;
    tx->setValue(chunk, lengte);
    tx->notify();
    _volgendeVerzendMoment = now + NOTIFY_INTERVAL_MS;
}

int BleSerial::available()
{
    return (int) ((_rxKop - _rxStaart + RXBUF_GROOTTE) % RXBUF_GROOTTE);
}

int BleSerial::read()
{
    if (_rxKop == _rxStaart)
        return -1;
    uint8_t c = _rxBuf[_rxStaart];
    _rxStaart = (_rxStaart + 1) % RXBUF_GROOTTE;
    return c;
}

int BleSerial::peek()
{
    if (_rxKop == _rxStaart)
        return -1;
    return _rxBuf[_rxStaart];
}

void BleSerial::_setVerbonden(bool verbonden)
{
    _verbonden = verbonden;
}

void BleSerial::_setGeabonneerd(bool geabonneerd)
{
    _geabonneerd = geabonneerd;
}

void BleSerial::_ontvangData(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        size_t volgendeKop = (_rxKop + 1) % RXBUF_GROOTTE;
        if (volgendeKop == _rxStaart)
            break;  // buffer vol, rest van dit pakket wordt genegeerd
        _rxBuf[_rxKop] = data[i];
        _rxKop = volgendeKop;
    }
}
