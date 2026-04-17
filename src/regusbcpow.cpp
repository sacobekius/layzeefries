#include <arduino.h>
#include <regusbcpow.h>

#define AP33772S_ADDRESS 0x52
#define READ_BUFF_LENGTH 128
#define WRITE_BUFF_LENGTH 6
#define SRCPDO_LENGTH 28

void PDOInfo::printTo(Print &p) const {
    switch (type) {
        case PDO_LEEG:
            p.print("Leeg");
            break;
        case PDO_FIXED:
            p.print("Fixed: ");
            p.print(voltage_mV);
            p.print("mV ");
            p.print(current_max_mA);
            p.print("mA");
            break;
        case PDO_PPS:
            p.print("PPS: ");
            p.print(voltage_min_mV);
            p.print("-");
            p.print(voltage_max_mV);
            p.print("mV ");
            p.print(current_max_mA);
            p.print("mA");
            break;
        case PDO_AVS:
            p.print("AVS: ");
            p.print(voltage_min_mV);
            p.print("-");
            p.print(voltage_max_mV);
            p.print("mV ");
            p.print(current_max_mA);
            p.print("mA");
            break;
    }
}

// Interrupt
volatile int interruptFlag = 0;
volatile uint8_t interruptStatus = 0;
TwoWire *i2cPort{};

static void handleInterrupt()
{
    i2cPort->beginTransmission(AP33772S_ADDRESS);    // transmit to device SLAVE_ADDRESS
    i2cPort->write(0x01);                         // sets the CMD_STATUS register
    i2cPort->endTransmission();                     // stop transmitting
    i2cPort->requestFrom(AP33772S_ADDRESS, 1);      // request 1 bytes from peripheral device
    if (i2cPort->available())
        interruptStatus = (byte)i2cPort->read();
    interruptFlag++;
}

// Constructor
regUSBCPow::regUSBCPow(TwoWire &wire) : _readBuf{}, _writeBuf{} {
    _next_avsTick = 0;
    i2cPort = &wire;
}

// Initialisatie
void regUSBCPow::begin(int i)
{
    pinMode(i, INPUT);
    attachInterrupt(digitalPinToInterrupt(i), handleInterrupt, FALLING);
    _writeBuf[0] = PDO_STARTED | PDO_READY | PDO_NEWPDO;
    i2c_write(CMD_MASK, 1);
    _started_at = millis() + 200;
}

void regUSBCPow::srcpdo()
{
    for (auto & _pdo : _pdos) {
        _pdo = PDOInfo();
    }
    _ppsPDOIndex = -1;
    _avsPDOIndex = -1;
    _pdoCount = 0;
    _huidigPDOIndex = -1;
    _huidigVoltage_mV = 5000;
    _huidigStroom_mA = 1000;
    _maxVoltage_mV = 0;
    _maxStroom_mA = 5000;
    _newPdo = false;
    i2c_read(CMD_OPMODE,1);
    if (!_readBuf[0] & 0x02) // PD source?
        return;
    i2c_read(CMD_SRCPDO, 26);

    _pdoCount = 0;
    for (int i = 0; i < 13; i++) {
        _srcPDOs[i].byte0 = _readBuf[i * 2];
        _srcPDOs[i].byte1 = _readBuf[i * 2 + 1];

        if (_srcPDOs[i].fixed.detect == 0) {
            _pdos[i] = PDOInfo();
            continue;
        }

        _pdoCount++;
        bool isEPR = (i >= 7);

        if (_srcPDOs[i].fixed.type == 0) {
            // Fixed PDO
            int voltage = _srcPDOs[i].fixed.voltage_max * (isEPR ? 200 : 100);
            int current = _currentMapInverse(_srcPDOs[i].fixed.current_max);
            _pdos[i] = PDOInfo(voltage, current);
        } else if (isEPR) {
            // AVS
            int current = _currentMapInverse(_srcPDOs[i].avs.current_max);
            _pdos[i] = PDOInfo(
                PDO_AVS,
                _srcPDOs[i].avs.voltage_min * 200,
                _srcPDOs[i].avs.voltage_max * 200,
                current
            );
            _avsPDOIndex = i + 1;
        } else {
            // PPS
            int current = _currentMapInverse(_srcPDOs[i].pps.current_max);
            _pdos[i] = PDOInfo(
                PDO_PPS,
                _srcPDOs[i].pps.voltage_min * 100,
                _srcPDOs[i].pps.voltage_max * 100,
                current
            );
            _ppsPDOIndex = i + 1;
        }
    }
}

// Spanning instellen
bool regUSBCPow::setVoltage(unsigned int voltage_mV, unsigned int current_mA) {

    _huidigPDOIndex = -1;
    if (_avsPDOIndex > 0) {
        if (voltage_mV >= 15000 &&
            voltage_mV <= _pdos[_avsPDOIndex-1].voltage_max_mV) {
            Serial.println("AVS");
            _huidigPDOIndex = _avsPDOIndex;
        }
    }
    if (_huidigPDOIndex < 0 && _ppsPDOIndex > 0) {
        if (voltage_mV >= _pdos[_ppsPDOIndex-1].voltage_min_mV &&
            voltage_mV <= _pdos[_ppsPDOIndex-1].voltage_max_mV) {
            Serial.println("PPS");
            _huidigPDOIndex = _ppsPDOIndex;
        }
    }
    if (_huidigPDOIndex < 0) {
        return false;  // geen geschikt protocol voor gevraagd voltage
    }

    // Afronding
    const int stap = (_pdos[_huidigPDOIndex].type == PDO_AVS) ? 200 : 100;
    _huidigVoltage_mV = (voltage_mV / stap) * stap;

    // Stroom binnen de grenzen (stroom == 0 is niet handig)
    _huidigStroom_mA = current_mA;
    if (_huidigStroom_mA > _pdos[_huidigPDOIndex-1].current_max_mA)
        _huidigStroom_mA = _pdos[_huidigPDOIndex-1].current_max_mA;

    return _stuurAan();
}

// Stroom instellen
bool regUSBCPow::setStroom(unsigned int current_mA) {
    _huidigStroom_mA = current_mA;
    return _stuurAan();
}

// Uitgang
bool regUSBCPow::outputAan() {
    _writeBuf[0] = 0b00010010;
    i2c_write(CMD_SYSTEM, 1);
    return true;
}

bool regUSBCPow::outputUit() {
    _writeBuf[0] = 0b00010001;
    i2c_write(CMD_SYSTEM, 1);
    return true;
}

// Meten
unsigned int regUSBCPow::leesVoltage() {
    i2c_read(CMD_VOLTAGE, 2);
    return ((unsigned int)_readBuf[1] << 8 | _readBuf[0]) * 80;  // 80mV/LSB
}

unsigned int regUSBCPow::leesStroom() {
    i2c_read(CMD_CURRENT, 1);
    return (unsigned int)_readBuf[0] * 24;  // 24mA/LSB
}

unsigned int regUSBCPow::leesVREQ() {
    i2c_read(CMD_VREQ, 1);
    return (unsigned int)_readBuf[0] * 50;  // 50mV/LSB — overflow fix via cast naar int
}

int regUSBCPow::leesTemp() {
    i2c_read(CMD_TEMP, 1);
    return (int)_readBuf[0];  // 1°C/LSB
}

// Status
bool regUSBCPow::isKlaar() {
    i2c_read(CMD_STATUS, 1);
    return (_readBuf[0] & 0x02) != 0;  // READY bit
}

void regUSBCPow::printTo(Print &p) const {
    if (_trans_stat != 0)
    {
        p.print("RotoPD communicatiefout, status: 0x");
        p.println(_trans_stat, HEX);
    }
    p.println("RotoPD profielen:");
    for (int i = 0; i < 13; i++) {
        if (_pdos[i].type == PDO_LEEG) continue;
        p.print("  PDO");
        p.print(i + 1);
        p.print(": ");
        _pdos[i].printTo(p);
        p.println();
    }
    p.print("PPS index: ");
    p.println(_ppsPDOIndex);
    p.print("AVS index: ");
    p.println(_avsPDOIndex);
}

void regUSBCPow::handleWork()
{
    unsigned long now = millis();

    if (interruptFlag > 0)
    {
        Serial.print("interruptFlag: 0x");
        Serial.println(interruptFlag, HEX);
        interruptFlag = 0;
        Serial.print("interruptStatus: 0x");
        Serial.println(interruptStatus, HEX);
        if (interruptStatus & PDO_NEWPDO)
            _newPdo = true;
        if (interruptStatus & PDO_READY)
            _ready = true;
        if (interruptStatus & PDO_STARTED)
            _started_at = now + 100;
        _writeBuf[0] = PDO_STARTED | PDO_READY | PDO_NEWPDO;
        i2c_write(CMD_MASK, 1);
    }
    if (now > _started_at && _ready && _newPdo)
    {
        srcpdo();
        printTo(Serial);
    }
    if (now > _next_avsTick)
    {
        _next_avsTick = now + 500;
        _stuurAan();
    }
}

// Private helpers
bool regUSBCPow::_stuurAan() {
    if (_huidigPDOIndex < 0)
        return false;

    int huidigeModus = _pdos[_huidigPDOIndex-1].type;
    RDO_DATA_T rdo;
    rdo.data = 0;
    rdo.REQMSG_Fields.PDO_INDEX = _huidigPDOIndex;
    rdo.REQMSG_Fields.CURRENT_SEL = _currentMap(_huidigStroom_mA);

    if (huidigeModus == PDO_PPS || huidigeModus == PDO_AVS) {
        int stap = (huidigeModus == PDO_AVS) ? 200 : 100;
        rdo.REQMSG_Fields.VOLTAGE_SEL = _huidigVoltage_mV / stap;
    }
    if (huidigeModus == PDO_AVS)
    {
        _next_avsTick = millis() + 500;
    }
    _writeBuf[0] = rdo.byte0;
    _writeBuf[1] = rdo.byte1;
    i2c_write(CMD_PD_REQMSG, 2);
    return true;
}

int regUSBCPow::_currentMap(int current_mA) {
    if (current_mA < 1250) return 0;
    return ((current_mA - 1250) / 250) + 1;
}

int regUSBCPow::_currentMapInverse(int waarde) {
    if (waarde == 0) return 1000;
    return 1250 + (waarde - 1) * 250;
}

void regUSBCPow::i2c_read(byte cmdAddr, byte len)
{
    // clear readBuffer
    memset(_readBuf, 0, sizeof(_readBuf));
    i2cPort->beginTransmission(AP33772S_ADDRESS);    // transmit to device SLAVE_ADDRESS
    i2cPort->write(cmdAddr);                         // sets the command register
    _trans_stat = i2cPort->endTransmission();        // stop transmitting
    if (_trans_stat != 0)
    {
        Serial.print("I2C read failed: 0x");
        Serial.println(_trans_stat, HEX);
    }

    i2cPort->requestFrom(AP33772S_ADDRESS, len);      // request len bytes from peripheral device
    if (len <= i2cPort->available())
    {
        byte i = 0;
        // if len bytes were received
        while (i2cPort->available())
        {
            _readBuf[i] = (byte)i2cPort->read();
            i++;
        }
    }
}

void regUSBCPow::i2c_write(byte cmdAddr, byte len)
{
    i2cPort->beginTransmission(AP33772S_ADDRESS);     // transmit to device SLAVE_ADDRESS
    i2cPort->write(cmdAddr);                          // sets the command register
    i2cPort->write(_writeBuf, len);                   // write data with len
    _trans_stat = i2cPort->endTransmission();        // stop transmitting
    if (_trans_stat != 0)
    {
        Serial.print("I2C write failed: 0x");
        Serial.println(_trans_stat, HEX);
    }

    // clear readBuffer
    memset(_readBuf, 0, sizeof(_readBuf));
}

