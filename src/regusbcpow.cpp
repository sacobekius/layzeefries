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

// Constructor
regUSBCPow::regUSBCPow(TwoWire &wire) : _readBuf{}, _writeBuf{} {
    _wire = &wire;
    _ppsPDOIndex = -1;
    _avsPDOIndex = -1;
    _pdoCount = 0;
    _huidigPDOIndex = -1;
    _huidigVoltage_mV = 0;
    _huidigStroom_mA = 0;
    _maxVoltage_mV = 0;
    _maxStroom_mA = 5000;
    _huidigeModus = PDO_LEEG;
    _interruptPin = -1;
    _interruptFlag = false;
    _interruptStatus = 0;

    for (int i = 0; i < 13; i++) {
        _pdos[i] = PDOInfo();
    }
}

// Initialisatie
void regUSBCPow::begin() {
    delay(100);
    i2c_read(CMD_SRCPDO, 26);
    
    _pdoCount = 0;
    for (int i = 0; i < 13; i++) {
        uint8_t byte0 = _readBuf[i * 2];
        uint8_t byte1 = _readBuf[i * 2 + 1];
        
        if (byte0 == 0 && byte1 == 0) {
            _pdos[i].type = PDO_LEEG;
            continue;
        }
        
        _pdoCount++;
        bool isEPR = (i >= 7);  // index 7-12 zijn EPR
        uint8_t type = (byte1 >> 4) & 0x03;
        
        if (type == 0) {
            // Fixed PDO
            _pdos[i].type = PDO_FIXED;
            _pdos[i].voltage_mV = ((byte1 & 0x0F) << 6 | byte0 >> 2) 
                                  * (isEPR ? 200 : 100);
            _pdos[i].voltage_min_mV = 0;
            _pdos[i].voltage_max_mV = _pdos[i].voltage_mV;
            _pdos[i].current_max_mA = _currentMapInverse(byte0 & 0x0F);
        } else {
            // PPS of AVS
            if (isEPR) {
                _pdos[i].type = PDO_AVS;
                _pdos[i].voltage_min_mV = 15000;  // AVS minimum
                _pdos[i].voltage_max_mV = ((byte1 & 0x0F) << 6 | byte0 >> 2) * 200;
                _pdos[i].voltage_mV = _pdos[i].voltage_max_mV;
                _pdos[i].current_max_mA = _currentMapInverse(byte0 & 0x0F);
                _avsPDOIndex = i + 1;  // 1-based
            } else {
                _pdos[i].type = PDO_PPS;
                _pdos[i].voltage_min_mV = 3300;   // PPS minimum
                _pdos[i].voltage_max_mV = ((byte1 & 0x0F) << 6 | byte0 >> 2) * 100;
                _pdos[i].voltage_mV = _pdos[i].voltage_max_mV;
                _pdos[i].current_max_mA = _currentMapInverse(byte0 & 0x0F);
                _ppsPDOIndex = i + 1;  // 1-based
            }
        }
    }
}

// Spanning instellen
bool regUSBCPow::setVoltage(int voltage_mV) {

    if (voltage_mV >= 15000 && _avsPDOIndex > 0) {
        _huidigeModus = PDO_AVS;
        _huidigPDOIndex = _avsPDOIndex;
    } else if (_ppsPDOIndex > 0) {
        _huidigeModus = PDO_PPS;
        _huidigPDOIndex = _ppsPDOIndex;
    } else {
        return false;  // geen geschikt protocol
    }

    // Grenzen bewaken
    PDOInfo &pdo = _pdos[_huidigPDOIndex - 1];
    if (voltage_mV < pdo.voltage_min_mV) voltage_mV = pdo.voltage_min_mV;
    if (voltage_mV > pdo.voltage_max_mV) voltage_mV = pdo.voltage_max_mV;

    // Afronding
    _huidigVoltage_mV = _snapVoltage(voltage_mV);

    return _stuurAan();
}

// Stroom instellen
bool regUSBCPow::setStroom(int current_mA) {
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
int regUSBCPow::leesVoltage() {
    i2c_read(CMD_VOLTAGE, 2);
    return ((int)_readBuf[1] << 8 | _readBuf[0]) * 80;  // 80mV/LSB
}

int regUSBCPow::leesStroom() {
    i2c_read(CMD_CURRENT, 1);
    return (int)_readBuf[0] * 24;  // 24mA/LSB
}

int regUSBCPow::leesVREQ() {
    i2c_read(CMD_VREQ, 1);
    return (int)_readBuf[0] * 50;  // 50mV/LSB — overflow fix via cast naar int
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

// Interrupt
void regUSBCPow::setInterruptPin(int pin) {
    _interruptPin = pin;
    pinMode(pin, INPUT);
}

void regUSBCPow::handleInterrupt() {
    // Lees interrupt status register
    i2c_read(CMD_INTSTATUS, 1);
    _interruptStatus = _readBuf[0];
    _interruptFlag = true;
}

// Private helpers
bool regUSBCPow::_stuurAan() {
    if (_huidigeModus == PDO_LEEG || _huidigPDOIndex < 0) return false;
    
    // RDO opbouwen
    uint16_t rdo = 0;
    rdo |= (_huidigPDOIndex & 0x0F) << 12;
    rdo |= (_currentMap(_huidigStroom_mA) & 0x0F) << 8;
    
    if (_huidigeModus == PDO_PPS || _huidigeModus == PDO_AVS) {
        int stap = (_huidigeModus == PDO_AVS) ? 200 : 100;
        rdo |= (_huidigVoltage_mV / stap) & 0xFF;
    }
    
    _writeBuf[0] = rdo & 0xFF;
    _writeBuf[1] = (rdo >> 8) & 0xFF;
    i2c_write(CMD_REQMSG, 2);
    return true;
}

int regUSBCPow::_snapVoltage(int voltage_mV) const {
    int stap = (_huidigeModus == PDO_AVS) ? 200 : 100;
    return (voltage_mV / stap) * stap;
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
    byte i = 0;
    // clear readBuffer
    memset(_readBuf, 0, sizeof(_readBuf));
    Wire.beginTransmission(AP33772S_ADDRESS); // transmit to device SLAVE_ADDRESS
    Wire.write(cmdAddr);             // sets the command register
    Wire.endTransmission();          // stop transmitting

    Wire.requestFrom(AP33772S_ADDRESS, len); // request len bytes from peripheral device
    if (len <= Wire.available())
    { // if len bytes were received
        while (Wire.available())
        {
            _readBuf[i] = (byte)Wire.read();
            i++;
        }
    }
}

void regUSBCPow::i2c_write(byte cmdAddr, byte len)
{
    Wire.beginTransmission(AP33772S_ADDRESS); // transmit to device SLAVE_ADDRESS
    Wire.write(cmdAddr);             // sets the command register
    Wire.write(_writeBuf, len);       // write data with len
    Wire.endTransmission();          // stop transmitting

    // clear readBuffer
    for (byte i = 0; i < WRITE_BUFF_LENGTH; i++)
    {
        _writeBuf[i] = 0;
    }
}

