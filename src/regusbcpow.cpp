#include <arduino.h>
#include <regusbcpow.h>
#include "console.h"

// uint8_t (i.p.v. #define, dus een int-literal): op de ESP32 heeft Wire's
// requestFrom() meerdere overloads (uint8_t/uint16_t/int) die met een
// ongetypeerd adres ambigu worden — dat gaf steeds een compiler-warning.
static constexpr uint8_t AP33772S_ADDRESS = 0x52;
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
    // Alleen een vlag zetten: I2C-transacties zijn niet veilig binnen een
    // ISR op ESP32 (Wire gebruikt semaforen, niet ISR-safe). De echte
    // STATUS-uitlezing gebeurt in handleWork(), in de hoofdloop-context.
    interruptFlag++;
}

// Constructor
regUSBCPow::regUSBCPow(TwoWire &wire) : _readBuf{}, _writeBuf{} {
    // "Oneindig" i.p.v. 0: anders is de periodieke AVS-refresh in handleWork()
    // al vanaf de allereerste loop()-iteratie verlopen, ongeacht modus. Alleen
    // _stuurAan() zelf mag dit legitiem vervroegen, en doet dat uitsluitend
    // wanneer de actieve modus daadwerkelijk AVS is.
    _next_avsTick = 0xFFFFFFFF;
    i2cPort = &wire;
}

// Initialisatie
void regUSBCPow::begin(int i)
{
    // INPUT_PULLDOWN i.p.v. INPUT: voorkomt een zwevende pin (en dus
    // interruptstorm) zolang de RotoPD niet aangesloten/gevoed is. PULLUP zou
    // hier verkeerd zijn: de AP33772S-INT-pin is idle-LOW/actief-HIGH
    // (datasheet), dus zonder actieve aansturing (geen VBUS) hoort de pin
    // richting LOW te zakken — met PULLUP zou een ongevoede/losse pin juist
    // HIGH blijven hangen en een level-HIGH-interrupt continu laten afgaan.
    pinMode(i, INPUT_PULLDOWN);
    // De AP33772S-INT-pin is level-triggered en gaat naar HIGH bij een event
    // (datasheet "Interrupt Signal"-sectie) — geen dalende flank (FALLING),
    // dat liet interrupts nooit binnenkomen.
    attachInterrupt(digitalPinToInterrupt(i), handleInterrupt, HIGH);

    reset();  // schone herstart van de PD-onderhandeling bij elke boot

    _writeBuf[0] = PDO_STARTED | PDO_READY | PDO_NEWPDO;
    i2c_write(CMD_MASK, 1);
    _started_at = millis() + 200;
}

void regUSBCPow::reset()
{
    _writeBuf[0] = 0x01;  // PD_CMDMSG.HRST
    i2c_write(CMD_PD_CMDMSG, 1);
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
                15000, // _srcPDOs[i].avs.voltage_min * 200,
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
        if (voltage_mV >= _pdos[_avsPDOIndex-1].voltage_min_mV &&
            voltage_mV <= _pdos[_avsPDOIndex-1].voltage_max_mV) {
            console.println("AVS");
            _huidigPDOIndex = _avsPDOIndex;
        }
    }
    if (_huidigPDOIndex < 0 && _ppsPDOIndex > 0) {
        if (voltage_mV >= _pdos[_ppsPDOIndex-1].voltage_min_mV &&
            voltage_mV <= _pdos[_ppsPDOIndex-1].voltage_max_mV) {
            console.println("PPS");
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
    // VREQ (0x14) is een 2-byte register (datasheet Table 19) — voorheen
    // werd hier maar 1 byte gelezen, dus alleen de lage byte.
    i2c_read(CMD_VREQ, 2);
    return ((unsigned int)_readBuf[1] << 8 | _readBuf[0]) * 50;  // 50mV/LSB
}

unsigned int regUSBCPow::leesIREQ() {
    i2c_read(CMD_IREQ, 2);
    return ((unsigned int)_readBuf[1] << 8 | _readBuf[0]) * 10;  // 10mA/LSB
}

int regUSBCPow::leesTemp() {
    i2c_read(CMD_TEMP, 1);
    return (int)_readBuf[0];  // 1°C/LSB
}

unsigned int regUSBCPow::leesMinVoltage() const {
    if (_ppsPDOIndex > 0) return _pdos[_ppsPDOIndex - 1].voltage_min_mV;
    if (_avsPDOIndex > 0) return _pdos[_avsPDOIndex - 1].voltage_min_mV;
    return 0;
}

unsigned int regUSBCPow::leesMaxVoltage() const {
    if (_avsPDOIndex > 0) return _pdos[_avsPDOIndex - 1].voltage_max_mV;
    if (_ppsPDOIndex > 0) return _pdos[_ppsPDOIndex - 1].voltage_max_mV;
    return 0;
}

// Beveiligingsdrempels
unsigned int regUSBCPow::leesVSELMIN() {
    i2c_read(CMD_VSELMIN, 1);
    return (unsigned int)_readBuf[0] * 200;  // 200mV/LSB
}

void regUSBCPow::stelVSELMIN(unsigned int voltage_mV) {
    _writeBuf[0] = (uint8_t) (voltage_mV / 200);
    i2c_write(CMD_VSELMIN, 1);
}

int regUSBCPow::leesUVPPercentage() {
    i2c_read(CMD_UVPTHR, 1);
    switch (_readBuf[0]) {
        case 1: return 80;
        case 2: return 75;
        case 3: return 70;
        default: return -1;
    }
}

void regUSBCPow::stelUVPPercentage(int percentage) {
    switch (percentage) {
        case 80: _writeBuf[0] = 1; break;
        case 75: _writeBuf[0] = 2; break;
        case 70: _writeBuf[0] = 3; break;
        default: return;  // alleen 70/75/80 geldig
    }
    i2c_write(CMD_UVPTHR, 1);
}

unsigned int regUSBCPow::leesOVPTHR() {
    i2c_read(CMD_OVPTHR, 1);
    return (unsigned int)_readBuf[0] * 80;  // 80mV/LSB, offset boven VREQ
}

void regUSBCPow::stelOVPTHR(unsigned int offset_mV) {
    _writeBuf[0] = (uint8_t) (offset_mV / 80);
    i2c_write(CMD_OVPTHR, 1);
}

unsigned int regUSBCPow::leesOCPTHR() {
    i2c_read(CMD_OCPTHR, 1);
    return (unsigned int)_readBuf[0] * 50;  // 50mA/LSB
}

void regUSBCPow::stelOCPTHR(unsigned int current_mA) {
    _writeBuf[0] = (uint8_t) (current_mA / 50);
    i2c_write(CMD_OCPTHR, 1);
}

int regUSBCPow::leesOTPTHR() {
    i2c_read(CMD_OTPTHR, 1);
    return (int)_readBuf[0];  // 1°C/LSB
}

void regUSBCPow::stelOTPTHR(int temp_C) {
    _writeBuf[0] = (uint8_t) temp_C;
    i2c_write(CMD_OTPTHR, 1);
}

int regUSBCPow::leesDRTHR() {
    i2c_read(CMD_DRTHR, 1);
    return (int)_readBuf[0];  // 1°C/LSB
}

void regUSBCPow::stelDRTHR(int temp_C) {
    _writeBuf[0] = (uint8_t) temp_C;
    i2c_write(CMD_DRTHR, 1);
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

void regUSBCPow::printStatus(Print &p) {
    unsigned int huidig_stroom = leesStroom();
    unsigned int huidig_voltage = leesVoltage();
    unsigned int gevraagde_voltage = leesVREQ();
    unsigned int gevraagde_stroom = leesIREQ();
    p.print("Huidig: ");
    p.print(huidig_stroom);
    p.print("mA\t");
    p.print(huidig_voltage);
    p.print("mV (gevraagd: ");
    p.print(gevraagde_voltage);
    p.print("mV, ");
    p.print(gevraagde_stroom);
    p.println("mA)");
}

RotoPdStatus regUSBCPow::handleWork()
{
    unsigned long now = millis();

    if (interruptFlag > 0)
    {
        interruptFlag = 0;
        // STATUS-uitlezing gebeurt hier (hoofdloop-context), niet meer in de
        // ISR — lezen reset het register (datasheet: "Reset to 0 after every
        // Read"), dus dit is de enige plek waar we het mogen lezen.
        i2c_read(CMD_STATUS, 1);
        if (_trans_stat != 0)
        {
            // Geen contact met de RotoPD (bv. kabel eruit/geen VBUS) — geen
            // bevestigd werkende PDO, dus fail-safe terug naar GEEN_PDO.
            _status = RotoPdStatus::GEEN_PDO;
        }
        else
        {
            interruptStatus = _readBuf[0];
            // Geen trace hier: dit vuurt elke 500ms door de AVS-keepalive
            // (_stuurAan() hieronder) en vervuilde de PID-observatie zonder
            // nieuwe informatie — de interessante gevallen (fout-bits, nieuwe
            // PDO-lijst) worden hieronder al apart afgehandeld/geprint.
            if (interruptStatus & (PDO_UVP | PDO_OVP | PDO_OCP | PDO_OTP))
            {
                // Beveiliging getriggerd: chip schakelt VOUT uit. Datasheet:
                // "the host MCU will need to load new PD_REQMSG to start a
                // PDO negotiation process to resume" — dat gebeurt bij het
                // herstel (zie check_mode()'s "pop"), niet hier.
                _status = RotoPdStatus::PDO_OVERFLOW;
            }
            else if ((interruptStatus & PDO_READY) && _status == RotoPdStatus::PDO_OVERFLOW)
            {
                // Schoon READY-signaal zonder foutbits: hersteld van de
                // beveiligingstrip.
                _status = RotoPdStatus::PDO_OK;
            }
            if (interruptStatus & PDO_NEWPDO)
                _newPdo = true;
            if (interruptStatus & PDO_READY)
                _ready = true;
            if (interruptStatus & PDO_STARTED)
                _started_at = now + 100;
            _writeBuf[0] = PDO_STARTED | PDO_READY | PDO_NEWPDO;
            i2c_write(CMD_MASK, 1);
        }
    }
    // Een vers NEWPDO-signaal betekent altijd: PDO-lijst opnieuw inlezen —
    // ongeacht wat _status daarvoor toevallig was. Niet vastklinken aan
    // "_status was GEEN_PDO": een reconnect triggert niet altijd (opnieuw)
    // een UVP/OVP/OCP/OTP-bit of I2C-fout onderweg (gezien op echte hardware:
    // I2C bleef werken, interruptStatus was zelfs 0x0 tijdens het loskoppelen),
    // dus _status kan best op PDO_OK zijn blijven staan terwijl er ondertussen
    // wél een volledige her-onderhandeling plaatsvond.
    if (_ready && _newPdo && now > _started_at)
    {
        srcpdo();
        printTo(console);
        // EPR-PDO's (de 28V-opties) kunnen iets later pas klaar zijn dan de
        // basis SPR-onderhandeling. Alleen over 1s nog eens kijken als AVS er
        // nu nog niet bij zat — anders lezen/printen we de lijst nodeloos
        // twee keer bij elke normale boot.
        if (_avsPDOIndex <= 0)
            _herlees_pdo_at = now + 1000;
        _status = RotoPdStatus::PDO_OK;  // een verse PDO-lijst is per definitie weer OK
    }
    if (now > _herlees_pdo_at)
    {
        _herlees_pdo_at = 0xFFFFFFFF;  // eenmalig
        srcpdo();
        printTo(console);
    }
    if (now > _next_avsTick)
    {
        _next_avsTick = now + 500;
        _stuurAan();
    }
    // Verificatie: is de laatst aangevraagde spanning ook echt gehonoreerd?
    // De periodieke AVS-refresh hierboven (elke 500ms _stuurAan() herhalen)
    // loste dat niet altijd op — real-hardware logs lieten een aanvraag zien
    // die minutenlang op de bron z'n vorige/standaard contract bleef hangen.
    // Bij aanhoudende mismatch (~10s) forceert een PD-reset een schone
    // heronderhandeling; de eerstvolgende aanroep van buitenaf (setVoltage())
    // vraagt daarna vanzelf opnieuw aan, geen aparte hersteldelogica nodig.
    if (_huidigPDOIndex > 0 && now > _next_verify_at)
    {
        _next_verify_at = now + 2000;
        unsigned int vreq = leesVREQ();
        int verschil = (int) vreq - (int) _huidigVoltage_mV;
        if (verschil < 0) verschil = -verschil;
        if (verschil > 200)
        {
            _verify_mismatch_teller++;
            if (_verify_mismatch_teller >= 5)
            {
                console.println("PD-aanvraag blijft afwijken van wat gevraagd is — forceer PD-reset");
                reset();
                _verify_mismatch_teller = 0;
            }
        }
        else
        {
            _verify_mismatch_teller = 0;
        }
    }
    return _status;
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
    } else
    {
        _next_avsTick = 0xFFFFFFFF;
    }
    _writeBuf[0] = rdo.byte0;
    _writeBuf[1] = rdo.byte1;
    i2c_write(CMD_PD_REQMSG, 2);
    return true;
}

// AP33772S PD_REQMSG.CURRENT_SEL / SRC_PDO.CURRENT_MAX codering (datasheet
// Table 2, EVB guide "SRC_SPR_PDO" tabel). De buckets zijn niet allemaal
// even breed: 0 is 0.00-1.24A, 1 t/m 13 zijn telkens 250mA-stappen vanaf
// 1250mA, 14 is dubbel breed (4500-4999mA), en 15 is de open "5.00A of
// meer"-sentinel (ook gebruikt om expliciet de maximale stroom op te vragen).
unsigned int regUSBCPow::_currentMap(unsigned int current_mA) {
    if (current_mA < 1250) return 0;
    if (current_mA < 4500) return ((current_mA - 1250) / 250) + 1;
    if (current_mA < 5000) return 14;
    return 15;  // zonder deze afkapping zou >=5000mA de 4-bits CURRENT_SEL laten overlopen
}

int regUSBCPow::_currentMapInverse(int waarde) {
    if (waarde <= 0) return 1000;
    if (waarde >= 15) return 5000;  // "5.00A of meer" — geen losse 250mA-stap
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
        console.print("I2C read failed: 0x");
        console.println(_trans_stat, HEX);
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
        console.print("I2C write failed: 0x");
        console.println(_trans_stat, HEX);
    }

    // clear readBuffer
    memset(_readBuf, 0, sizeof(_readBuf));
}

