#ifndef LAYZEEFRIES_REGUSBCPOW_H
#define LAYZEEFRIES_REGUSBCPOW_H

#include <Wire.h>

#define AP33772S_ADDRESS 0x52

enum PDOType {
    PDO_LEEG,
    PDO_FIXED,
    PDO_PPS,
    PDO_AVS
};

class PDOInfo {
public:
    PDOType type;
    int voltage_mV;
    int voltage_min_mV;
    int voltage_max_mV;
    int current_max_mA;

    PDOInfo()
        : type(PDO_LEEG)
        , voltage_mV(0)
        , voltage_min_mV(0)
        , voltage_max_mV(0)
        , current_max_mA(0)
    {}

    PDOInfo(int voltage, int current)
        : type(PDO_FIXED)
        , voltage_mV(voltage)
        , voltage_min_mV(voltage)
        , voltage_max_mV(voltage)
        , current_max_mA(current)
    {}

    PDOInfo(PDOType t, int voltage_min, int voltage_max, int current)
        : type(t)
        , voltage_mV(voltage_max)
        , voltage_min_mV(voltage_min)
        , voltage_max_mV(voltage_max)
        , current_max_mA(current)
    {}

    void printTo(Print &p) const;
};

class regUSBCPow {
public:
    explicit regUSBCPow(TwoWire &wire = Wire);

    // Initialisatie
    void begin();

    // Instellen — schrijven naar AP33772S
    bool setVoltage(int voltage_mV);
    bool setStroom(int current_mA);

    // Meten — lezen van AP33772S registers
    int leesVoltage();              // mV, 80mV/LSB
    int leesStroom();               // mA, 24mA/LSB
    int leesVREQ();                 // mV, overflow fix
    int leesTemp();                 // °C

    // Uitgang
    bool outputAan();
    bool outputUit();

    // Interrupt configuratie
    void setInterruptPin(int pin);
    void handleInterrupt();         // aanroepen vanuit ISR

    // Status
    bool isKlaar();
    void printTo(Print &p) const;
private:
    TwoWire *_wire;
    // Geïnitialiseerd bij declaratie
    int _ppsPDOIndex = -1;
    int _avsPDOIndex = -1;
    int _pdoCount = 0;
    int _huidigPDOIndex = -1;
    int _huidigVoltage_mV = 0;
    int _huidigStroom_mA = 0;
    int _maxVoltage_mV = 0;
    int _maxStroom_mA = 5000;

    PDOType _huidigeModus = PDO_LEEG;
    PDOInfo _pdos[13] = {};  // zero-initialisatie

    // Interrupt
    int _interruptPin = -1;
    volatile bool _interruptFlag = false;
    volatile uint8_t _interruptStatus = 0;

    // I2C
    // AP3377S i2c adres
    void i2c_read(uint8_t cmd, uint8_t len);
    void i2c_write(uint8_t cmd, uint8_t len);

    uint8_t _readBuf[32];
    uint8_t _writeBuf[8];

    bool _stuurAan();

    // Helpers
    int _snapVoltage(int voltage_mV) const;   // afronding PPS/AVS
    static int _currentMap(int current_mA);    // mA naar 4-bit
    static int _currentMapInverse(int waarde);

    // AP33772S register adressen
    static constexpr uint8_t CMD_STATUS    = 0x01;
    static constexpr uint8_t CMD_SRCPDO    = 0x04;
    static constexpr uint8_t CMD_REQMSG    = 0x06;
    static constexpr uint8_t CMD_VOLTAGE   = 0x11;
    static constexpr uint8_t CMD_CURRENT   = 0x12;
    static constexpr uint8_t CMD_TEMP      = 0x13;
    static constexpr uint8_t CMD_VREQ      = 0x14;
    static constexpr uint8_t CMD_SYSTEM    = 0x20;
    static constexpr uint8_t CMD_INTMASK   = 0x16;
    static constexpr uint8_t CMD_INTSTATUS = 0x17;
};
#endif //LAYZEEFRIES_REGUSBCPOW_H