#ifndef LAYZEEFRIES_REGUSBCPOW_H
#define LAYZEEFRIES_REGUSBCPOW_H

#include <Wire.h>

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
    explicit regUSBCPow(TwoWire &wire=Wire);
    // Initialisatie
    void begin(int i);
    void srcpdo();

    // Instellen — schrijven naar AP33772S
    bool setVoltage(unsigned int voltage_mV, unsigned int current_mA);
    bool setStroom(unsigned int current_mA);

    // Meten — lezen van AP33772S registers
    unsigned int leesVoltage();              // mV, 80mV/LSB
    unsigned int leesStroom();               // mA, 24mA/LSB
    unsigned int leesVREQ();                 // mV, overflow fix
    int leesTemp();                 // °C

    // Uitgang
    bool outputAan();
    bool outputUit();

    // Interrupt configuratie
    // void handleInterrupt();         // ISR
    void handleWork();

    // Status
    bool isKlaar();
    void printTo(Print &p) const;

private:
    // Geïnitialiseerd bij declaratie
    int _ppsPDOIndex = -1;
    int _avsPDOIndex = -1;
    int _pdoCount = 0;
    int _huidigPDOIndex = -1;
    unsigned int _huidigVoltage_mV = 0;
    unsigned int _huidigStroom_mA = 0;
    unsigned int _maxVoltage_mV = 0;
    unsigned int _maxStroom_mA = 5000;

    PDOType _huidigeModus = PDO_LEEG;
    PDOInfo _pdos[13] = {};  // zero-initialisatie

    // Interrupt
    bool _newPdo = true;
    bool _ready = true;
    unsigned long _started_at = 0;
    unsigned long _next_avsTick;

    // I2C
    // AP3377S i2c adres
    void i2c_read(uint8_t cmd, uint8_t len);
    void i2c_write(uint8_t cmd, uint8_t len);
    byte _trans_stat = 0;

    uint8_t _readBuf[32];
    uint8_t _writeBuf[8];

    bool _stuurAan();

    // Helpers
    static int _currentMap(int current_mA);    // mA naar 4-bit
    static int _currentMapInverse(int waarde);

    // Bit field structs — implementatiedetail
    typedef struct {
        union {
            struct {
                unsigned int voltage_max:  8;
                unsigned int peak_current: 2;
                unsigned int current_max:  4;
                unsigned int type:         1;
                unsigned int detect:       1;
            } fixed;
            struct {
                unsigned int voltage_max: 8;
                unsigned int voltage_min: 2;
                unsigned int current_max: 4;
                unsigned int type:        1;
                unsigned int detect:      1;
            } pps;
            struct {
                unsigned int voltage_max: 8;
                unsigned int voltage_min: 2;
                unsigned int current_max: 4;
                unsigned int type:        1;
                unsigned int detect:      1;
            } avs;
            struct {
                uint8_t byte0;
                uint8_t byte1;
            };
        };
        uint32_t data;
    } SRC_SPRandEPR_PDO_Fields;

    typedef struct {
        union {
            struct {
                unsigned int VOLTAGE_SEL: 8;
                unsigned int CURRENT_SEL: 4;
                unsigned int PDO_INDEX:   4;
            } REQMSG_Fields;
            struct {
                uint8_t byte0;
                uint8_t byte1;
            };
            uint32_t data;
        };
    } RDO_DATA_T;

    SRC_SPRandEPR_PDO_Fields _srcPDOs[13]{};

    // Status en configuratie
    static constexpr uint8_t CMD_STATUS   = 0x01; // Reset to 0 after every Read
    static constexpr uint8_t CMD_MASK     = 0x02;
    static constexpr uint8_t CMD_OPMODE   = 0x03;
    static constexpr uint8_t CMD_CONFIG   = 0x04;
    static constexpr uint8_t CMD_PDCONFIG = 0x05;
    static constexpr uint8_t CMD_SYSTEM   = 0x06;

    // Temperatuur instelling
    static constexpr uint8_t CMD_TR25     = 0x0C;
    static constexpr uint8_t CMD_TR50     = 0x0D;
    static constexpr uint8_t CMD_TR75     = 0x0E;
    static constexpr uint8_t CMD_TR100    = 0x0F;

    // Vermogen meting
    static constexpr uint8_t CMD_VOLTAGE  = 0x11;
    static constexpr uint8_t CMD_CURRENT  = 0x12;
    static constexpr uint8_t CMD_TEMP     = 0x13;
    static constexpr uint8_t CMD_VREQ     = 0x14;
    static constexpr uint8_t CMD_IREQ     = 0x15;

    // Beveiliging drempelwaarden
    static constexpr uint8_t CMD_VSELMIN  = 0x16; // Minimum Selection Voltage
    static constexpr uint8_t CMD_UVPTHR   = 0x17;
    static constexpr uint8_t CMD_OVPTHR   = 0x18;
    static constexpr uint8_t CMD_OCPTHR   = 0x19;
    static constexpr uint8_t CMD_OTPTHR   = 0x1A;
    static constexpr uint8_t CMD_DRTHR    = 0x1B;

    // PDO
    static constexpr uint8_t CMD_SRCPDO   = 0x20;

    // PD berichten
    static constexpr uint8_t CMD_PD_REQMSG = 0x31;
    static constexpr uint8_t CMD_PD_CMDMSG = 0x32;
    static constexpr uint8_t CMD_PD_MSGRLT = 0x33;

    // Status en interrupt bits
    static constexpr uint8_t PDO_STARTED   = 0x01;
    static constexpr uint8_t PDO_READY     = 0x02;
    static constexpr uint8_t PDO_NEWPDO    = 0x04;
    static constexpr uint8_t PDO_UVP       = 0x08;  // Undervoltage
    static constexpr uint8_t PDO_OVP       = 0x10;  // Overvoltage
    static constexpr uint8_t PDO_OCP       = 0x20;  // Over current
    static constexpr uint8_t PDO_OTP       = 0x40;  // Over temperature

};
#endif //LAYZEEFRIES_REGUSBCPOW_H