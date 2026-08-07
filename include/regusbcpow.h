#ifndef LAYZEEFRIES_REGUSBCPOW_H
#define LAYZEEFRIES_REGUSBCPOW_H

#include <Wire.h>

enum PDOType {
    PDO_LEEG,
    PDO_FIXED,
    PDO_PPS,
    PDO_AVS
};

// Actuele toestand van de RotoPD-koppeling — geen eenmalige events, een status
// die handleWork() elke aanroep opnieuw teruggeeft. GEEN_PDO is ook de
// beginwaarde en de fail-safe waarde bij I2C-communicatiefouten (bv. kabel
// eruit/geen VBUS): als we het goed doen komen die communicatiefouten in
// normaal bedrijf niet meer voor, en anders is er sowieso iets grondig mis —
// vandaar geen aparte I2C_FOUT-waarde.
enum class RotoPdStatus {
    GEEN_PDO,      // geen bevestigd werkende PDO — niet verwarmen/koelen
    PDO_OK,        // normale werking, een PDO is actief
    PDO_OVERFLOW   // UVP/OVP/OCP/OTP-beveiliging getriggerd
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
    void reset();  // PD hard reset (PD_CMDMSG.HRST) — dwingt een schone herstart van de PD-onderhandeling af

    // Instellen — schrijven naar AP33772S
    bool setVoltage(unsigned int voltage_mV, unsigned int current_mA);
    bool setStroom(unsigned int current_mA);

    // Meten — lezen van AP33772S registers
    unsigned int leesVoltage();              // mV, 80mV/LSB
    unsigned int leesStroom();                // mA, 24mA/LSB
    unsigned int leesVREQ();                  // mV, aangevraagde spanning
    unsigned int leesIREQ();                  // mA, aangevraagde stroom
    int leesTemp();                           // °C

    // Bereikbare spanning voor continue regeling — combineert AVS (hoge kant)
    // en PPS (lage kant), net zoals setVoltage() zelf automatisch tussen
    // AVS/PPS kiest op basis van de gevraagde spanning. Zo hoeft een
    // regelaar (bv. de PID) zijn grenzen niet zelf vast te leggen. 0 zolang
    // de PDO-lijst nog niet bekend is (of geen van beide aanwezig is).
    unsigned int leesMinVoltage() const;   // mV — PPS-minimum, anders AVS-minimum
    unsigned int leesMaxVoltage() const;   // mV — AVS-maximum, anders PPS-maximum

    // Beveiligingsdrempels — lezen/instellen
    unsigned int leesVSELMIN();               // mV, 200mV/LSB
    void stelVSELMIN(unsigned int voltage_mV);
    int leesUVPPercentage();                  // 70/75/80 (%), -1 = ongeldig
    void stelUVPPercentage(int percentage);   // alleen 70, 75 of 80 geldig
    unsigned int leesOVPTHR();                // mV, offset boven VREQ (default 2000mV)
    void stelOVPTHR(unsigned int offset_mV);
    unsigned int leesOCPTHR();                // mA
    void stelOCPTHR(unsigned int current_mA);
    int leesOTPTHR();                         // °C
    void stelOTPTHR(int temp_C);
    int leesDRTHR();                          // °C (de-rating drempel)
    void stelDRTHR(int temp_C);

    // Uitgang
    bool outputAan();
    bool outputUit();

    // Interrupt configuratie
    // void handleInterrupt();         // ISR
    RotoPdStatus handleWork();

    // Status
    bool isKlaar();
    void printTo(Print &p) const;
    // Rapporteert huidige/gevraagde spanning en stroom (leesStroom/
    // leesVoltage/leesVREQ/leesIREQ) — niet const, die doen elk een I2C-read.
    void printStatus(Print &p);

private:
    // Geïnitialiseerd bij declaratie
    int _ppsPDOIndex = -1;
    int _avsPDOIndex = -1;
    int _pdoCount = 0;
    int _huidigPDOIndex = -1;
    unsigned int _huidigVoltage_mV = 0;
    unsigned int _huidigStroom_mA = 0;

    PDOType _huidigeModus = PDO_LEEG;
    PDOInfo _pdos[13] = {};  // zero-initialisatie

    RotoPdStatus _status = RotoPdStatus::GEEN_PDO;

    // Interrupt
    bool _newPdo = true;
    bool _ready = true;
    unsigned long _started_at = 0;
    unsigned long _next_avsTick;
    // EPR-onderhandeling (de 28V-PDO's) komt soms pas iets ná de eerste
    // NEWPDO-lijst binnen (gezien bij een live reconnect: eerste srcpdo()
    // toonde alleen SPR-PDO's, AVS-index -1). Eenmalige her-lees-poging kort
    // na de eerste succesvolle srcpdo(); 0xFFFFFFFF = "niet gepland".
    unsigned long _herlees_pdo_at = 0xFFFFFFFF;
    // Verificatie dat een aanvraag (_stuurAan()) ook echt is gehonoreerd:
    // soms blijft de bron op zijn vorige/standaard contract hangen (bv. 5V)
    // terwijl wij een andere spanning aanvroegen — de periodieke AVS-refresh
    // alleen (elke 500ms _stuurAan() herhalen) loste dat niet altijd op.
    // Bij aanhoudende mismatch forceert handleWork() een PD-reset.
    unsigned long _next_verify_at = 0;
    int _verify_mismatch_teller = 0;

    // I2C
    // AP3377S i2c adres
    void i2c_read(uint8_t cmd, uint8_t len);
    void i2c_write(uint8_t cmd, uint8_t len);
    byte _trans_stat = 0;

    uint8_t _readBuf[32];
    uint8_t _writeBuf[8];

    bool _stuurAan();

    // Helpers
    static unsigned int _currentMap(unsigned int current_mA);    // mA naar 4-bit
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