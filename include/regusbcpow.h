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

    // Meten — gecachte waarden, ~1x/seconde bijgewerkt door handleWork()
    // (niet meer een directe I2C-read per aanroep — alle operationele I2C
    // zit geconcentreerd in handleWork(), zie de private members hieronder).
    unsigned int leesVoltage() const;         // mV — via de INA238, zie toelichting bij _gemetenVoltage_mV
    int leesStroom() const;                   // ruwe (signed) INA238-registerwaarde, GEEN geijkte mA — zie toelichting bij _gemetenStroom_mA
    unsigned int leesVREQ() const;            // mV, aangevraagde spanning
    unsigned int leesIREQ() const;            // mA, aangevraagde stroom
    int leesTemp();                           // °C — buiten scope, nog directe I2C (ongebruikt)

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

    RotoPdStatus handleWork();

    // Status
    bool isKlaar();
    void printTo(Print &p) const;
    // Rapporteert de gecachte spanning/stroom (leesStroom/leesVoltage/
    // leesVREQ/leesIREQ) — geen eigen I2C meer, zie hierboven.
    void printStatus(Print &p) const;

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

    // Garandeert een verse STATUS-read elke ~1s, ongeacht of de interrupt
    // (zie handleInterrupt() in de .cpp) meewerkt — die mag deze read
    // alleen vervroegen (interruptFired), niet vervangen. Zie git-historie:
    // op echte hardware bleek geen enkele geprobeerde attachInterrupt()-
    // vorm betrouwbaar genoeg om er blind op te vertrouwen (soms nooit meer
    // een edge/level na de eerste paar events), dus deze klok is de
    // eigenlijke garantie.
    unsigned long _next_statusPoll = 0;
    bool _newPdo = true;
    bool _ready = true;
    unsigned long _started_at = 0;
    unsigned long _next_avsTick;
    // EPR-onderhandeling (de 28V-PDO's) komt soms pas iets ná de eerste
    // NEWPDO-lijst binnen (gezien bij een live reconnect: eerste srcpdo()
    // toonde alleen SPR-PDO's, AVS-index -1). Eenmalige her-lees-poging kort
    // na de eerste succesvolle srcpdo(); 0xFFFFFFFF = "niet gepland".
    unsigned long _herlees_pdo_at = 0xFFFFFFFF;

    // srcpdo() bestaat zelf ook uit 2 losse I2C-reads (OPMODE, dan het grote
    // SRCPDO-blok), elk in een eigen handleWork()-aanroep in plaats van
    // synchroon achter elkaar — reads verbruiken _i2cVeilig niet (zie
    // hierboven), dus dit is puur "één stap per aanroep", geen wachten op
    // een tussenliggende READY. _srcpdoStap: 0 = nog niet begonnen (of net
    // klaar) → eerst OPMODE lezen; 1 = OPMODE gedaan, SRCPDO nog te lezen.
    // _srcpdoGewenst: een cyclus is aangevraagd (NEWPDO-event of de
    // eenmalige EPR-nalees) maar nog niet (volledig) uitgevoerd.
    // _srcpdoIsPrimair: alleen de NEWPDO-aanleiding doet na afloop de
    // bookkeeping (printTo, EPR-nalees plannen, status naar PDO_OK) — de
    // eenmalige EPR-nalees zelf niet, anders plant hij zichzelf steeds
    // opnieuw.
    uint8_t _srcpdoStap = 0;
    bool _srcpdoGewenst = false;
    bool _srcpdoIsPrimair = false;

    // Eerder gekoppeld aan specifiek de STATUS.READY-bit (datasheet: "1:
    // Ready to receive I2C request/command") — bleek op echte hardware met
    // een 1s-poll te streng: STATUS levert nog vaak 0x0 op (geen enkele bit
    // gezet), dan bleef _i2cVeilig onterecht lang false terwijl er niets
    // mis was, alleen toevallig geen nieuw event tussen twee polls in.
    // Uiteindelijke vorm: alleen de spanningsaanvraag zelf (_stuurAan()'s
    // PD_REQMSG-write) zet _i2cVeilig op false — geen enkele andere
    // i2c_read()/i2c_write() raakt 'm nog aan (ook niet bij een
    // transactiefout). True wordt hij zodra handleWork()'s periodieke
    // STATUS-poll (zie _next_statusPoll) zélf slaagt — die geslaagde
    // transactie bewijst al dat de chip reageert, ongeacht welke bits het
    // gelezen STATUS-byte draagt. Alle andere I2C (output aan/uit, metingen,
    // PDO-lijst lezen) wacht wel op dit veilig-moment om te vuren, maar
    // consumeert het zelf niet — zo lopen de periodieke AVS-herbevestiging
    // en een verse vraagbijstelling (allebei via _stuurAan()) elkaar niet
    // meer in de weg via een omweg langs een ongerelateerd write-commando.
    // Losstaand van _ready, die een eigen betekenis heeft
    // (PDO-lijst-herlees-gate, specifiek wél aan de READY-bit gekoppeld) en
    // niet verstoord mag worden.
    bool _i2cVeilig = false;

    // Gecachte meetwaarden — één register per _i2cVeilig-beurt door
    // handleWork() ververst (zie _meetStap), volledige cyclus ~1x/seconde
    // gestart. Zie leesVoltage()/leesStroom()/leesVREQ()/leesIREQ().
    //
    // _gemetenVoltage_mV/_gemetenStroom_mA komen sinds de RotoPD Pro NIET
    // meer van de AP33772S zelf, maar van de losse INA238-vermogensmonitor
    // die op dit board naast de AP33772S zit (ander I2C-adres, 0x40,
    // bevestigd via MANUFACTURER_ID=0x5449="TI"). Reden: de AP33772S's
    // eigen VOLTAGE-register bleek op echte hardware op sommige momenten
    // niet mee te bewegen met de daadwerkelijke VOUT (bv. bleef 4960mV
    // tonen terwijl een multimeter én de INA238 tegelijk 8.8V+ maten,
    // gevraagd/onderhandeld was ook echt 9V — pdResultaat=succes) — en de
    // AP33772S's CURRENT liet exact hetzelfde patroon zien: bleef letterlijk
    // elke trace op 168mA staan, ongeacht spanning/belasting, terwijl de
    // INA238's eigen CURRENT-register wél meebewoog (0→740→1105 etc.) toen
    // die nog los werd uitgelezen.
    // Let op: _gemetenStroom_mA is voorlopig de RUWE (signed) INA238-
    // registerwaarde, GEEN geijkte mA — dat vereist het CAL-register
    // (shunt-kalibratie), en die weerstandswaarde van dit specifieke board
    // kennen we nog niet. Bruikbaar om trends/verandering te zien, niet als
    // absolute stroom. Signed (int, niet unsigned): bij een lage/nul
    // belasting kan de ruwe ADC-lezing best een fractie onder 0 uitkomen
    // (ruis) — als unsigned zou dat naar een absurd groot getal wrappen.
    unsigned int _gemetenVoltage_mV = 0;
    int _gemetenStroom_mA = 0;
    unsigned int _gemetenVREQ_mV = 0;
    unsigned int _gemetenIREQ_mA = 0;
    // Resultaat van de laatste PD_REQMSG/PD_CMDMSG (PD_MSGRLT.RESPONSE,
    // datasheet: 0=busy/geen respons, 1=succes, 2=ongeldig commando/
    // argument, 3=niet ondersteund/geweigerd door de bron, 4=transactie
    // mislukt/geen GoodCRC). Puur diagnostisch — main.cpp doet er nog niets
    // mee, alleen zichtbaar via printStatus().
    uint8_t _gemetenPdResultaat = 0;
    // Teruggelezen SYSTEM-register (0x06) — vooral voor VOUTCTL (bits 1:0:
    // 0=auto, 1=force off, 2=force on), om te verifiëren dat outputAan()'s
    // write ook echt is aangekomen zoals bedoeld, in plaats van blind op
    // onze eigen _gewildOutputAan-intentie te vertrouwen.
    uint8_t _gemetenSystem = 0;
    // Teruggelezen CONFIG-register (0x04) — om te bevestigen dat UVP_EN
    // (bit 3) daadwerkelijk uitstaat, i.p.v. blind op de write in
    // srcpdo() te vertrouwen (die kan net zo goed een keer stil falen).
    uint8_t _gemetenConfig = 0;
    unsigned long _next_meetTick = 0;
    // 0=VOLTAGE (INA238), 1=CURRENT (INA238), 2=VREQ, 3=IREQ, 4=PD_MSGRLT,
    // 5=SYSTEM, 6=CONFIG — welke meting handleWork() als eerstvolgende
    // oppakt zodra er weer een veilig moment is.
    uint8_t _meetStap = 0;

    // Verificatie dat een aanvraag (_stuurAan()) ook echt is gehonoreerd:
    // soms blijft de bron op zijn vorige/standaard contract hangen (bv. 5V)
    // terwijl wij een andere spanning aanvroegen — de periodieke AVS-refresh
    // alleen (elke 500ms _stuurAan() herhalen) loste dat niet altijd op.
    // Bij aanhoudende mismatch forceert handleWork() een PD-reset. Hergebruikt
    // _gemetenVREQ_mV (zelfde meet-tick), geen aparte I2C-read.
    int _verify_mismatch_teller = 0;

    // "Wens vastleggen, centraal toepassen": setVoltage()/setStroom() doen
    // zelf geen I2C meer, ze zetten deze vlag; handleWork() past 'm toe op
    // zijn eigen ritme (of bij de periodieke AVS-herbevestiging).
    bool _aansturingGewijzigd = false;

    // Zelfde patroon voor outputAan()/outputUit().
    bool _gewildOutputAan = false;
    bool _outputWijzigingGewenst = false;

    // I2C — slaveAddr default is de AP33772S zelf; de INA238-lezingen
    // (andere chip, ander adres, zie hierboven) geven hun eigen adres mee.
    void i2c_read(uint8_t cmd, uint8_t len, uint8_t slaveAddr = AP33772S_ADDRESS);
    void i2c_write(uint8_t cmd, uint8_t len);
    byte _trans_stat = 0;
    // Aantal ACHTEREENVOLGENDE mislukte I2C-transacties (opgeteld in
    // i2c_read()/i2c_write() zelf bij _trans_stat != 0, naar 0 bij succes) —
    // geldt dus voor elke I2C-aanroep in de klasse. handleWork() degradeert
    // pas naar RotoPdStatus::GEEN_PDO als dit een drempel haalt, niet al bij
    // de eerste de beste NACK: op echte hardware bleek een losse, transiënte
    // mislukking (bv. net de STATUS-read) geen betrouwbaar signaal dat de
    // PDO echt weg is — de rest bleef gewoon werken. Telt alleen AP33772S-
    // transacties (i2c_read()/i2c_write() met slaveAddr==AP33772S_ADDRESS)
    // — dit stuurt _status aan, dus een INA238-fout hoort hier niet in mee
    // te tellen (zie _ina238FoutTeller).
    int _i2cFoutTeller = 0;
    // Zelfde tel-principe, maar los, voor de INA238 (ander chip, andere
    // gezondheid) — puur zichtbaar via printStatus(), stuurt niets aan.
    int _ina238FoutTeller = 0;

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

    // I2C-adressen — hier (i.p.v. losse file-scope constanten in de .cpp)
    // zodat i2c_read()'s default-parameter hierboven hetzelfde symbool kan
    // hergebruiken i.p.v. een los magic number.
    static constexpr uint8_t AP33772S_ADDRESS = 0x52;
    static constexpr uint8_t INA238_ADDRESS   = 0x40;

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

    // Vermogen meting — VOLTAGE (0x11)/CURRENT (0x12) van de AP33772S zelf
    // niet meer gebruikt: beide komen nu van de INA238, zie de toelichting
    // bij _gemetenVoltage_mV/_gemetenStroom_mA hierboven.
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