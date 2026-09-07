#include <arduino.h>
#include <regusbcpow.h>
#include "console.h"

// AP33772S_ADDRESS/INA238_ADDRESS staan als static constexpr in de klasse
// zelf (regusbcpow.h) — uint8_t i.p.v. #define, dus een int-literal, want op
// de ESP32 heeft Wire's requestFrom() meerdere overloads (uint8_t/uint16_t/
// int) die met een ongetypeerd adres ambigu worden.
static constexpr uint8_t CMD_INA238_BUS_VOLTAGE = 0x05;
static constexpr uint8_t CMD_INA238_CURRENT     = 0x07;
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

// STATUS-byte van de laatste poll (zie handleWork()'s _next_statusPoll).
uint8_t pdoStatus = 0;
TwoWire *i2cPort{};

// De interrupt is hier terug, maar nu puur als snelheidsbonus bovenop de
// 1s-poll (zie handleWork()) — niet meer als enige bron van een verse
// STATUS-read zoals eerder dit project (zie git-historie: op echte
// hardware bleek geen enkele attachInterrupt()-vorm betrouwbaar genoeg om
// daar blind op te vertrouwen, dus _i2cVeilig kon permanent false blijven
// hangen). Nu geldt: komt de interrupt binnen, dan poll je meteen i.p.v.
// tot een volle seconde te wachten; blijft-ie een keer uit (zoals eerder
// gebeurde), dan pakt de gewone 1s-poll het gewoon weer op — geen enkel
// pad hangt hier nog van af.
static int interruptPin = -1;
volatile bool interruptFired = false;

static void handleInterrupt()
{
    // Detach meteen: de AP33772S-INT-pin is level-triggered (blijft HIGH
    // tot STATUS is uitgelezen), dus zonder detach zou een level-trigger
    // non-stop opnieuw afgaan totdat handleWork() aan de STATUS-read
    // toekomt (I2C is niet ISR-safe, kan dus niet hier) — precies de
    // interrupt-storm die eerder al een keer het board onbereikbaar maakte.
    // handleWork() doet weer attach() nadat de poll is afgehandeld.
    detachInterrupt(interruptPin);
    interruptFired = true;
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
    // INPUT_PULLDOWN i.p.v. INPUT: voorkomt een zwevende pin (en dus een
    // valse interrupt) zolang de RotoPD niet aangesloten/gevoed is. PULLUP
    // zou hier verkeerd zijn: de AP33772S-INT-pin is idle-LOW/actief-HIGH
    // (datasheet), dus zonder actieve aansturing (geen VBUS) hoort de pin
    // richting LOW te zakken.
    pinMode(i, INPUT_PULLDOWN);
    interruptPin = digitalPinToInterrupt(i);
    // ONHIGH: echte level-trigger, matcht de datasheet ("Interrupt Signal").
    // De detach/attach-cyclus (zie handleInterrupt()/handleWork()) voorkomt
    // de storm die een kale ONHIGH eerder gaf. Maar zelfs met die cyclus is
    // dit alleen nog een snelheidsbonus bovenop de 1s-poll — zie de
    // toelichting hierboven bij interruptFired.
    attachInterrupt(interruptPin, handleInterrupt, ONHIGH);

    reset();  // schone herstart van de PD-onderhandeling bij elke boot

    _writeBuf[0] = PDO_STARTED | PDO_READY | PDO_NEWPDO;
    i2c_write(CMD_MASK, 1);
    // UVP_EN (CONFIG, 0x04) wordt uitgezet in srcpdo(), niet hier — zie
    // toelichting daar: dit punt in begin() draait vóórdat we I2C-succes
    // ooit bevestigd hebben, en vóórdat Serial/BLE actief zijn om een
    // eventuele mislukking te tonen.

    _started_at = millis() + 200;
}

void regUSBCPow::reset()
{
    // Een reset gooit de PD-onderhandeling om — tot de eerstvolgende verse
    // READY-bit is I2C niet betrouwbaar (datasheet: READY = "ready to
    // receive I2C request/command"). i2c_write() zelf zet _i2cVeilig al
    // false zodra dit commando verstuurd wordt, geen aparte toewijzing nodig.
    _writeBuf[0] = 0x01;  // PD_CMDMSG.HRST
    i2c_write(CMD_PD_CMDMSG, 1);
}

void regUSBCPow::srcpdo()
{
    if (_srcpdoStap == 0)
    {
        // Stap 0: alles resetten en OPMODE lezen (is dit wel een PD-bron?).
        // De SRCPDO-read zelf wacht op de eerstvolgende veilige beurt —
        // zie de toelichting bij _srcpdoStap in de header.
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

        i2c_read(CMD_OPMODE, 1);
        if (!_readBuf[0] & 0x02) // PD source?
        {
            _srcpdoGewenst = false;  // niets te lezen, cyclus is klaar
            return;
        }
        _srcpdoStap = 1;
        return;
    }

    // Stap 1: OPMODE was al gedaan, nu het eigenlijke PDO-blok.
    i2c_read(CMD_SRCPDO, 26);
    _srcpdoStap = 0;
    _srcpdoGewenst = false;

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

    printTo(console);
    if (_srcpdoIsPrimair)
    {
        // EPR-PDO's (de 28V-opties) kunnen iets later pas klaar zijn dan de
        // basis SPR-onderhandeling — alleen over 1s nog eens kijken als AVS
        // er nu nog niet bij zat. Alleen de primaire (NEWPDO-)aanleiding
        // doet dit; de eenmalige EPR-nalees zelf plant zichzelf niet opnieuw.
        if (_avsPDOIndex <= 0)
            _herlees_pdo_at = millis() + 1000;
        _status = RotoPdStatus::PDO_OK;  // een verse PDO-lijst is per definitie weer OK

        // UVP uitschakelen — hier, niet in begin(): dit punt bewijst dat I2C
        // al werkt (we lezen net een geldige PDO-lijst terug), i.p.v. de
        // vroege begin()-poging die vóór Serial/BLE actief zijn kon falen
        // zonder dat we het ooit zagen (zelfde valkuil als eerder bij de
        // INA238-identiteitscheck). Idempotent bij elke verse PDO-lijst
        // (dus ook na een reconnect) is geen probleem, eerder robuuster.
        i2c_read(CMD_CONFIG, 1);
        _writeBuf[0] = _readBuf[0] & ~0x08;
        i2c_write(CMD_CONFIG, 1);
    }
}

// Spanning instellen
bool regUSBCPow::setVoltage(unsigned int voltage_mV, unsigned int current_mA) {
    // setVoltage() wordt nu elke ~1s aangeroepen (main.cpp's vraagTick()) —
    // AVS/PPS alleen loggen bij een daadwerkelijke wissel, niet elke keer.
    int vorigPDOIndex = _huidigPDOIndex;
    _huidigPDOIndex = -1;
    if (_avsPDOIndex > 0) {
        if (voltage_mV >= _pdos[_avsPDOIndex-1].voltage_min_mV &&
            voltage_mV <= _pdos[_avsPDOIndex-1].voltage_max_mV) {
            _huidigPDOIndex = _avsPDOIndex;
        }
    }
    if (_huidigPDOIndex < 0 && _ppsPDOIndex > 0) {
        if (voltage_mV >= _pdos[_ppsPDOIndex-1].voltage_min_mV &&
            voltage_mV <= _pdos[_ppsPDOIndex-1].voltage_max_mV) {
            _huidigPDOIndex = _ppsPDOIndex;
        }
    }
    if (_huidigPDOIndex != vorigPDOIndex && _huidigPDOIndex > 0) {
        console.println(_huidigPDOIndex == _avsPDOIndex ? "AVS" : "PPS");
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

    // Geen I2C hier — alleen de wens vastleggen, handleWork() past 'm toe
    // op zijn eigen ritme (zie de "I2C veilig"-toelichting bij _i2cVeilig).
    _aansturingGewijzigd = true;
    return true;
}

// Stroom instellen
bool regUSBCPow::setStroom(unsigned int current_mA) {
    _huidigStroom_mA = current_mA;
    _aansturingGewijzigd = true;
    return true;
}

// Uitgang — geen I2C hier, alleen de wens vastleggen (zelfde patroon als
// setVoltage()/setStroom()); handleWork() past 'm toe.
bool regUSBCPow::outputAan() {
    _gewildOutputAan = true;
    _outputWijzigingGewenst = true;
    return true;
}

bool regUSBCPow::outputUit() {
    _gewildOutputAan = false;
    _outputWijzigingGewenst = true;
    return true;
}

// Meten — gecachte waarden, zie _gemeten*_mV/_mA en de meet-tick in handleWork().
unsigned int regUSBCPow::leesVoltage() const {
    return _gemetenVoltage_mV;
}

int regUSBCPow::leesStroom() const {
    return _gemetenStroom_mA;
}

unsigned int regUSBCPow::leesVREQ() const {
    return _gemetenVREQ_mV;
}

unsigned int regUSBCPow::leesIREQ() const {
    return _gemetenIREQ_mA;
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

void regUSBCPow::printStatus(Print &p) const {
    int huidig_stroom = leesStroom();
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

    if (now > _next_statusPoll || interruptFired)
    {
        _next_statusPoll = now + 1000;
        interruptFired = false;
        // De 1s-cadans is de garantie (zie het commentaar bij
        // interruptFired hierboven); een binnengekomen interrupt mag deze
        // read alleen vervroegen. Lezen reset het register (datasheet:
        // "Reset to 0 after every Read").
        i2c_read(CMD_STATUS, 1);
        // Bij falen niets te verwerken (geen geldige data) — de eventuele
        // degradatie naar GEEN_PDO gebeurt hierna centraal, pas na een paar
        // opeenvolgende mislukkingen (zie _i2cFoutTeller).
        if (_trans_stat == 0)
        {
            pdoStatus = _readBuf[0];
            // Deze STATUS-poll zelf is een geslaagde I2C-transactie met de
            // AP33772S — dat alleen al bewijst dat de chip nu reageert, dus
            // veilig genoeg om ook te schrijven. Eerder stond dit gekoppeld
            // aan specifiek de READY-bit in dit STATUS-byte, maar die bleek
            // op echte hardware met een 1s-poll nog vaak 0x0 op te leveren
            // (geen enkele bit gezet) — dan bleef _i2cVeilig onnodig lang
            // false terwijl er niets mis was, alleen toevallig geen nieuw
            // event tussen twee polls in. PDO_READY hieronder blijft wel
            // los bestaan voor zijn eigen betekenis (_ready, gekoppeld aan
            // _newPdo voor het herlezen van de PDO-lijst).
            _i2cVeilig = true;
            // 0x2 (kale READY) is een echt, betekenisvol event — als kale
            // punt afgedrukt, anders zou de trace bij elke poll opblazen.
            // 0x0 (niets gezet) is iets anders: geen event, gewoon een poll
            // die toevallig niets te melden had — dat is geen READY en
            // helemaal niet printen, ook niet als punt. Elke andere waarde
            // (foutbits, NEWPDO, STARTED) wél volledig afdrukken.
            if (pdoStatus == 0x2)
            {
                console.print('.');
            } else if (pdoStatus != 0x0)
            {
                console.print("pdoStatus: 0x");
                console.println(pdoStatus, HEX);
            }

            if (pdoStatus & (PDO_UVP | PDO_OVP | PDO_OCP | PDO_OTP))
            {
                // Beveiliging getriggerd: chip schakelt VOUT uit. Datasheet:
                // "the host MCU will need to load new PD_REQMSG to start a
                // PDO negotiation process to resume" — dat gebeurt bij het
                // herstel (zie check_mode()'s "pop"), niet hier.
                _status = RotoPdStatus::PDO_OVERFLOW;
            }
            else if ((pdoStatus & PDO_READY) && _status == RotoPdStatus::PDO_OVERFLOW)
            {
                // Schoon READY-signaal zonder foutbits: hersteld van de
                // beveiligingstrip.
                _status = RotoPdStatus::PDO_OK;
            }
            if (pdoStatus & PDO_NEWPDO)
                _newPdo = true;
            if (pdoStatus & PDO_READY) {
                // _ready is los van _i2cVeilig: dit stuurt alleen het
                // herlezen van de PDO-lijst aan (samen met _newPdo), geen
                // schrijf-veiligheid meer — zie hierboven.
                _ready = true;
            }
            if (pdoStatus & PDO_STARTED)
                _started_at = now + 100;
            // MASK (0x02) is een gewoon RW-configuratieregister, geen
            // Read-to-Clear zoals STATUS (datasheet Table 11/20: geen
            // "reset on read"-vermelding) — eenmalig gezet in begin()
            // volstaat. Hem hier bij elke interrupt herschrijven was een
            // overbodige tweede I2C-commando direct na de STATUS-read,
            // zonder op een verse READY te wachten voor dat tweede
            // commando — precies het patroon dat we nu juist vermijden.
        }
        // Weer attach() — ongeacht of de poll hierboven via de 1s-klok dan
        // wel de interrupt kwam, en ongeacht of de STATUS-read slaagde: bij
        // een falende read willen we ook niet doof blijven voor de
        // volgende interrupt (de 1s-poll ving dat sowieso al op, maar dan
        // zonder de snelheidsbonus). Staat de pin nu nog HIGH (event nog
        // niet echt gecleard), dan vuurt de ISR gewoon meteen opnieuw —
        // geen storm, want handleInterrupt() zelf detacht meteen weer.
        attachInterrupt(interruptPin, handleInterrupt, ONHIGH);
    }
    // Een vers NEWPDO-signaal betekent altijd: PDO-lijst opnieuw inlezen —
    // ongeacht wat _status daarvoor toevallig was. Niet vastklinken aan
    // "_status was GEEN_PDO": een reconnect triggert niet altijd (opnieuw)
    // een UVP/OVP/OCP/OTP-bit of I2C-fout onderweg (gezien op echte hardware:
    // I2C bleef werken, pdoStatus was zelfs 0x0 tijdens het loskoppelen),
    // dus _status kan best op PDO_OK zijn blijven staan terwijl er ondertussen
    // wél een volledige her-onderhandeling plaatsvond.
    if (_ready && _newPdo && now > _started_at)
    {
        // Wens vastleggen, niet meteen lezen — srcpdo() zelf wacht met zijn
        // I2C op _i2cVeilig (zie hieronder en de toelichting bij
        // _srcpdoStap in de header).
        _srcpdoGewenst = true;
        _srcpdoIsPrimair = true;
    }
    if (now > _herlees_pdo_at)
    {
        _herlees_pdo_at = 0xFFFFFFFF;  // eenmalig
        // Alleen aanvragen als er niet toevallig al een (primaire) cyclus
        // onderweg is — anders zou deze aanvraag die halverwege ombuigen
        // naar "niet-primair" en de afrondende bookkeeping ervan missen.
        if (_srcpdoStap == 0)
        {
            _srcpdoGewenst = true;
            _srcpdoIsPrimair = false;
        }
    }
    // srcpdo() bestaat zelf ook uit losse I2C-reads (zie _srcpdoStap) — dus
    // ook hier: pas oppakken op een veilig moment, en verder laten lopen
    // zolang een cyclus al onderweg is (_srcpdoStap != 0), ongeacht of er
    // ondertussen nog een nieuwe wens is bijgekomen.
    if (_i2cVeilig && (_srcpdoGewenst || _srcpdoStap != 0))
    {
        srcpdo();
    }
    // Alle "gewone bedrijfsvoering"-I2C hieronder wacht op een veilig moment
    // (zie _i2cVeilig hierboven) — zo staat alle operationele I2C op één
    // plek, gecoördineerd, i.p.v. verspreid over losse aanroepen vanuit
    // main.cpp op willekeurige momenten. _i2cVeilig is een grove
    // aanspreekbaarheids-vlag, geen per-commando-token: zolang hij true is
    // mogen meerdere van onderstaande blokken gewoon in dezelfde aanroep
    // vuren.
    //
    // Metingen staan HIER BEWUST als eerste, vóór output/aansturing: reads
    // verbruiken _i2cVeilig niet, maar _stuurAan() (spanning/stroom) wél —
    // en bepaalAansturing() in main.cpp roept setVoltage() elke regel-tick
    // opnieuw aan, ook als het doel niet wijzigt, dus _aansturingGewijzigd
    // staat vrijwel altijd weer vers op true tegen de tijd dat een nieuwe
    // READY binnenkomt. Stond dit blok ná de aansturing, dan graaide die elk
    // veilig moment meteen weg vóórdat de metingen ooit aan de beurt kwamen
    // — reëel waargenomen op echte hardware: bij PPS (geen periodieke
    // keepalive zoals AVS, dus weinig verse READY's) bleef _gemetenVoltage_mV
    // minutenlang op een oude waarde hangen terwijl VREQ allang het nieuwe
    // contract toonde. Nu de reads voorop staan, krijgen ze altijd hun kans
    // op elk veilig moment; de aansturing kan daarna nog gewoon in dezelfde
    // aanroep vuren, niets aan die logica verandert.
    //
    // VOLTAGE/CURRENT/VREQ/IREQ/PD_MSGRLT/SYSTEM, één register per aanroep
    // (_meetStap telt 0..5) in plaats van alle 6 in één keer — puur om het
    // niet allemaal in dezelfde handleWork()-aanroep te proppen. De cyclus
    // loopt vanzelf door over opeenvolgende aanroepen totdat hij weer bij 0
    // uitkomt. _next_meetTick wordt pas dán vooruitgezet, dus de gate
    // hieronder blokkeert alleen het *starten* van een nieuwe cyclus vóór
    // zijn tijd, niet het afmaken van een lopende.
    if (_i2cVeilig && now > _next_meetTick)
    {
        switch (_meetStap)
        {
            case 0:
                // Van de INA238, niet de AP33772S — zie toelichting bij
                // _gemetenVoltage_mV in de header. Big-endian, 3.125mV/LSB.
                i2c_read(CMD_INA238_BUS_VOLTAGE, 2, INA238_ADDRESS);
                _gemetenVoltage_mV = (unsigned int) (((uint16_t) _readBuf[0] << 8 | _readBuf[1]) * 3.125f);
                break;
            case 1:
                // Van de INA238, niet de AP33772S — zie toelichting bij
                // _gemetenStroom_mA in de header. Ruwe registerwaarde, geen
                // geijkte mA (geen CAL/shunt-kalibratie gezet).
                i2c_read(CMD_INA238_CURRENT, 2, INA238_ADDRESS);
                _gemetenStroom_mA = (int16_t) (((uint16_t) _readBuf[0] << 8) | _readBuf[1]);
                break;
            case 2:
                // VREQ (0x14) is een 2-byte register (datasheet Table 19).
                i2c_read(CMD_VREQ, 2);
                _gemetenVREQ_mV = ((unsigned int) _readBuf[1] << 8 | _readBuf[0]) * 50;
                break;
            case 3:
                i2c_read(CMD_IREQ, 2);
                _gemetenIREQ_mA = ((unsigned int) _readBuf[1] << 8 | _readBuf[0]) * 10;
                break;
            case 4:
                // PD_MSGRLT.RESPONSE (bits 2:0): 0=busy/geen respons,
                // 1=succes, 2=ongeldig, 3=geweigerd door de bron,
                // 4=transactie mislukt/geen GoodCRC — het daadwerkelijke
                // resultaat van de laatste PD_REQMSG, in tegenstelling tot
                // VREQ/IREQ (die volgens de EVB-guide alleen bij succes
                // bijwerken, maar dat geeft geen reden waarom niet).
                i2c_read(CMD_PD_MSGRLT, 1);
                _gemetenPdResultaat = _readBuf[0] & 0x07;
                break;
            case 5:
                // SYSTEM teruglezen — vooral VOUTCTL (bits 1:0: 0=auto,
                // 1=force off, 2=force on) om te verifiëren dat outputAan()'s
                // write ook echt is aangekomen zoals bedoeld.
                i2c_read(CMD_SYSTEM, 1);
                _gemetenSystem = _readBuf[0];
                break;
            case 6:
            default:
                // CONFIG teruglezen — vooral UVP_EN (bit 3) om te
                // bevestigen dat de write in srcpdo() daadwerkelijk is
                // aangekomen.
                i2c_read(CMD_CONFIG, 1);
                _gemetenConfig = _readBuf[0];
                break;
        }

        _meetStap = (_meetStap + 1) % 7;
        if (_meetStap == 0)
        {
            // Cyclus rond: volgende cyclus plannen en de verificatie doen
            // (gebruikt de net ververste VREQ uit deze cyclus).
            _next_meetTick = now + 1000;

            // Verificatie: is de laatst aangevraagde spanning ook echt
            // gehonoreerd? Real-hardware logs lieten een aanvraag zien die
            // minutenlang op de bron z'n vorige/standaard contract bleef
            // hangen (de periodieke AVS-refresh hierboven loste dat niet
            // altijd op). Bij aanhoudende mismatch (~10s) forceert een
            // PD-reset een schone heronderhandeling; de eerstvolgende
            // aansturing vraagt daarna vanzelf opnieuw aan.
            if (_huidigPDOIndex > 0)
            {
                int verschil = (int) _gemetenVREQ_mV - (int) _huidigVoltage_mV;
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
        }
    }

    // Output aan/uit — TIJDELIJK UITGESCHAKELD voor de AVS-diagnose.
    // outputAan()/outputUit() zetten nog wel _gewildOutputAan/
    // _outputWijzigingGewenst (main.cpp's aanroepen hoeven niet te
    // wijzigen), maar de daadwerkelijke VOUTCTL-write hieronder gebeurt nu
    // niet meer: VOUTCTL blijft op zijn power-on-default staan (0x10, bits
    // 1:0 = 00 = auto). Reden: de trace liet zien dat spanning/stroom
    // elektrisch prima waren terwijl STATUS.UVP + de FAULT-LED toch bleven
    // hangen — mogelijk omdat ons eigen forceren van VOUTCTL (aan/uit) de
    // chip's eigen schakelaar-/foutherstellogica overrulet/maskeert, i.p.v.
    // de chip zelf te laten herstellen via alleen een verse RDO (zoals de
    // datasheet beschrijft: "load a new PD_REQMSG ... to resume").
    // if (_i2cVeilig && _outputWijzigingGewenst)
    // {
    //     _writeBuf[0] = _gewildOutputAan ? 0b00010010 : 0b00010001;
    //     i2c_write(CMD_SYSTEM, 1);
    //     _outputWijzigingGewenst = false;
    // }

    // Spanning/stroom — wens van setVoltage()/setStroom(), of de periodieke
    // AVS/PPS-herbevestiging (_stuurAan() beheert _next_avsTick zelf).
    if (_i2cVeilig && (_aansturingGewijzigd || now > _next_avsTick))
    {
        _stuurAan();
        _aansturingGewijzigd = false;
    }

    // Pas na een paar ACHTEREENVOLGENDE mislukte I2C-transacties degraderen
    // naar GEEN_PDO, niet al bij de eerste de beste — zie _i2cFoutTeller.
    // Een enkele transiënte NACK (bv. net de STATUS-read) betekende op
    // echte hardware niet dat de PDO ook echt weg was; de rest bleef gewoon
    // werken.
    if (_i2cFoutTeller >= 3)
        _status = RotoPdStatus::GEEN_PDO;
    return _status;
}

// Private helpers
bool regUSBCPow::_stuurAan() {
    if (_huidigPDOIndex < 0)
    {
        // Geen PDO geselecteerd (bv. net na een PD-reset/HRST, vóórdat de
        // eerstvolgende setVoltage() 'm opnieuw kiest) — hier eerder een
        // stille no-op, maar zonder _i2cVeilig te verbruiken of
        // _next_avsTick te verzetten bleef de aanroepende blok in
        // handleWork() dit iedere loop()-iteratie opnieuw proberen: een
        // ongecontroleerde busy-loop (zichtbaar geworden doordat die
        // aanroeper nu ook nog logt vlak vóór elke poging). Zelfde
        // "verbruikt bij elke poging, ook een mislukte" patroon als de rest
        // van deze functie hanteren.
        _i2cVeilig = false;
        _next_avsTick = millis() + 500;
        return false;
    }

    int huidigeModus = _pdos[_huidigPDOIndex-1].type;
    RDO_DATA_T rdo;
    rdo.data = 0;
    rdo.REQMSG_Fields.PDO_INDEX = _huidigPDOIndex;

    rdo.REQMSG_Fields.CURRENT_SEL = _currentMap(_huidigStroom_mA);
    if (huidigeModus == PDO_PPS || huidigeModus == PDO_AVS) {
        int stap = (huidigeModus == PDO_AVS) ? 200 : 100;
        rdo.REQMSG_Fields.VOLTAGE_SEL = _huidigVoltage_mV / stap;
    }
    // USB-PD-spec: een APDO-contract (PPS én AVS) moet minstens elke 10s
    // opnieuw bevestigd worden, anders mag de bron het laten verlopen/een
    // hard reset doen. Dit stond hier eerder alleen voor AVS aan — voor PPS
    // op "nooit" (0xFFFFFFFF), terwijl onze eigen regellus (~60s-cadans)
    // ruim boven die 10s-grens zit. Gevolg op echte hardware: een PPS-
    // aanvraag negotieerde prima (VREQ/pdResultaat klopten), maar de bron
    // liet 'm stilletjes verlopen vóór de volgende meting — VOLTAGE bleef
    // dan op de teruggevallen 5V-standaard hangen terwijl VREQ nog het
    // (inmiddels vervallen) hogere contract toonde. Fixed PDO's kennen dit
    // probleem niet (geen APDO, geen periodieke herbevestiging vereist).
    // 1000ms voor zowel AVS als PPS — matcht CentyLab's eigen RotoPD Pro-
    // referentievoorbeeld ("Some charger will disconnect with sink if no
    // refresh request is sent within 1s"). De eerdere aanname dat een AVS-
    // herhaling zélf een glitch veroorzaakte klopte niet — de echte oorzaak
    // van de UVP-FAULT is dat UVP op dit board default aanstaat en (per
    // diezelfde referentiecode) uitgezet moet worden, zie begin().
    if (huidigeModus == PDO_AVS || huidigeModus == PDO_PPS)
    {
        _next_avsTick = millis() + 1000;
    } else
    {
        _next_avsTick = 0xFFFFFFFF;
    }
    _writeBuf[0] = rdo.byte0;
    _writeBuf[1] = rdo.byte1;
    i2c_write(CMD_PD_REQMSG, 2);
    _i2cVeilig = false;
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

void regUSBCPow::i2c_read(byte cmdAddr, byte len, byte slaveAddr)
{
    memset(_readBuf, 0, sizeof(_readBuf));
    i2cPort->beginTransmission(slaveAddr);           // transmit to device SLAVE_ADDRESS
    i2cPort->write(cmdAddr);                         // sets the command register
    _trans_stat = i2cPort->endTransmission();        // stop transmitting
    // _i2cFoutTeller drijft de degradatie naar GEEN_PDO (zie handleWork()) —
    // hoort dus alleen te tellen over de AP33772S zelf, niet over de losse
    // INA238 (ander chip, andere gezondheid; een INA238-hikje zegt niets
    // over de PD-onderhandeling). INA238-fouten krijgen hun eigen, aparte
    // teller, puur voor logging.
    bool isAp33772s = (slaveAddr == AP33772S_ADDRESS);
    if (_trans_stat != 0)
    {
        console.print("I2C read failed: 0x");
        console.println(_trans_stat, HEX);
        if (isAp33772s) _i2cFoutTeller++;
        else _ina238FoutTeller++;
    }
    else
    {
        if (isAp33772s) _i2cFoutTeller = 0;
        else _ina238FoutTeller = 0;
    }

    i2cPort->requestFrom(slaveAddr, len);             // request len bytes from peripheral device
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
        _i2cFoutTeller++;
    }
    else
    {
        _i2cFoutTeller = 0;
    }

    // clear readBuffer
    memset(_readBuf, 0, sizeof(_readBuf));
}

