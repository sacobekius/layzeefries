#include <Arduino.h>
#include <../include/regusbcpow.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include "console.h"
#include "bleserial.h"
#include "wifi_credentials.h"

#define ledAan(led) digitalWrite(led, HIGH)
#define ledUit(led) digitalWrite(led, LOW);

// Let op: op de Nano ESP32 zijn de rauwe pinnummers 14-16 de onboard RGB-led
// (i.p.v. A0-A2 zoals op de oude AVR Nano). Daarom hier expliciet A0..A3
// gebruiken, zodat de LEDs op dezelfde header-pinnen blijven zitten.
#define LEDBLAUW A0
#define LEDGEEL A1
#define LEDGROEN A2
#define LEDROOD A3
#define KOUDE_VRAAG 2
#define RELAIS1 3
#define RELAIS2 4
#define WARMTE_VRAAG 5
// OneWire bit-bangt via directe GPIO-registers (PIN_TO_BITMASK) en omzeilt
// daarmee pinMode()/digitalWrite() — en dus ook de pin-remap-laag die "D6"
// normaal naar de echte GPIO zou vertalen. Daarom hier het RAUWE GPIO-nummer
// (9) i.p.v. het logische Nano-pinnummer "D6" (6).
#define TEMPERATUUR_IN 9
#define ROTOPD_INT 7
// Let op: dit is het LOGISCHE Nano-pinnummer "D9" (analogWrite() is
// remap-bewust, i.t.t. OneWire hierboven). Toevallig ook een "9", maar dat
// is een heel andere fysieke pin dan TEMPERATUUR_IN's RAUWE GPIO 9 (=D6)!
#define FAN_PWM 9  // gedeelde PWM-lijn naar de 4 Noctua's (25kHz, geen tacho — 4 fans op 1 lijn)

#define FAN_PWM_FREQUENTIE 25000  // Hz, Noctua/Intel 4-pin PWM-spec

#define DOEL_TEMPERATUUR_FALLBACK_C 10.0  // gebruikt zolang NVS nog geen echte capture heeft
#define PID_KP 1000.0             // mV per graad afwijking — bij fout=13C al verzadigd op het maximum (was 500, te traag naar vol vermogen)
#define PID_KI_PER_TICK 250.0     // mV integraal-opbouw per graad fout, per bepaalAansturing()-aanroep (~elke 60s) — was 50, te traag bij het laatste stukje vlak bij het doel; grote fouten (>3C) komen hier toch niet, die vangt de vriezerregelaar's actieve vraag al af
#define PID_STROOM_MA 5000        // vast voor nu, alleen de spanning wordt geregeld — Peltiers verdragen tot 7000mA, 3000 bleek te krap (OCP-trip bij een richtingwissel)
// Bij het loslaten van een vraag (koude_vraag/warmte_vraag) valt de fout
// terug naar ~0 — zonder meer zou de regeling dan in één klap van vol
// vermogen naar bijna niets springen, en dat bleek in de praktijk veel te
// weinig om de temperatuur vast te houden (spanning kroop na 35 minuten nog
// maar tot 13500mV, tegen 28000mV tijdens actief koelen). AFSCHAAL_KD zet de
// afkoelsnelheid over de laatste TREND_VENSTER metingen (dus vlak vóór het
// signaal, bij vol vermogen) om in een startwaarde voor de integraal — die
// dooft niet vanzelf uit maar wordt door de normale I-opbouw verder
// bijgesteld op de daadwerkelijke fout.
// Eerste kalibratie (grens/snelheid uit een echte meting) schoot meteen door
// naar 100% van het bereik en koelde te ver door — AFSCHAAL_KD_MAX_FRACTIE
// begrenst de STARTSCHATTING specifiek (niet de doorlopende I-opbouw erna)
// tot een fractie van het bereik, zodat er nog ruimte overblijft om verder
// bij te sturen i.p.v. meteen te verzadigen. Beide startwaarden, verder te
// tunen op basis van logging.
#define AFSCHAAL_KD 100000.0
#define AFSCHAAL_MAX_FRACTIE 0.5
#define TREND_VENSTER 10
#define VOLTAGE_MIN_MV 10000  // vloer voor de regeling — zie toelichting bij bepaalAansturing()
// Ventilator tijdens vasthouden: comfortband i.p.v. eerste hendel vóór de
// spanning. Nooit onder de vloer (geen meerwaarde), comfortabel tot
// FAN_COMFORT bij de eerste FAN_COMFORT_GRENS_PCT% van het vermogensbereik;
// pas daarboven wijkt comfort voor koelvermogen. Tijdens actieve vraag blijft
// de ventilator gewoon op 100% (ongewijzigd, elders in bepaalAansturing()).
#define FAN_VLOER 40
#define FAN_COMFORT 60
#define FAN_COMFORT_GRENS_PCT 80

// Grote fouten horen niet via P+I afgehandeld te worden — dat gaf te veel
// overshoot-risico (zie de afschaal-geschiedenis hierboven). De vriezerregelaar
// ving dat tot nu toe af via koude_vraag/warmte_vraag, maar die is niet
// per se blijvend aangesloten. Dezelfde drempel (3°C, gelijk aan de F2-
// hysterese van de vriezerregelaar) dus ook zelfstandig in software: een
// afwijking groter dan dit gaat naar vol vermogen, ongeacht of er een
// externe vraag actief is.
#define GROTE_FOUT_DREMPEL_C 3.0

// Puur nog richting-constantes (geen modus-machine meer, zie check_fout()/
// bepaalAansturing()/pasAansturingToe()).
#define MODE_KOELEN 1
#define MODE_VERWARMEN 2
#define MODE_UIT 5
#define MODE_FOUT 6

regUSBCPow usbpd;
Preferences prefs;
DeviceAddress temperatuurMeter;
OneWire oneWire(TEMPERATUUR_IN);
DallasTemperature sensors(&oneWire);
int numberOfSensors;
bool testSucces = true;

int PPSIndex;
int AVSIndex;
float huidige_temperatuur;
// Temperatuur van de vorige temperatuurTick() (~60s eerder) — fallback voor
// de trend zolang temp_geschiedenis nog niet vol is.
float vorige_temperatuur;
// Ringbuffer met de laatste TREND_VENSTER metingen, voor een minder
// ruisgevoelige inschatting van de afkoelsnelheid bij een capture dan een
// enkele tick-tot-tick delta — zie AFSCHAAL_KD hierboven.
float temp_geschiedenis[TREND_VENSTER] = {};
int temp_geschiedenis_index = 0;
bool temp_geschiedenis_vol = false;
// Doeltemperatuur voor de regeling. Wordt herijkt zodra koude_vraag/
// warmte_vraag wegvalt (huidige temperatuur wordt het nieuwe doel — "hou vast
// waar we zijn") en persistent gemaakt in NVS, zodat een herstart hem niet
// kwijtraakt. Bij een lege NVS (allereerste boot) DOEL_TEMPERATUUR_FALLBACK_C.
float doel_temperatuur;
int koude_stand;
int warmte_stand;
int demp_stand_dender = 0;
RotoPdStatus huidige_pdo_status = RotoPdStatus::GEEN_PDO;
// RotoPD-communicatie onbetrouwbaar (GEEN_PDO/PDO_OVERFLOW) — zie check_fout().
bool in_fout = false;
// Was de fout t.o.v. doel_temperatuur de vorige keer groter dan
// GROTE_FOUT_DREMPEL_C? Voor het detecteren van het loslaten van een eigen
// (niet door koude_vraag/warmte_vraag getriggerde) grote-fout-episode, zie
// bepaalAansturing().
bool was_grote_fout = false;
// Laatst toegepaste richting (MODE_KOELEN/MODE_VERWARMEN/MODE_UIT), voor
// pasAansturingToe() om te zien of relais/LED daadwerkelijk moeten wisselen.
// -1 = nog niets toegepast (vlak na boot).
int huidige_richting = -1;
// Integraal-opbouw van de PID, in mV.
float pid_integraal_mV = 0;
// Eigen, trage klok specifiek voor de I-opbouw (zie bepaalAansturing()) —
// losgekoppeld van hoe vaak bepaalAansturing() zelf wordt aangeroepen. Die
// laatste mag nu vaker (vraagTick(), elke ~1s) om snel op een grote fout te
// reageren, maar PID_KI_PER_TICK is getuned op ~60s per opbouw-stap; zonder
// deze eigen klok zou de integraal 60x te snel oplopen.
unsigned long next_pid_tick = 0;
// Eigen, tragere klok specifiek voor de "Aansturing: ..."-logregel in
// bepaalAansturing(). Sinds vraagTick() elke ~1s toepast (zie hierboven)
// zou dit anders elke seconde opnieuw printen — bij een aanhoudende grote
// fout (bv. tijdens deze AVS-diagnose) genoeg volume om de BLE-verbinding
// te verstoppen (bufferophoping/herhaling). Loggen mag trager dan toepassen.
unsigned long next_log_tick = 0;
// Welke LED het vermogen-knipperpatroon toont: LEDBLAUW bij koelen, LEDGEEL
// bij verwarmen, -1 = geen (bv. FOUT/UIT). pasAansturingToe() zet 'm bij
// elke richtingwissel en wist het knipperplan van de vorige LED, anders
// blijft die zijn laatste patroon herhalen (ledTick() past dat elke 200ms
// opnieuw toe, los van een eenmalige ledAan()/ledUit()).
int actieveLed = -1;

struct ledPlan {
  int led;
  int deel;
  bool aan;
};
struct ledPlan ledPlannen[4];
int aantalLedPlannen = 0;
int ledMoment = 0;

void ledTick()
{
  for (int ledPlanI = 0; ledPlanI < aantalLedPlannen; ledPlanI++) {
    if (ledMoment > ledPlannen[ledPlanI].deel && ledPlannen[ledPlanI].aan) {
      ledUit(ledPlannen[ledPlanI].led);
      ledPlannen[ledPlanI].aan = false;
    }
    if (ledMoment <= ledPlannen[ledPlanI].deel && !ledPlannen[ledPlanI].aan) {
      ledAan(ledPlannen[ledPlanI].led);
      ledPlannen[ledPlanI].aan = true;
    }
  }
  if (ledMoment++ > 10)
    ledMoment = 1;
}

/* deel: 0 .. 10, 0 altijd uit, 10 altijd aan, 1 1/10 aan, etc
 */
void setLedPlan(int led, int deel)
{
  int ledPlanI;

  for (ledPlanI = 0; ledPlannen[ledPlanI].led != led && ledPlanI < aantalLedPlannen; ledPlanI++) {}
  if (ledPlanI < sizeof(ledPlannen)/sizeof(ledPlan)) {
    ledPlannen[ledPlanI].led = led;
    ledPlannen[ledPlanI].deel = deel;
    if (ledPlanI == aantalLedPlannen)
      aantalLedPlannen++;
  }
}

void initLedPlannen()
{
  for (auto & ledPlanI : ledPlannen) {
    ledPlanI.led = -1;
    ledPlanI.deel = 0;
    ledPlanI.aan = false;
  }
}

void stroomUit()
{
  console.println("stroomUit");
  usbpd.outputUit();
}

void testLed(int led)
{
    ledAan(led);
    delay(500);
    ledUit(led);
}

// Zet de fan-PWM-lijn op een percentage (0-100). analogWrite() gebruikt hier
// 8-bit resolutie (0-255) op de frequentie die met analogWriteFrequency()
// in setup() is ingesteld (25kHz, Noctua/Intel 4-pin PWM-spec).
void stelFanSnelheid(int percentage)
{
  if (percentage < 0) percentage = 0;
  if (percentage > 100) percentage = 100;
  analogWrite(FAN_PWM, (int) (percentage * 255L / 100));
}

void temperatuurTick()
{
  vorige_temperatuur = huidige_temperatuur;
  sensors.requestTemperatures();
  huidige_temperatuur = sensors.getTempC(temperatuurMeter);
  temp_geschiedenis[temp_geschiedenis_index] = huidige_temperatuur;
  temp_geschiedenis_index = (temp_geschiedenis_index + 1) % TREND_VENSTER;
  if (temp_geschiedenis_index == 0)
    temp_geschiedenis_vol = true;
  console.print(huidige_temperatuur);
  console.print("C\n");
}

struct Aansturing {
  int richting;             // MODE_KOELEN, MODE_VERWARMEN of MODE_UIT
  unsigned int voltage_mV;
  unsigned int current_mA;
  unsigned int fan_percentage;
  unsigned int vermogen_percentage;  // ingezet vermogen t.o.v. maximaal (0-100), voor de richting-LED
};

// Startwaarde voor de integraal bij het verlaten van vol vermogen (naar
// vasthouden): de afkoelsnelheid over de laatste TREND_VENSTER metingen
// geeft een realistischere schatting dan vanaf 0 beginnen, begrensd op een
// fractie van het bereik om niet meteen te verzadigen (zie AFSCHAAL_KD/
// AFSCHAAL_MAX_FRACTIE hierboven). Gebruikt door zowel het loslaten van een
// externe vraag (vraagTick()) als een eigen grote-fout-episode
// (bepaalAansturing()).
float berekenAfschaalSeed(unsigned int voltageMax)
{
  float trend = temp_geschiedenis_vol
      ? (huidige_temperatuur - temp_geschiedenis[temp_geschiedenis_index]) / TREND_VENSTER
      : (huidige_temperatuur - vorige_temperatuur);
  float seed = -AFSCHAAL_KD * trend;
  float seed_grens = (voltageMax - VOLTAGE_MIN_MV) * AFSCHAAL_MAX_FRACTIE;
  if (seed > seed_grens) seed = seed_grens;
  if (seed < -seed_grens) seed = -seed_grens;
  return seed;
}

// PI-regeling op de laatst gemeten temperatuur, bepaalt zowel richting als
// vermogen. Richting volgt uit het TEKEN van de fout (positief = te warm =
// koelen) — niet meer rechtstreeks uit koude_vraag/warmte_vraag. Geen
// kunstmatige dode zone nodig om klapperen te voorkomen: na koelen loopt de
// temperatuur vanzelf weer wat op, na verwarmen vanzelf weer wat terug, en de
// regel-cadans (~elke 60s) is toch al traag. De I-term compenseert het
// P-only euvel dat het voltage te vroeg terugzakt naar een niveau dat de
// bereikte temperatuur niet kan handhaven. Windup wordt simpel geklemd op
// het volledige bruikbare spanningsbereik — geen conditionele/
// back-calculation anti-windup; als de logs alsnog forse overshoot laten
// zien is dat de volgende tuningstap.
Aansturing bepaalAansturing()
{
  if (koude_stand && warmte_stand)
    return { MODE_UIT, 0, 0, 0, 0 };  // tegenstrijdige vraag — veiligheid

  // Bovenkant komt uit de PDO (AVS-max, zie regUSBCPow::leesMaxVoltage). 0
  // zolang de PDO-lijst nog niet bekend is; dan nog even niets doen.
  // Onderkant TIJDELIJK vast (i.p.v. usbpd.leesMinVoltage(), PPS-min,
  // geadverteerd als 100mV): logs lieten zien dat de bron sub-volt
  // PPS-aanvragen negeert (VREQ bleef op 5V hangen). Nog aan het tunen
  // (tussen 5-10V) welke vloer de bron wél altijd honoreert, vóór we
  // leesMinVoltage() weer vertrouwen.
  unsigned int voltageMin = VOLTAGE_MIN_MV;
  unsigned int voltageMax = usbpd.leesMaxVoltage();
  if (voltageMax <= voltageMin) {
    console.println("Aansturing: PDO-bereik nog niet bekend, wacht");
    return { MODE_UIT, 0, 0, 0, 0 };
  }

  float fout = huidige_temperatuur - doel_temperatuur;  // positief = te warm
  bool grote_fout = fabs(fout) > GROTE_FOUT_DREMPEL_C;
  // Alleen puur intern (geen externe vraag actief) bijhouden voor de
  // afschaal-seed hieronder. Zonder deze uitsluiting zou dit overlappen met
  // vraagTick()'s eigen seed op het moment dat een externe vraag loslaat:
  // doel_temperatuur is dan al vers bijgewerkt (±1.0°C-offset), dus fout
  // duikt op datzelfde moment toch al onder de drempel — een tweede,
  // overbodige (zij het onschadelijke) reseed met dezelfde waarde.
  bool eigen_grote_fout = grote_fout && !koude_stand && !warmte_stand;

  if (was_grote_fout && !eigen_grote_fout) {
    // Terugkeer binnen de tolerantie ná een eigen grote-fout-episode (dus
    // niet via koude_vraag/warmte_vraag losgelaten — dat wordt al apart
    // afgehandeld in vraagTick()). Zelfde afschaal-seed, maar geen
    // doel_temperatuur-wijziging: dat blijft ons eigen, al bestaande ijkpunt.
    pid_integraal_mV = berekenAfschaalSeed(voltageMax);
  }
  was_grote_fout = eigen_grote_fout;

  // Tijdens actieve vraag ÓF een eigen grote fout: rechtstreeks vol vermogen
  // (spanning én ventilator), geen fijnregeling. doel_temperatuur is tijdens
  // actieve vraag niet betrouwbaar (kan nog op de fallback staan, of op een
  // oude capture die niets met de huidige vraag te maken heeft) en zou de
  // fysieke vraag kunnen tegenwerken i.p.v. volgen; bij een eigen grote fout
  // is de richting gewoon die van de fout zelf. Grote fouten via P+I
  // afhandelen gaf te veel overshoot-risico — vandaar deze eigen drempel,
  // zodat dat ook geldt zonder aangesloten vriezerregelaar.
  if (koude_stand || warmte_stand || grote_fout) {
    int richting = koude_stand ? MODE_KOELEN
                  : warmte_stand ? MODE_VERWARMEN
                  : (fout > 0 ? MODE_KOELEN : MODE_VERWARMEN);
    unsigned long now = millis();
    if (now > next_log_tick) {
      next_log_tick = now + 5000;
      console.print("[");
      console.print(now);
      console.print("] Aansturing: ");
      console.print((koude_stand || warmte_stand) ? "actieve vraag" : "grote fout");
      console.print(" -> ");
      console.print(richting == MODE_KOELEN ? "KOELEN " : "VERWARMEN ");
      console.println(voltageMax);
    }
    return { richting, voltageMax, (unsigned int) PID_STROOM_MA, 100, 100 };
  }

  // Spanning schaalt over het hele bereik (floor..max); de ventilator is
  // geen "eerste hendel" meer maar volgt hetzelfde vermogen_percentage via
  // een eigen comfortband (zie FAN_VLOER/FAN_COMFORT hieronder). De integraal
  // wordt bij een capture al gezinigd met de recente afkoelsnelheid (zie
  // vraagTick()) i.p.v. hier vanaf 0 te beginnen.
  float grens = voltageMax - voltageMin;

  // Alleen de I-opbouw zelf aan de trage 60s-klok binden (zie
  // next_pid_tick hierboven) — bepaalAansturing() als geheel mag vaker
  // aangeroepen worden, de P-term en de rest van deze functie zijn een
  // pure functie van de actuele fout en hebben geen cadans-eis.
  unsigned long now = millis();
  if (now > next_pid_tick) {
    next_pid_tick = now + 60000;
    pid_integraal_mV += PID_KI_PER_TICK * fout;
    if (pid_integraal_mV < -grens) pid_integraal_mV = -grens;
    if (pid_integraal_mV > grens) pid_integraal_mV = grens;
  }

  float correctie = PID_KP * fout + pid_integraal_mV;  // signed
  if (correctie < -grens) correctie = -grens;
  if (correctie > grens) correctie = grens;

  int richting = (correctie >= 0) ? MODE_KOELEN : MODE_VERWARMEN;
  float magnitude = fabs(correctie);
  unsigned int voltage = voltageMin + (unsigned int) magnitude;
  unsigned int vermogen_percentage = (unsigned int) round(magnitude / grens * 100);

  // Ventilator: comfortband. Nooit onder FAN_VLOER (geen meerwaarde),
  // comfortabel tot FAN_COMFORT bij de eerste FAN_COMFORT_GRENS_PCT% van het
  // vermogen; pas daarboven wijkt comfort voor koelvermogen.
  unsigned int fan_percentage;
  if (vermogen_percentage <= FAN_COMFORT_GRENS_PCT) {
    fan_percentage = FAN_VLOER + (unsigned int) round(
        (float) vermogen_percentage / FAN_COMFORT_GRENS_PCT * (FAN_COMFORT - FAN_VLOER));
  } else {
    fan_percentage = FAN_COMFORT + (unsigned int) round(
        (float) (vermogen_percentage - FAN_COMFORT_GRENS_PCT) / (100 - FAN_COMFORT_GRENS_PCT) * (100 - FAN_COMFORT));
  }

  if (now > next_log_tick) {
    next_log_tick = now + 5000;
    console.print("[");
    console.print(now);
    console.print("] Aansturing: fout=");
    console.print(fout);
    console.print("C  I=");
    console.print(pid_integraal_mV);
    console.print("mV -> ");
    console.print(richting == MODE_KOELEN ? "KOELEN " : "VERWARMEN ");
    console.print("fan=");
    console.print(fan_percentage);
    console.print("% spanning=");
    console.print(voltage);
    console.print("mV (");
    console.print(vermogen_percentage);
    console.println("% van max)");
  }

  return { richting, voltage, (unsigned int) PID_STROOM_MA, fan_percentage, vermogen_percentage };
}

// Past een Aansturing-beslissing toe op relais, PDO en LED. Relais/LED
// wisselen alleen bij een daadwerkelijke richtingwissel (voorkomt onnodig
// relaisgeklik elke tick); de spanningsaanvraag gaat wél elke keer opnieuw
// de deur uit, ook na herstel van een RotoPD-fout (datasheet: na een trip
// moet de host sowieso opnieuw een PD_REQMSG sturen om te hervatten).
void pasAansturingToe(Aansturing a)
{
  if (a.richting != huidige_richting) {
    if (actieveLed >= 0)
      setLedPlan(actieveLed, 0);

    switch (a.richting) {
      case MODE_KOELEN:
        digitalWrite(RELAIS1, HIGH);
        digitalWrite(RELAIS2, HIGH);
        actieveLed = LEDBLAUW;
        ledAan(LEDBLAUW);
        ledUit(LEDGEEL);
        usbpd.outputAan();
        break;
      case MODE_VERWARMEN:
        digitalWrite(RELAIS1, LOW);
        digitalWrite(RELAIS2, LOW);
        actieveLed = LEDGEEL;
        ledUit(LEDBLAUW);
        ledAan(LEDGEEL);
        usbpd.outputAan();
        break;
      default:  // MODE_UIT
        digitalWrite(RELAIS1, HIGH);
        digitalWrite(RELAIS2, HIGH);
        actieveLed = -1;
        ledUit(LEDBLAUW);
        ledUit(LEDGEEL);
        stroomUit();
        stelFanSnelheid(0);
        break;
    }
    huidige_richting = a.richting;
  }

  if (a.richting == MODE_UIT)
    return;

  usbpd.setVoltage(a.voltage_mV, a.current_mA);
  stelFanSnelheid(a.fan_percentage);
  setLedPlan(actieveLed, round(a.vermogen_percentage / 10.0)+0.1);
}

// Leest de fysieke vriezerregelaar en reageert op een gewijzigde vraag.
// Bij het wegvallen van een actieve vraag (1->0) is het doel bereikt: de
// huidige temperatuur wordt het nieuwe doel_temperatuur ("hou vast waar we
// zijn"), persistent in NVS zodat een herstart dat niet kwijtraakt. Elke
// wijziging (ook het aangaan van een vraag) laat de aansturing meteen
// herevalueren — anders zou een nieuwe/wegvallende vraag pas op de
// eerstvolgende regel-tick (~60s) doorwerken.
void vraagTick()
{
  int vorige_koude_stand = koude_stand;
  int vorige_warmte_stand = warmte_stand;
  koude_stand = !digitalRead(KOUDE_VRAAG);
  warmte_stand = !digitalRead(WARMTE_VRAAG);

  if (koude_stand != vorige_koude_stand || warmte_stand != vorige_warmte_stand) {
    bool was_actief = vorige_koude_stand || vorige_warmte_stand;
    bool nu_actief = koude_stand || warmte_stand;
    if (was_actief && !nu_actief) {
      doel_temperatuur = huidige_temperatuur - (vorige_koude_stand ? +1.0 : -1.0);
      // Startwaarde voor de integraal i.p.v. een reset naar 0 — zie
      // berekenAfschaalSeed(). Dooft niet uit, de normale I-opbouw stelt 'm
      // verder bij op de daadwerkelijke fout.
      pid_integraal_mV = berekenAfschaalSeed(usbpd.leesMaxVoltage());
      prefs.putFloat("doelC", doel_temperatuur);
      console.print("Doeltemperatuur bijgewerkt: ");
      console.print(doel_temperatuur);
      console.println("C");
    }
  }

  // Elke ~1s opnieuw toepassen, niet alleen bij een gewijzigde vraag: een
  // actieve vraag/grote fout loopt via bepaalAansturing()'s bang-bang-tak,
  // die de PID-integraal niet aanraakt (zie next_pid_tick daar) — vaker
  // aanroepen dan de trage 60s-cadans is dus veilig, en voorkomt dat de
  // eerste/eerstvolgende sturing tot een volle minuut op zich laat wachten.
  pasAansturingToe(bepaalAansturing());
}

// Bewaakt alleen nog de RotoPD-communicatie — geen modus-stack meer nodig:
// er is nog maar één "normale" toestand om naar terug te keren (de
// regelroutines pakken vanzelf de draad weer op, zie pasAansturingToe()'s
// commentaar over de verse PD_REQMSG na herstel). true = deze loop-iteratie
// de regelroutines overslaan.
bool check_fout()
{
  bool rotopd_fout = (huidige_pdo_status == RotoPdStatus::GEEN_PDO ||
                      huidige_pdo_status == RotoPdStatus::PDO_OVERFLOW);

  if (rotopd_fout && !in_fout) {
    in_fout = true;
    stroomUit();
    digitalWrite(RELAIS1, HIGH);
    digitalWrite(RELAIS2, HIGH);
    if (actieveLed >= 0)
      setLedPlan(actieveLed, 0);
    actieveLed = -1;
    huidige_richting = -1;  // volgende pasAansturingToe() moet alles opnieuw zetten
    ledUit(LEDBLAUW);
    ledUit(LEDGEEL);
    ledAan(LEDROOD);
  } else if (!rotopd_fout && in_fout) {
    in_fout = false;
    ledUit(LEDROOD);
  }

  return in_fout;
}

void wifiOtaSetup()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  console.print("Verbinden met WiFi");
  for (int pogingen = 0; WiFi.status() != WL_CONNECTED && pogingen < 20; pogingen++) {
    delay(500);
    console.print(".");
  }
  console.println();
  if (WiFi.status() == WL_CONNECTED) {
    console.print("WiFi verbonden, IP: ");
    console.println(WiFi.localIP());
  } else {
    console.println("Geen WiFi-verbinding, ga verder zonder OTA");
    return;
  }

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    // Flash wissen/schrijven tijdens OTA kan langer blokkeren dan de 8s
    // watchdog-timeout, zonder kans om esp_task_wdt_reset() aan te roepen.
    // Tijdelijk afmelden voorkomt een reset midden in de overdracht.
    esp_task_wdt_delete(NULL);
    console.println("OTA update gestart");
  });
  ArduinoOTA.onEnd([]() {
    esp_task_wdt_add(NULL);
    console.println("OTA update klaar");
  });
  ArduinoOTA.onError([](ota_error_t fout) {
    esp_task_wdt_add(NULL);
    console.print("OTA fout: ");
    console.println((int) fout);
  });
  ArduinoOTA.begin();
}

void setup() {
  Wire.begin();
  usbpd.begin(ROTOPD_INT);  // print o.a. de INA238 MANUFACTURER_ID-check

  prefs.begin("layzee", false);
  doel_temperatuur = prefs.getFloat("doelC", DOEL_TEMPERATUUR_FALLBACK_C);

  Serial.begin(9600);
  delay(1000); //Ensure everything got enough time to bootup

  // BLE eerst opstarten: de TX-buffer/characteristic moeten al bestaan
  // voordat wifiOtaSetup() begint te loggen, anders wordt die trace
  // stilletjes weggegooid (write() heeft dan nog geen _txKenmerk).
  bleSerial.begin(OTA_HOSTNAME);
  wifiOtaSetup();

  pinMode(LEDBLAUW, OUTPUT);
  pinMode(LEDGEEL, OUTPUT);
  pinMode(LEDGROEN, OUTPUT);
  pinMode(LEDROOD, OUTPUT);

  pinMode(KOUDE_VRAAG, INPUT_PULLUP);
  pinMode(WARMTE_VRAAG, INPUT_PULLUP);
  pinMode(RELAIS1, OUTPUT);
  pinMode(RELAIS2, OUTPUT);
  digitalWrite(RELAIS1, LOW);
  digitalWrite(RELAIS2, LOW);

  testLed(LEDBLAUW);
  testLed(LEDGEEL);
  testLed(LEDGROEN);
  testLed(LEDROOD);

  analogWriteFrequency(FAN_PWM_FREQUENTIE);
  stelFanSnelheid(0);  // bepaalAansturing()/pasAansturingToe() nemen het vanaf hier over

  console.print("doel_temperatuur (uit NVS): ");
  console.print(doel_temperatuur);
  console.println("C");

  // usbpd.srcpdo();
  // usbpd.printTo(console);
  sensors.begin();
  console.println("sensors.begin: klaar");
  numberOfSensors = sensors.getDeviceCount();
  if (numberOfSensors == 1 && sensors.getAddress(temperatuurMeter, 0)) {
    console.print("numberOfSensors: ");
    console.println(numberOfSensors);
    sensors.setResolution(temperatuurMeter, 12);
  } else {
    console.print("numberOfSensors: ");
    console.println(numberOfSensors);
    console.println("Geen of te veel temperatuurmeters\n");
  }
  digitalWrite(RELAIS1, HIGH);
  digitalWrite(RELAIS2, HIGH);

  esp_task_wdt_init(8, true);  // 8 seconden, reset bij timeout
  esp_task_wdt_add(NULL);      // volg de loop-task
}

unsigned long next_vraagTick = 0;
unsigned long next_temperatuurTick = 0;
unsigned long next_ledTick = 0;
// Los van temperatuurTick()'s trage 60s-cadans: sturing reageert sinds
// vraagTick() al binnen ~1s, maar printStatus() zelf leest alleen al
// gecachte waarden (geen eigen I2C) — geen reden om de rapportage daarvan
// nog aan diezelfde 60s vast te binden. Zo kun je ook direct na het sturen
// meten, i.p.v. tot een minuut wachten.
unsigned long next_status_tick = 0;

void loop() {
  if (testSucces)
  {
    unsigned long now = millis();
    ArduinoOTA.handle();
    huidige_pdo_status = usbpd.handleWork();
    bleSerial.tick();
    esp_task_wdt_reset();
    // TIJDELIJK, voor de AVS-diagnose: check_fout() nog wel aanroepen (rode
    // LED/in_fout blijven zichtbaar), maar niet meer als poort gebruiken om
    // de rest over te slaan. Anders stopt alle meting/print zodra de FAULT
    // toeslaat — precies het moment dat we willen zien. Gevolg: de regellus
    // blijft de mislukte AVS-aanvraag steeds opnieuw proberen i.p.v. veilig
    // uit te blijven staan — bewust, voor deze diagnostische sessie, niet
    // het gewenste eindgedrag.
    check_fout();
    if (now > next_vraagTick) {
      next_vraagTick = now + 1000;
      vraagTick();
    }
    if (now > next_temperatuurTick) {
      next_temperatuurTick = now + 60000;
      temperatuurTick();
      pasAansturingToe(bepaalAansturing());
    }
    if (now > next_status_tick) {
      next_status_tick = now + 5000;
      usbpd.printStatus(console);
    }
    if (now > next_ledTick) {
      next_ledTick = now + 200;
      ledTick();
    }
  } else
  {

  }
}