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
#define PID_STROOM_MA 5000        // vast voor nu, alleen de spanning wordt geregeld — Peltiers verdragen tot 7000mA, 3000 bleek te krap (OCP-trip bij een richtingwissel)
// ADRC (Active Disturbance Rejection Control) i.p.v. klassieke P+I: de oude
// integraal (PID_KI_PER_TICK) bouwde vast op een 60s-cadans op en liep
// daardoor structureel achter een verstoring aan die zelf ook voortdurend
// verandert — met name de buitentemperatuur, die zowel de warmtelek naar
// binnen als het Peltier-rendement beïnvloedt. Een Extended State Observer
// (ESO) schat i.p.v. een trage integraal continu een "totale verstoring"
// (z2, C/s) — ongeacht of die van warmtelek, Peltier-nietlineariteit of
// iets anders komt — en de regelwet compenseert die schatting direct.
// Eerste-orde model: dT/dt = -adrc_b0*correctie + verstoring(t). adrc_b0 is
// GEEN vaste constante meer maar een lopend-gemiddelde schatting (zie
// globals hieronder + kalibreerB0()/seedObserver()): elke vol-vermogen-
// episode van voldoende duur (ADRC_B0_KALIBRATIE_MIN_MS) geeft een verse,
// directe (correctie, afkoelsnelheid)-meting — geen enkele veiligheidsrisico's
// zoals bij een drone-parameter, dus niets op tegen "gewoon even vol gas
// proberen en meten" i.p.v. voorzichtig alleen uit oude logs schatten.
// Apart bijgehouden voor koelen en verwarmen (adrc_b0_koelen/
// adrc_b0_verwarmen): een Peltier is niet symmetrisch — de resistieve
// (I²R) verliezen komen altijd op de warme kant terecht, wat verwarmen
// helpt maar koelen tegenwerkt, dus verwarmen is doorgaans effectiever per
// mV. Eén gedeelde b0 (in de praktijk vrijwel uitsluitend uit koelen-
// episodes gekalibreerd, want die komen veel vaker voor) gaf stelselmatig
// te grote verwarmen-correcties.
// Fallback/startwaarde (voor de allereerste boot, of zolang NVS nog niets
// geleerd heeft, voor beide richtingen gelijk) uit tools/out1 (een
// meerdere uren durende bang-bang-koelperiode op vol vermogen,
// gevraagd=28000mV dus correctie=grens=18000mV constant): t=665624ms
// T=4.31C -> t=27606072ms T=-2.75C, dus ~-2.62e-4 C/s bij 18000mV ->
// ~1.46e-8 C/s per mV. Dat verwaarloost de warmtelek (verstoring warmt
// juist op, camoufleert een deel van het koeleffect), dus de werkelijke
// waarde ligt vermoedelijk iets hoger — vandaar naar boven afgerond.
#define ADRC_B0_FALLBACK 0.00000002f   // 2e-8 C/s per mV
// Alleen een episode van minstens deze duur vertrouwen om adrc_b0 bij te
// stellen — korter is te ruisgevoelig (sensor-resolutie 0.0625C/tick).
#define ADRC_B0_KALIBRATIE_MIN_MS (5UL * 60UL * 1000UL)  // vijf minuten
// Gewicht van een verse meting in het lopend gemiddelde — laag genoeg om
// niet op één (mogelijk rommelige) episode te schieten, hoog genoeg om
// binnen een paar episodes echt te leren i.p.v. oneindig na te ijlen.
#define ADRC_B0_LEERSNELHEID 0.3f
// omega_o (waarnemer) / omega_c (regelaar), rad/s. Vuistregel omega_o =
// 3-5x omega_c. Met een ESO-stap T=60s (zie ADRC_ESO_TICK_MS) moet
// omega_o*T ruim onder ~0.3 blijven (anders is de Euler-discretisatie
// hieronder geen goede benadering meer van de continue ESO) — onderstaande
// startwaarden geven omega_o*T ≈ 0.2. Beide nog te tunen op hardware, net
// zoals PID_KP/PID_KI_PER_TICK hiervoor iteratief getuned zijn.
#define ADRC_OMEGA_O 0.0033f   // PLACEHOLDER, te tunen
#define ADRC_OMEGA_C 0.00067f  // PLACEHOLDER, te tunen (~omega_o/5)
// Zelfde cadans-eis als de oude next_pid_tick: de ESO-update mag niet
// vaker dan een nieuwe temperatuurmeting (temperatuurTick(), ~60s) — vaker
// zou de waarnemer alleen laten "updaten" op een meting die niet ververst is.
#define ADRC_ESO_TICK_MS 60000UL
// Bij het loslaten van een vraag (koude_vraag/warmte_vraag) valt de fout
// terug naar ~0 — zonder meer zou de regeling dan in één klap van vol
// vermogen naar bijna niets springen, en dat bleek in de praktijk veel te
// weinig om de temperatuur vast te houden (spanning kroop na 35 minuten nog
// maar tot 13500mV, tegen 28000mV tijdens actief koelen). seedObserver()
// zet de afkoelsnelheid over de laatste TREND_VENSTER metingen (dus vlak
// vóór het loslaten, bij vol vermogen) om in een startwaarde voor de ESO
// (z1/z2) — die dooft niet vanzelf uit maar wordt door de normale
// ESO-opbouw verder bijgesteld op het daadwerkelijke verloop.
// ADRC_Z2_SEED_MAX_FRACTIE begrenst alleen die STARTSCHATTING (niet de
// doorlopende ESO-opbouw erna) tot een fractie van wat vol vermogen zou
// verklaren, zodat er nog ruimte overblijft om verder bij te sturen i.p.v.
// meteen te verzadigen — zelfde reden als de oude AFSCHAAL_MAX_FRACTIE.
#define ADRC_Z2_SEED_MAX_FRACTIE 0.5f
#define TREND_VENSTER 10
#define VOLTAGE_MIN_MV 10000  // vloer voor de regeling — zie toelichting bij bepaalAansturing()
// NB: outputUit() (zie stroomUit()) zet alleen VOUTCTL uit (het lokale pad
// naar onze uitgang) en laat het lopende PD-contract (VREQ/IREQ) bewust
// ongemoeid — een eerdere poging om dat via setVoltage() ook terug te
// parkeren dwong een AVS->PPS-profielwissel af die op echte hardware
// herhaalde OCP-fouten triggerde (de bron trok daarna zijn AVS/28000mV-
// aanbod zelfs blijvend in). De "gevraagd"-regel in de log kan daardoor
// tijdens stroomUit() nog het oude, niet meer actieve verzoek tonen — dat
// is cosmetisch, en weegt niet op tegen dat risico.
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
// enkele tick-tot-tick delta — zie seedObserver() verderop.
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
// BLE-advertentie kon na een disconnect niet opnieuw starten (zie
// bleSerial.h) — blijft staan totdat er weer daadwerkelijk een client
// verbindt (isVerbonden()), niet zomaar na één ledTick-cyclus: zie
// werkRoodLedBij().
bool ble_fout = false;
// Was de fout t.o.v. doel_temperatuur de vorige keer groter dan
// GROTE_FOUT_DREMPEL_C? Voor het detecteren van het loslaten van een eigen
// (niet door koude_vraag/warmte_vraag getriggerde) grote-fout-episode, zie
// bepaalAansturing().
bool was_grote_fout = false;
// Laatst toegepaste richting (MODE_KOELEN/MODE_VERWARMEN/MODE_UIT), voor
// pasAansturingToe() om te zien of relais/LED daadwerkelijk moeten wisselen.
// -1 = nog niets toegepast (vlak na boot).
int huidige_richting = -1;
// ESO-toestand (zie adrc_b0/ADRC_OMEGA_O/ADRC_OMEGA_C): z1 = gefilterde
// temperatuurschatting (C), z2 = geschatte "lumped disturbance" (C/s) —
// alles wat het simpele b0-model niet verklaart (thermische massa,
// omgevingstemperatuur, Peltier-nietlineariteit). Vervangt de enkele
// pid_integraal_mV door twee toestanden.
float adrc_z1 = 0;
float adrc_z2 = 0;
// Lopend-gemiddelde schatting van de controle-effectiviteit (C/s per mV) —
// apart voor koelen en verwarmen, want een Peltier is niet symmetrisch: de
// resistieve (I²R) verliezen in het element komen altijd op de warme kant
// terecht — dat helpt verwarmen (gratis extra warmte) maar werkt koelen
// tegen (parasitaire warmte die de gepompte koeling deels tenietdoet).
// Verwarmen is dus doorgaans effectiever per mV dan koelen; één gedeelde b0
// (gekalibreerd op vrijwel uitsluitend koelen-episodes, in de praktijk) gaf
// stelselmatig te grote verwarmen-correcties. Zie
// ADRC_B0_FALLBACK/ADRC_B0_KALIBRATIE_MIN_MS/ADRC_B0_LEERSNELHEID hierboven
// en kalibreerB0()/seedObserver() verderop. In setup() overschreven door de
// NVS-waarden indien een eerdere sessie al iets geleerd heeft.
float adrc_b0_koelen = ADRC_B0_FALLBACK;
float adrc_b0_verwarmen = ADRC_B0_FALLBACK;
// Nulmeting van de lopende vol-vermogen-episode (zie bepaalAansturing()'s
// bang-bang-tak) — puur voor de adrc_b0-kalibratie in kalibreerB0(), los
// van de temp_geschiedenis-ringbuffer (die anders bij een korte episode
// vervuild zou zijn met metingen van vóór de episode).
bool was_in_bangbang = false;
unsigned long bangbang_episode_start_tijd = 0;
float bangbang_episode_start_temp = 0;
// Richting die gold toen de lopende episode begon — een richtingwissel
// (koelen<->verwarmen) binnen een aaneengesloten bang-bang-periode moet de
// nulmeting óók resetten, anders vermengt seedObserver() twee episodes met
// tegengestelde u_vol_vermogen tot een onzinnige b0-meting.
int bangbang_episode_richting = -1;
// Laatste 'correctie' (mV, signed, positief=koelen) die daadwerkelijk is
// toegepast sinds de vorige ESO-update — nodig omdat die update het
// b0*u-model van die periode moet aftrekken van de waargenomen verandering.
// De oude PID had dit niet nodig (zijn wiskunde had geen geheugen van de
// eigen output nodig, enkel van de fout).
float laatst_toegepaste_correctie_mV = 0;
// Eigen, trage klok specifiek voor de ESO-update (zie bepaalAansturing()) —
// losgekoppeld van hoe vaak bepaalAansturing() zelf wordt aangeroepen. Die
// laatste mag nu vaker (vraagTick(), elke ~1s) om snel op een grote fout te
// reageren, maar de ESO krijgt pas echt nieuwe informatie bij een verse
// temperatuurmeting (temperatuurTick(), ~60s); zonder deze eigen klok zou
// de waarnemer 60x te snel "updaten" op een meting die niet ververst is.
unsigned long next_eso_tick = 0;
// Wachten-op-kentering: na het loslaten van koude_vraag/warmte_vraag (zie
// vraagTick()) staat alles uit (stroomUit()) en wordt er niet aangestuurd
// tot de meting daadwerkelijk van richting wisselt — pas dan is de
// thermische naijl (momentum van het net gestopte vol vermogen) voorbij en
// heeft de fijnregeling weer zinnige informatie. kentering_richting is de
// richting van de zojuist afgelopen episode (de kant waarin we nog naijl
// verwachten); huidige_richting zelf is dan al -1 (door stroomUit()).
bool wacht_op_kentering = false;
int kentering_richting = -1;
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

void setLedPlan(int led, int deel);  // forward-declaratie: ledTick() hieronder gebruikt 'm al, definitie staat verderop

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

void stelFanSnelheid(int percentage);  // forward-declaratie: stroomUit() hieronder gebruikt 'm al, definitie staat verderop

// Eén centrale, zelfstandige "alles uit"-actie: relais naar de veilige
// stand, PD-uitgang uit, ventilator uit, richting-LED's uit, en de
// boekhouding (actieveLed/huidige_richting) zo teruggezet dat de
// eerstvolgende pasAansturingToe() alles weer van scratch opbouwt. Eerder
// stond dit verspreid over check_fout() en pasAansturingToe()'s MODE_UIT-
// tak, allebei met hun eigen (net iets andere) subset — nu volstaat overal
// gewoon één aanroep van stroomUit() zelf.
void stroomUit()
{
  console.println("stroomUit");
  usbpd.outputUit();
  stelFanSnelheid(0);
  digitalWrite(RELAIS1, HIGH);
  digitalWrite(RELAIS2, HIGH);
  if (actieveLed >= 0)
    setLedPlan(actieveLed, 0);
  actieveLed = -1;
  huidige_richting = -1;
  setLedPlan(LEDBLAUW, 0);
  setLedPlan(LEDGEEL, 0);
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

// b0-kalibratie op de zojuist afgelopen vol-vermogen-episode (bangbang_
// episode_start_tijd/_temp, zie bepaalAansturing()'s bang-bang-tak) — geen
// vaste constante meer, maar een lopend gemiddelde dat elke voldoende lange
// episode bijstelt. Alleen vertrouwen als de episode minstens
// ADRC_B0_KALIBRATIE_MIN_MS duurde (korter is te ruisgevoelig, sensor-
// resolutie 0.0625C/tick) en er daadwerkelijk een bekende richting is
// toegepast (huidige_richting, niet vlak na boot). Dit is een koelkast, geen
// drone die bij een verkeerde parameter uit de lucht valt — een paar minuten
// "gewoon vol gas proberen en meten" geeft een prima directe indicatie,
// veiliger dan blind op oude logs vertrouwen.
//
// Los van de ESO-seed (seedObserver() hieronder) omdat de twee niet meer
// altijd op hetzelfde moment gebeuren: bij het loslaten van een externe
// vraag (vraagTick()) is deze kalibratie nog geldig (de episode is echt
// gebeurd), maar de ESO-seed wacht tot de kentering (zie wacht_op_kentering)
// — vandaar dat dit vóór stroomUit() moet, die huidige_richting op -1 zet.
void kalibreerB0(unsigned int voltageMax)
{
  float grens = (float) (voltageMax - VOLTAGE_MIN_MV);
  float u_vol_vermogen = 0;
  if (huidige_richting == MODE_KOELEN) u_vol_vermogen = grens;
  else if (huidige_richting == MODE_VERWARMEN) u_vol_vermogen = -grens;
  // anders (MODE_UIT/nog nooit toegepast): laat op 0 — kan alleen vlak na
  // boot optreden, vóór ooit een geldige Aansturing is toegepast.
  if (u_vol_vermogen == 0) return;

  unsigned long episode_duur_ms = millis() - bangbang_episode_start_tijd;
  if (episode_duur_ms < ADRC_B0_KALIBRATIE_MIN_MS) return;

  float episode_trend_C_per_s =
      (huidige_temperatuur - bangbang_episode_start_temp) / (episode_duur_ms / 1000.0f);
  // b0 = -trend/u: bij koelen is u>0 en trend (normaal) <0, bij verwarmen
  // andersom — deze formule geeft in beide gevallen een positieve b0.
  float b0_gemeten = -episode_trend_C_per_s / u_vol_vermogen;
  if (b0_gemeten <= 0) return;
  // Negatief zou fysisch onzinnig zijn (een verstoring die vol vermogen
  // volledig overheerst) — dan liever de oude waarde behouden dan 'm laten
  // omslaan naar iets onbruikbaars.

  bool was_koelen = huidige_richting == MODE_KOELEN;
  float &adrc_b0 = was_koelen ? adrc_b0_koelen : adrc_b0_verwarmen;
  adrc_b0 = (1.0f - ADRC_B0_LEERSNELHEID) * adrc_b0 + ADRC_B0_LEERSNELHEID * b0_gemeten;
  prefs.putFloat(was_koelen ? "adrcB0Koel" : "adrcB0Warm", adrc_b0);
  console.print(was_koelen ? "adrc_b0_koelen bijgesteld naar " : "adrc_b0_verwarmen bijgesteld naar ");
  console.print(adrc_b0, 10);
  console.print(" (meting: ");
  console.print(b0_gemeten, 10);
  console.println(")");
}

// Seedt de ESO (adrc_z1/adrc_z2) bij het verlaten van vol vermogen (naar
// vasthouden) — analoog aan de oude berekenAfschaalSeed(), zelfde
// TREND_VENSTER-ringbuffer, maar nu voor twee toestanden i.p.v. één
// integraal:
//  - z1 heeft geen zinnige geschiedenis uit de vol-vermogen-episode (er
//    viel toen niets te schatten, de uitkomst stond toch al vast) — start
//    'm gewoon op de laatste meting.
//  - z2 volgt uit "wat de temperatuur ECHT deed" min "wat het b0-model
//    voorspelde" bij de bekende, vaste sturing tijdens die episode (vol
//    vermogen, richting = huidige_richting — dat is op dit moment nog de
//    richting van de ZOJUIST AFGELOPEN episode, de aanroeper heeft 'm nog
//    niet bijgewerkt): z2 = trend_C_per_s + adrc_b0 * u_vol_vermogen
//    (u_vol_vermogen signed: +grens tijdens koelen, -grens tijdens
//    verwarmen — zelfde tekenconventie als 'correctie' hieronder).
// Gebruikt door het verlaten van een eigen grote-fout-episode
// (bepaalAansturing()) — daar blijft doel_temperatuur ongewijzigd, dus de
// trend tijdens de episode is nog steeds relatief aan hetzelfde doel en dus
// bruikbaar. Bij vraagTick()'s externe-vraag-loslaat-pad wordt doel_
// temperatuur WEL verzet (naar de huidige meting) en is deze trend niet meer
// bruikbaar — dat pad wacht in plaats daarvan op een kentering (zie
// wacht_op_kentering) en seedt dan direct met z2=0, zonder deze functie.
//
// Open tuningvraag: TREND_VENSTER (10 metingen = 10 minuten) was getuned
// voor de oude AFSCHAAL_KD-seed, niet specifiek voor deze z2-afleiding —
// eventueel te herzien op basis van bench-logging.
void seedObserver(unsigned int voltageMax)
{
  float trend_per_tick = temp_geschiedenis_vol
      ? (huidige_temperatuur - temp_geschiedenis[temp_geschiedenis_index]) / TREND_VENSTER
      : (huidige_temperatuur - vorige_temperatuur);
  float trend_C_per_s = trend_per_tick / (ADRC_ESO_TICK_MS / 1000.0f);

  float grens = (float) (voltageMax - VOLTAGE_MIN_MV);
  float u_vol_vermogen = 0;
  if (huidige_richting == MODE_KOELEN) u_vol_vermogen = grens;
  else if (huidige_richting == MODE_VERWARMEN) u_vol_vermogen = -grens;
  float adrc_b0 = huidige_richting == MODE_KOELEN ? adrc_b0_koelen : adrc_b0_verwarmen;

  float z2_seed = trend_C_per_s + adrc_b0 * u_vol_vermogen;
  float z2_seed_grens = grens * adrc_b0 * ADRC_Z2_SEED_MAX_FRACTIE;
  if (z2_seed > z2_seed_grens) z2_seed = z2_seed_grens;
  if (z2_seed < -z2_seed_grens) z2_seed = -z2_seed_grens;

  adrc_z1 = huidige_temperatuur;
  adrc_z2 = z2_seed;
  laatst_toegepaste_correctie_mV = u_vol_vermogen;
  next_eso_tick = millis() + ADRC_ESO_TICK_MS;  // niet meteen weer updaten met deze verse seed
}

// Neutrale ESO-seed na een kentering (zie wacht_op_kentering): geen trend-
// gok meer nodig — de kentering zelf is het bewijs dat de naijl van de
// afgelopen episode voorbij is, dus start gewoon schoon op de verse meting
// en laat de normale ESO-opbouw (beta1/beta2 op echte residuen) z2 vanaf
// hier leren.
void seedEsoNeutraal()
{
  adrc_z1 = huidige_temperatuur;
  adrc_z2 = 0;
  laatst_toegepaste_correctie_mV = 0;
  next_eso_tick = millis() + ADRC_ESO_TICK_MS;
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
    // afgehandeld in vraagTick()). doel_temperatuur wijzigt hier niet: dat
    // blijft ons eigen, al bestaande ijkpunt, dus de trend tijdens de
    // episode is nog steeds relatief aan hetzelfde doel en bruikbaar.
    kalibreerB0(voltageMax);
    seedObserver(voltageMax);
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
  bool in_bangbang_nu = koude_stand || warmte_stand || grote_fout;
  int bangbang_richting = koude_stand ? MODE_KOELEN
                : warmte_stand ? MODE_VERWARMEN
                : (fout > 0 ? MODE_KOELEN : MODE_VERWARMEN);
  if (in_bangbang_nu && (!was_in_bangbang || bangbang_richting != bangbang_episode_richting)) {
    // Nulmeting bij het INGAAN van een vol-vermogen-episode, of bij een
    // richtingwissel binnen een doorlopende episode — puur voor de
    // adrc_b0-kalibratie in seedObserver(), zie de globals hierboven.
    bangbang_episode_start_tijd = millis();
    bangbang_episode_start_temp = huidige_temperatuur;
    bangbang_episode_richting = bangbang_richting;
  }
  was_in_bangbang = in_bangbang_nu;
  unsigned long now = millis();
  if (in_bangbang_nu) {
    const char* richting_txt = bangbang_richting == MODE_KOELEN ? "koelen" : "verwarmen";
    if (now > next_log_tick)
    {
      next_log_tick = now + 5000;
      console.print("[");
      console.print(now);
      console.print("] Aansturing: ");
      console.print(richting_txt);
      console.print(", voltageMax: ");
      console.println(voltageMax);
    }
    return { bangbang_richting, voltageMax, (unsigned int) PID_STROOM_MA, 100, 100 };
  }

  // Spanning schaalt over het hele bereik (floor..max); de ventilator is
  // geen "eerste hendel" meer maar volgt hetzelfde vermogen_percentage via
  // een eigen comfortband (zie FAN_VLOER/FAN_COMFORT hieronder). De ESO
  // wordt bij een capture al gezinigd met de recente afkoelsnelheid (zie
  // seedObserver()) i.p.v. hier vanaf 0 te beginnen.
  float grens = voltageMax - voltageMin;

  // ==== ADRC-fijnregeling (Extended State Observer + regelwet) ====
  // De ESO-UPDATE zelf mag niet vaker dan elke ~60s (temperatuurTick()'s
  // cadans, ADRC_ESO_TICK_MS) — een nieuwe meting is de enige nieuwe
  // informatie die de waarnemer heeft; vaker bijwerken zou 'm alleen laten
  // "meebewegen" met een meting die toch niet ververst is.
  // bepaalAansturing() zelf mag wel vaker (vraagTick(), ~1s) om snel op een
  // net beëindigde grote-fout-episode te reageren — die snelheid raakt
  // alleen de TOEPASSING van de laatst geschatte z1/z2, niet de schatting
  // zelf.
  if (now > next_eso_tick) {
    next_eso_tick = now + ADRC_ESO_TICK_MS;

    float T_s = ADRC_ESO_TICK_MS / 1000.0f;
    float beta1 = 2.0f * ADRC_OMEGA_O;
    float beta2 = ADRC_OMEGA_O * ADRC_OMEGA_O;

    // Discrete Euler-stap op dT/dt = -adrc_b0*u + z2, met u = de correctie
    // die sinds de vorige ESO-update daadwerkelijk toegepast werd — het
    // TEKEN van die u bepaalt welke van de twee (koelen/verwarmen) b0's hier
    // van toepassing was.
    float residu = huidige_temperatuur - adrc_z1;  // meting minus voorspelling
    float b0_toegepast = laatst_toegepaste_correctie_mV >= 0 ? adrc_b0_koelen : adrc_b0_verwarmen;
    adrc_z1 += T_s * (adrc_z2 - b0_toegepast * laatst_toegepaste_correctie_mV + beta1 * residu);
    adrc_z2 += T_s * beta2 * residu;
  }

  // Regelwet: P op de fout t.o.v. het doel, uitgedrukt in de GEFILTERDE
  // schatting z1 (niet de rauwe meting — dat is precies het punt van de
  // ESO), plus rechtstreekse compensatie van de geschatte verstoring z2.
  // ADRC_OMEGA_C bepaalt hoe snel de fout mag wegregelen; adrc_b0 zet de
  // gewenste C/s-correctie om in mV. Welke van de twee b0's van toepassing
  // is hangt af van de richting van de correctie zelf — maar b0 is altijd
  // positief, dus het TEKEN van de teller alleen bepaalt de richting; delen
  // door de bijbehorende b0 verandert dat teken niet. Geen kip-en-ei-
  // probleem dus.
  float fout_op_z1 = adrc_z1 - doel_temperatuur;
  float teller = ADRC_OMEGA_C * fout_op_z1 + adrc_z2;
  float adrc_b0 = teller >= 0 ? adrc_b0_koelen : adrc_b0_verwarmen;
  float correctie = teller / adrc_b0;  // signed
  if (correctie < -grens) correctie = -grens;
  if (correctie > grens) correctie = grens;
  laatst_toegepaste_correctie_mV = correctie;  // input voor de VOLGENDE ESO-update

  int richting = (correctie >= 0) ? MODE_KOELEN : MODE_VERWARMEN;
  float magnitude = fabs(correctie);
  unsigned int voltage = voltageMin + (unsigned int) magnitude;
  unsigned int vermogen_percentage = (unsigned int) round(magnitude / grens * 100);
  // ==== einde ADRC-fijnregeling ====

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
    console.print("C z1=");
    console.print(adrc_z1);
    console.print("C z2=");
    console.print(adrc_z2 * 60.0f);   // C/min, leesbaarder dan C/s
    console.print("C/min b0u=");
    console.print(adrc_b0 * laatst_toegepaste_correctie_mV * 60.0f);  // C/min, modelbijdrage
    console.print("C/min -> ");
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
        usbpd.outputAan();
        break;
      case MODE_VERWARMEN:
        digitalWrite(RELAIS1, LOW);
        digitalWrite(RELAIS2, LOW);
        actieveLed = LEDGEEL;
        usbpd.outputAan();
        break;
      default:  // MODE_UIT
        stroomUit();  // regelt zelf relais/fan/LED's, zie de toelichting daar
        break;
    }
    huidige_richting = a.richting;
  }

  if (a.richting == MODE_UIT)
    return;

  usbpd.setVoltage(a.voltage_mV, a.current_mA);
  stelFanSnelheid(a.fan_percentage);
  setLedPlan(actieveLed, round(a.vermogen_percentage / 10.0)+1);
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
      // b0-kalibratie op de zojuist afgelopen episode is nog geldig (die
      // episode is echt gebeurd) — moet vóór stroomUit(), die huidige_
      // richting op -1 zet.
      kalibreerB0(usbpd.leesMaxVoltage());
      // De episode liep net op vol vermogen; door thermische traagheid
      // loopt de meting na het uitzetten nog even door in dezelfde richting
      // (naijl/momentum) voordat 'm daadwerkelijk kantelt. Tot die kentering
      // is elke trend-schatting nog steeds die naijl, niet de verstoring die
      // geldt bij vasthouden op het nieuwe doel — dus nu niets aansturen,
      // alleen het doel vastleggen en afwachten (zie temperatuurTick()).
      kentering_richting = huidige_richting;
      wacht_op_kentering = true;
      doel_temperatuur = huidige_temperatuur;
      prefs.putFloat("doelC", doel_temperatuur);
      console.print("Doeltemperatuur bijgewerkt: ");
      console.print(doel_temperatuur);
      console.println("C, wacht op kentering");
      stroomUit();
    }
  }

  // Elke ~1s opnieuw toepassen, niet alleen bij een gewijzigde vraag: een
  // actieve vraag/grote fout loopt via bepaalAansturing()'s bang-bang-tak,
  // die de ESO niet aanraakt (zie next_eso_tick daar) — vaker aanroepen
  // dan de trage 60s-cadans is dus veilig, en voorkomt dat de
  // eerste/eerstvolgende sturing tot een volle minuut op zich laat wachten.
  // Niet tijdens in_fout: anders kan deze aanroep de LED/richting die
  // check_fout() net expliciet uitzette een fractie later weer aanzetten
  // (bepaalAansturing() ziet de fout soms een tikje later dan check_fout(),
  // en pasAansturingToe() interpreteert huidige_richting==-1 dan als een
  // "nieuwe" richting om toe te passen). Niet tijdens wacht_op_kentering:
  // zie de toelichting bij het zetten van die vlag hierboven.
  if (!in_fout && !wacht_op_kentering)
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
    wacht_op_kentering = false;  // een echte fout maakt een lopende kentering-wacht irrelevant
    stroomUit();  // regelt zelf relais/fan/LED's, zie de toelichting daar
  } else if (!rotopd_fout && in_fout) {
    in_fout = false;
  }
  // De rode LED zelf wordt niet hier gezet — zie werkRoodLedBij() in de
  // hoofdlus, die in_fout en ble_fout samen tot één duty-cycle vertaalt.

  return in_fout;
}

// Rode-LED-protocol: BLE-fout op zichzelf 1/10 (kort knipperlicht),
// RotoPD-fout op zichzelf 5/10 (half-om-half), beide tegelijk 9/10 (bijna
// continu aan, kort knippertje) — drie visueel goed te onderscheiden
// patronen. Het plan wordt hier, centraal in de hoofdlus, uit de twee
// onafhankelijke foutstaten afgeleid — check_fout() en de BLE-callbacks
// (zie bleserial.cpp) hebben zelf geen weet meer van setLedPlan()/LEDROOD.
void werkRoodLedBij()
{
  if (bleSerial.heradverterenMislukt())
    ble_fout = true;
  if (bleSerial.isVerbonden())
    ble_fout = false;  // een geslaagde (her)verbinding maakt de eerdere mislukking irrelevant

  int deel = (in_fout && ble_fout) ? 9
           : in_fout ? 5
           : ble_fout ? 1
           : 0;
  setLedPlan(LEDROOD, deel);
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
  usbpd.begin(ROTOPD_INT);  // print o.a. de INA238 MANUFACTURER_ID-check; interrupt is terug als snelheidsbonus bovenop de 1s-poll (zie regusbcpow.cpp)

  prefs.begin("layzee", false);
  doel_temperatuur = prefs.getFloat("doelC", DOEL_TEMPERATUUR_FALLBACK_C);
  // adrc_b0_koelen/adrc_b0_verwarmen zijn lopend-gemiddelde schattingen die
  // elke vol-vermogen-episode bijstelt (zie kalibreerB0()) — apart per
  // richting (een Peltier is niet symmetrisch, zie de toelichting bij de
  // globals), en persistent zodat een herstart niet terugvalt op de
  // startschatting maar doorbouwt op wat eerdere sessies al geleerd hebben.
  adrc_b0_koelen = prefs.getFloat("adrcB0Koel", ADRC_B0_FALLBACK);
  adrc_b0_verwarmen = prefs.getFloat("adrcB0Warm", ADRC_B0_FALLBACK);

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

  // Hartslag op LEDGROEN: deel=1 geeft via de bestaande ledTick()-lus een
  // korte blip (ledMoment<=1 van de 10, dus ~1/10 van elke 2s-cyclus) die
  // los staat van blauw/geel — puur bewijs dat ledTick() (en dus loop())
  // nog daadwerkelijk doorloopt. Geen console-logging mogelijk op dit
  // board, dus dit is de enige manier om "vastgelopen" te onderscheiden van
  // "gewenst 100%-vermogen" (dan blijft blauw óók continu aan): dooft groen
  // mee, dan is de hoofdlus zelf gestopt; blijft groen doorknipperen, dan
  // is dat laatste gewoon een bang-bang-episode op vol vermogen.
  setLedPlan(LEDGROEN, 1);

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

  // adrc_z1/adrc_z2 seeden met de echte meting i.p.v. hun compile-time 0 —
  // anders moet de ESO na elke herstart eerst door een valse-opwarm-transient
  // heen lopen (grote residu vanaf z1=0 laadt z2 op met een schijn-trend die
  // de regelwet lang op verzadigd vol vermogen houdt, ook als de echte
  // meting al bij het doel in de buurt is/verder daalt).
  temperatuurTick();
  adrc_z1 = huidige_temperatuur;
  adrc_z2 = 0;

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
      if (wacht_op_kentering) {
        // Kentering: de verse meting beweegt niet meer in de richting van de
        // afgelopen episode (naijl voorbij) — of vangnet: de afwijking t.o.v.
        // doel_temperatuur is intussen zo groot geworden dat er iets anders
        // aan de hand is dan uitdovend momentum (bv. deur open); dan niet
        // langer wachten, de normale aansturing pakt via zijn eigen
        // grote-fout-tak vanzelf weer op.
        float delta = huidige_temperatuur - vorige_temperatuur;
        bool nog_aan_het_naijlen =
            (kentering_richting == MODE_KOELEN && delta <= 0) ||
            (kentering_richting == MODE_VERWARMEN && delta >= 0);
        bool te_grote_afwijking =
            fabs(huidige_temperatuur - doel_temperatuur) > GROTE_FOUT_DREMPEL_C;
        if (!nog_aan_het_naijlen || te_grote_afwijking) {
          wacht_op_kentering = false;
          seedEsoNeutraal();
          console.println(te_grote_afwijking ? "Wachten op kentering afgebroken (te grote afwijking)" : "Kentering gedetecteerd, fijnregeling hervat");
        }
      }
      if (!in_fout && !wacht_op_kentering)
        pasAansturingToe(bepaalAansturing());
    }
    if (now > next_status_tick) {
      next_status_tick = now + 60000;
      usbpd.printStatus(console);
    }
    if (now > next_ledTick) {
      next_ledTick = now + 200;
      werkRoodLedBij();
      ledTick();
    }
  } else
  {

  }
}