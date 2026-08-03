#include <Arduino.h>
#include <../include/regusbcpow.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
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

#define MODE_KOELEN 1
#define MODE_VERWARMEN 2
#define MODE_CONTINUE 3
#define MODE_NET_AAN 4
#define MODE_UIT 5
#define MODE_FOUT 6

regUSBCPow usbpd;
DeviceAddress temperatuurMeter;
OneWire oneWire(TEMPERATUUR_IN);
DallasTemperature sensors(&oneWire);
int numberOfSensors;
bool testSucces = true;

int doel_stroom;
int PPSIndex;
int AVSIndex;
float huidige_temperatuur;
int koude_stand;
int warmte_stand;
int demp_stand_dender = 0;
int huidige_mode = MODE_NET_AAN;

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

  if (false) {
    console.print("setLedPlan(");
    console.print(led);
    console.print(", ");
    console.print(deel);
    console.print(")\n");
  }
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

// Rapporterend: leest en print alleen. Het regelen (welke spanning/stroom we
// vragen) gebeurt via usbpd.setMaxVermogen() in stroomConstantAan() — zolang
// er nog geen PID-regeling is, vragen we gewoon het maximum op. Die nuance
// (wanneer wel/niet maximum, welke setpoint) hoort thuis in de PID, niet hier.
void stroomTick()
{
  unsigned int huidig_stroom = usbpd.leesStroom();
  unsigned int huidig_voltage = usbpd.leesVoltage();
  console.print("Huidig: ");
  console.print(huidig_stroom);
  console.print("mA\t");
  console.print(huidig_voltage);
  console.print("mV\t");
  float percentage = 100L - ((float) doel_stroom - huidig_stroom) * 100 / doel_stroom;
  console.print(percentage);
  console.println("%");

  if (percentage > 0){
    setLedPlan(LEDBLAUW, round(percentage/10));
    setLedPlan(LEDROOD, 0);
  } else {
    setLedPlan(LEDBLAUW, 10);
    setLedPlan(LEDROOD, (int)(-percentage/10));
  }
}

void stroomConstantAan(int stroom)
{
  console.print("stroomConstantAan(");
  console.print(stroom);
  console.println(")\n");
  usbpd.outputAan();
  usbpd.setMaxVermogen();
  doel_stroom = stroom;
}

void stroomUit()
{
  console.println("stroomUit");
  usbpd.outputUit();
  doel_stroom = 0;
}

void testLed(int led)
{
    ledAan(led);
    delay(500);
    ledUit(led);
}

void temperatuurTick()
{
  sensors.requestTemperatures();
  huidige_temperatuur = sensors.getTempC(temperatuurMeter);
  console.print(huidige_temperatuur);
  console.print("C\n");
}

void vraagTick()
{
  int new_koude_stand = !digitalRead(KOUDE_VRAAG);
  int new_warmte_stand = !digitalRead(WARMTE_VRAAG);
  if (new_koude_stand != koude_stand || new_warmte_stand != warmte_stand) {
    koude_stand = new_koude_stand;
    warmte_stand = new_warmte_stand;
  }
}

void check_mode()
{
  int nieuwe_mode = huidige_mode;
  if (huidige_mode != MODE_FOUT) {
    if (koude_stand && warmte_stand)
      nieuwe_mode = MODE_UIT;
    if ((koude_stand && !warmte_stand) || huidige_mode == MODE_NET_AAN)
      nieuwe_mode = MODE_KOELEN;
    if (!koude_stand && warmte_stand)
      nieuwe_mode = MODE_VERWARMEN;
    if (!koude_stand && !warmte_stand && huidige_mode != MODE_NET_AAN)
      nieuwe_mode = MODE_CONTINUE;
  }

  if (nieuwe_mode != huidige_mode) {
    huidige_mode = nieuwe_mode;
    switch (huidige_mode) {
      case MODE_UIT:
      case MODE_FOUT:
        stroomUit();
        digitalWrite(RELAIS1, HIGH);
        digitalWrite(RELAIS2, HIGH);
        ledUit(LEDBLAUW);
        ledUit(LEDGEEL);
        break;
      case MODE_CONTINUE:
        break;
      case MODE_NET_AAN:
      case MODE_KOELEN:
        stroomConstantAan(3000);
        digitalWrite(RELAIS1, HIGH);
        digitalWrite(RELAIS2, HIGH);
        ledAan(LEDBLAUW);
        ledUit(LEDGEEL);
        break;
      case MODE_VERWARMEN:
        stroomConstantAan(3000);
        digitalWrite(RELAIS1, LOW);
        digitalWrite(RELAIS2, LOW);
        ledUit(LEDBLAUW);
        ledAan(LEDGEEL);
        break;
      default:
        stroomUit();
        digitalWrite(RELAIS1, HIGH);
        digitalWrite(RELAIS2, HIGH);
        ledUit(LEDBLAUW);
        ledUit(LEDGEEL);
        ledAan(LEDROOD);
        break;
    }
  }
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
  usbpd.begin(ROTOPD_INT);

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
unsigned long next_stroomTick = 0;
unsigned long next_temperatuurTick = 0;
unsigned long next_ledTick = 0;

void loop() {
  if (testSucces)
  {
    unsigned long now = millis();
    ArduinoOTA.handle();
    usbpd.handleWork();
    bleSerial.tick();
    esp_task_wdt_reset();
    check_mode();
    if (huidige_mode != MODE_FOUT) {
      if (now > next_vraagTick) {
        next_vraagTick = now + 1000;
        vraagTick();
      }
      if (now > next_stroomTick) {
        next_stroomTick = now + 5000;
        stroomTick();
      }
      if (now > next_temperatuurTick) {
        next_temperatuurTick = now + 60000;
        temperatuurTick();
      }
      if (now > next_ledTick) {
        next_ledTick = now + 200;
        ledTick();
      }
    }
  } else
  {

  }
}