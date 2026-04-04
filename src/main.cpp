#include <Arduino.h>
#include <../include/regusbcpow.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ledAan(led) digitalWrite(led, HIGH)
#define ledUit(led) digitalWrite(led, LOW);

#define LEDBLAUW 14
#define LEDGEEL 15
#define LEDGROEN 16
#define LEDROOD 17
#define KOUDE_VRAAG 2
#define RELAIS1 3
#define RELAIS2 4
#define WARMTE_VRAAG 5
#define TEMPERATUUR_IN 6
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
  int ledPlanI;

  for (ledPlanI = 0; ledPlanI < aantalLedPlannen; ledPlanI++) {
    if (ledMoment > ledPlannen[ledPlanI].deel && ledPlannen[ledPlanI].aan) {
      ledUit(ledPlannen[ledPlanI].led);
      ledPlannen[ledPlanI].aan = 0;
    }
    if (ledMoment <= ledPlannen[ledPlanI].deel && !ledPlannen[ledPlanI].aan) {
      ledAan(ledPlannen[ledPlanI].led);
      ledPlannen[ledPlanI].aan = 1;
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

  if (0) {
    Serial.print("setLedPlan(");
    Serial.print(led);
    Serial.print(", ");
    Serial.print(deel);
    Serial.print(")\n");
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
  unsigned int ledPlanI;

  for (ledPlanI = 0; ledPlanI < sizeof(ledPlannen)/sizeof(ledPlan); ledPlanI++) {
    ledPlannen[ledPlanI].led = -1;
    ledPlannen[ledPlanI].deel = 0;
    ledPlannen[ledPlanI].aan = 0;
  }
}

void stroomTick()
{
  int huidig_stroom;
  int huidig_voltage;
  long nieuw_voltage;
  int nieuw_stroom;
  float delta_perc;
  float weerstand;

  huidig_stroom = usbpd.leesStroom();
  huidig_voltage = usbpd.leesVoltage();
  Serial.print("Huidig: ");
  Serial.print(huidig_stroom);
  Serial.print("mA\t");
  Serial.print(huidig_voltage);
  Serial.print("mV\t");
  weerstand = (float) huidig_voltage / huidig_stroom;
  delta_perc = (doel_stroom - huidig_stroom)*100.0/doel_stroom;
  Serial.print(delta_perc);
  Serial.print("%\t");
  Serial.print(weerstand);

  if (delta_perc > 80.0) {
    nieuw_voltage = 5000;
    nieuw_stroom = 2000;
  } else {
    nieuw_voltage = round((float) doel_stroom * weerstand);
    nieuw_stroom = doel_stroom;
  }
  /* Normaliseer voltage naar stappen van 100Mv */
  nieuw_voltage = (nieuw_voltage / 100) * 100;
  Serial.print("ohm\t Nieuw: ");
  Serial.print(nieuw_stroom);
  Serial.print("mA\t");
  Serial.print(nieuw_voltage);
  Serial.print("mV\n");
  if (delta_perc > 0){
    setLedPlan(LEDBLAUW, 10-round((delta_perc/10)));
    setLedPlan(LEDROOD, 0);
  } else {
    setLedPlan(LEDBLAUW, 10);
    setLedPlan(LEDROOD, int(-delta_perc/10));
  }

  usbpd.setVoltage(nieuw_voltage);
}

void stroomConstantAan(int stroom)
{
  usbpd.outputAan();
  doel_stroom = stroom;
}

void stroomUit()
{
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
  Serial.print(huidige_temperatuur);
  Serial.print("C\n");
}

void vraagTick()
{
  int new_koude_stand;
  int new_warmte_stand;

  new_koude_stand = !digitalRead(KOUDE_VRAAG);
  new_warmte_stand = !digitalRead(WARMTE_VRAAG);
  if (new_koude_stand != koude_stand || new_warmte_stand != warmte_stand) {
    koude_stand = new_koude_stand;
    warmte_stand = new_warmte_stand;
  }
}

void check_mode()
{
  int nieuwe_mode;

  nieuwe_mode = huidige_mode;
  if (huidige_mode != MODE_FOUT) {
    if (koude_stand && warmte_stand)
      nieuwe_mode = MODE_UIT;
    if (koude_stand && !warmte_stand)
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
      case MODE_CONTINUE:
        stroomUit();
        digitalWrite(RELAIS1, HIGH);
        digitalWrite(RELAIS2, HIGH);
        ledUit(LEDBLAUW);
        ledUit(LEDGEEL);
        break;
      case MODE_NET_AAN:
      case MODE_KOELEN:
        stroomConstantAan(4900);
        digitalWrite(RELAIS1, HIGH);
        digitalWrite(RELAIS2, HIGH);
        ledAan(LEDBLAUW);
        ledUit(LEDGEEL);
        break;
      case MODE_VERWARMEN:
        stroomConstantAan(4900);
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

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(9600);
  delay(1000); //Ensure everything got enough time to bootup

  pinMode(LEDBLAUW, OUTPUT);
  pinMode(LEDGEEL, OUTPUT);
  pinMode(LEDGROEN, OUTPUT);
  pinMode(LEDROOD, OUTPUT);

  pinMode(KOUDE_VRAAG, INPUT_PULLUP);
  pinMode(WARMTE_VRAAG, INPUT_PULLUP);

  pinMode(RELAIS1, OUTPUT);
  pinMode(RELAIS2, OUTPUT);
  digitalWrite(RELAIS1, HIGH);
  digitalWrite(RELAIS2, HIGH);

  testLed(LEDBLAUW);
  testLed(LEDGEEL);
  testLed(LEDGROEN);
  testLed(LEDROOD);
  usbpd.begin();
  usbpd.printTo(Serial);
  if (usbpd.setVoltage(5000)) {
    ledAan(LEDGROEN);
    stroomUit();
  } else {
    ledAan(LEDROOD);
    huidige_mode = MODE_FOUT;
  }
  sensors.begin();
  numberOfSensors = sensors.getDeviceCount();
  if (numberOfSensors == 1 && sensors.getAddress(temperatuurMeter, 0)) {
    Serial.print("numberOfSensors: ");
    Serial.println(numberOfSensors);
    sensors.setResolution(temperatuurMeter, 12);
  } else {
    Serial.print("Geen of te veel temperatuurmeters\n");
  }
  delay(1000);
  stroomConstantAan(2500);
  digitalWrite(RELAIS1, HIGH);
  digitalWrite(RELAIS2, HIGH);
  ledAan(LEDBLAUW);
  ledUit(LEDGEEL);
}

unsigned long next_vraagTick = 0;
unsigned long next_stroomTick = 0;
unsigned long next_temperatuurTick = 0;
unsigned long next_ledTick = 0;

void loop() {
  unsigned long now = millis();
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
  check_mode();
}