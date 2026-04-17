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
  for (auto & ledPlanI : ledPlannen) {
    ledPlanI.led = -1;
    ledPlanI.deel = 0;
    ledPlanI.aan = false;
  }
}

void stroomTick()
{
  unsigned int nieuw_voltage;
  unsigned int nieuw_stroom;

  unsigned int huidig_stroom = usbpd.leesStroom();
  unsigned int huidig_voltage = usbpd.leesVoltage();
  Serial.print("Huidig: ");
  Serial.print(huidig_stroom);
  Serial.print("mA\t");
  Serial.print(huidig_voltage);
  Serial.print("mV\t");
  float weerstand;
  if (huidig_stroom == 0)
    weerstand = 10;
  else
    weerstand = (float) huidig_voltage / huidig_stroom;
  float percentage = 100L - ((float) doel_stroom - huidig_stroom) * 100 / doel_stroom;
  Serial.print(percentage);
  Serial.print("%\t");
  Serial.print(weerstand);

  nieuw_voltage = (unsigned int) ((float) doel_stroom * weerstand);
  nieuw_stroom = doel_stroom;
  if (nieuw_voltage > 28000)
    nieuw_voltage = 28000;

  Serial.print("ohm\t Nieuw: ");
  Serial.print(nieuw_stroom);
  Serial.print("mA\t");
  Serial.print(nieuw_voltage);
  Serial.print("mV\n");
  if (percentage > 0){
    setLedPlan(LEDBLAUW, round(percentage/10));
    setLedPlan(LEDROOD, 0);
  } else {
    setLedPlan(LEDBLAUW, 10);
    setLedPlan(LEDROOD, (int)(-percentage/10));
  }

  if (!usbpd.setVoltage(nieuw_voltage, nieuw_stroom))
    huidige_mode = MODE_FOUT;

}

void stroomConstantAan(int stroom)
{
  Serial.print("stroomConstantAan(");
  Serial.print(stroom);
  Serial.println(")\n");
  usbpd.outputAan();
  doel_stroom = stroom;
}

void stroomUit()
{
  Serial.println("stroomUit");
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
        stroomConstantAan(2500);
        digitalWrite(RELAIS1, HIGH);
        digitalWrite(RELAIS2, HIGH);
        ledAan(LEDBLAUW);
        ledUit(LEDGEEL);
        break;
      case MODE_VERWARMEN:
        stroomConstantAan(2500);
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
  usbpd.begin(ROTOPD_INT);

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
  digitalWrite(RELAIS1, LOW);
  digitalWrite(RELAIS2, LOW);

  testLed(LEDBLAUW);
  digitalWrite(RELAIS1, HIGH);
  digitalWrite(RELAIS2, HIGH);
  testLed(LEDGEEL);
  digitalWrite(RELAIS1, LOW);
  digitalWrite(RELAIS2, LOW);
  testLed(LEDGROEN);
  digitalWrite(RELAIS1, HIGH);
  digitalWrite(RELAIS2, HIGH);
  testLed(LEDROOD);
  digitalWrite(RELAIS1, LOW);
  digitalWrite(RELAIS2, LOW);

  // usbpd.srcpdo();
  // usbpd.printTo(Serial);
  sensors.begin();
  numberOfSensors = sensors.getDeviceCount();
  if (numberOfSensors == 1 && sensors.getAddress(temperatuurMeter, 0)) {
    Serial.print("numberOfSensors: ");
    Serial.println(numberOfSensors);
    sensors.setResolution(temperatuurMeter, 12);
  } else {
    Serial.print("Geen of te veel temperatuurmeters\n");
  }
  digitalWrite(RELAIS1, HIGH);
  digitalWrite(RELAIS2, HIGH);
}

unsigned long next_vraagTick = 0;
unsigned long next_stroomTick = 0;
unsigned long next_temperatuurTick = 0;
unsigned long next_ledTick = 0;

void loop() {
  if (testSucces)
  {
    unsigned long now = millis();
    usbpd.handleWork();
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