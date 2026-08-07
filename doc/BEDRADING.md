# Layzee Fries — Bedrading documentatie

## ESP32 Nano — Pin toewijzing

| Bord | Kleur | Functie | Code define |
|------|-------|---------|-------------|
| VIN | rood | 12V van buck | — |
| GND | zwart | buck + breadboard | — |
| 3.3V | rood | breadboard | — |
| D2 | wit | Koude vraag vriesregelaar | KOUDE_VRAAG |
| D3 | paars | Relais 1 | RELAIS1 |
| D4 | oranje | Relais 2 | RELAIS2 |
| D5 | grijs | Warmte vraag vriesregelaar | WARMTE_VRAAG |
| D6 | bruin/zwarte band | DS18B20 temperatuursensor | TEMPERATUUR_IN |
| D7 | bruin | RotoPD interrupt | ROTOPD_INT |
| D9 | blauw | Noctua PWM | NOCTUA_PWM |
| A0 | blauw | LED blauw | LEDBLAUW |
| A1 | geel | LED geel | LEDGEEL |
| A2 | groen | LED groen | LEDGROEN |
| A3 | oranje | LED rood | LEDROOD |
| A4 | geel | SDA RotoPD | — |
| A5 | oranje | SCL RotoPD | — |
| A6 | — | vrij | — |

## RotoPD — Aansluiting

| RotoPD pin | Kleur | Van/naar |
|------------|-------|---------|
| VIN | rood | 3.3V breadboard (van ESP32) |
| SCL | oranje | A5 ESP32 |
| SDA | geel | A4 ESP32 |
| INT | bruin | D7 (A7 op board) ESP32 |
| GND | zwart | GND breadboard |

## Noctua NF-A20 PWM — Aansluiting

| Pin | Kleur | Van/naar |
|-----|-------|---------|
| GND (1) | zwart | GND 12V |
| 12V (2) | geel | 12V Bluetti DC |
| Tachometer (3) | groen | niet aangesloten |
| PWM (4) | blauw | D9 ESP32 |

⚠️ Nooit 12V op de PWM pin aansluiten!

## Peltiers — 4S schakeling

Kast op de kant, voor = onderkant kijkend vanaf bodem Layzee Fries:

```
Voeding + (rood)
    ↓
Achter rood (in)
Achter zwart (uit) → Voor rood (in)
Voor zwart (uit)   → Links rood (in)
Links zwart (uit)  → Rechts rood (in)
Rechts zwart (uit)
    ↓
Voeding - (terug)
```

Koelstand:    grijs = -, bruin = +  
Verwarmstand: omgekeerd via wisselrelais

## Noctua kabels — in slang naar kast

| Kleur | Functie |
|-------|---------|
| Zwart | - Noctua's |
| Rood | + Noctua's |
| Blauw | PWM Noctua's |
| Grijs | - Peltier (koelstand) |
| Bruin | + Peltier (koelstand) |

## DC5521 verlengkabel (doorgeknipt)

⚠️ Let op afwijkende kleurcodering:

| Kleur | Functie |
|-------|---------|
| Wit | + (12V) |
| Rood | - (GND) |

## Voedingsarchitectuur

```
Bluetti Elite 30 V2
├── USB-C 140W → RotoPD AVS → Peltiers 4S (~27.2V / 2.5A)
│              └── RotoPD 5V buck → Relaisbordje VCC
└── 12V DC5521
    ├── ESP32 Nano VIN
    ├── KW-2274 buck-boost (→12,00V vast) → 4× Noctua NF-A20 PWM
    └── reserve
```

## Relaisbordje — SRD-05VDC-SL-C

| Aansluiting | Van/naar |
|-------------|---------|
| VCC | 5V van RotoPD |
| GND | GND gemeenschappelijk |
| IN1 | D3 ESP32 (RELAIS1) |
| IN2 | D4 ESP32 (RELAIS2) |

⚠️ Active low: LOW = relais aan, HIGH = relais uit

## Vrije pinnen ESP32

| Pin | Status |
|-----|--------|
| A6 | vrij |
| D8 | vrij — reserve |
| D10 | vrij |
| D11 | vrij |
| D12 | vrij |
| A7 | vrij |
| B0, B1 | strapping pins |

## Aandachtspunten

- **Twee bruinen** vanaf breadboard:
  - bruin **met** zwarte band → DS18B20 (D6)
  - bruin **zonder** band → RotoPD INT (D7 = A7 op board)
- **D4 oranje** (relais) en **A3 oranje** (LED rood) hebben verschillende routes in de kast
- **Pull-up weerstanden** 10kΩ op SDA en SCL naar 3.3V op breadboard
- **DS18B20 pull-up** 4.7kΩ op datalijn naar 3.3V op breadboard
- **Relaisbordje**: active low
- **DS18B20** probe door gaatje in aluminium deksel, afgedicht met kit
- **Kleurcodering DC kabels is niet gestandaardiseerd** — altijd meten voor aansluiten!

## Meetpunten

| Meting | Locatie sensor |
|--------|---------------|
| Binnentemperatuur | DS18B20 vrij hangend in koelruimte via gat in deksel |
| Koelvinntemperatuur | Extech TM20 probe op middelste rib, boven Peltier element |
| Spanning/stroom Peltiers | RotoPD registers via I2C |
