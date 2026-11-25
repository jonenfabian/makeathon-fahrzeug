// config.h — alle Fahrzeug- und Sicherheitsparameter der Pico-Firmware.
//
// Regel: Nichts davon im übrigen Code hart kodieren. Jeder Wert, der vom
// konkret gelieferten Motor/Rad abhängt, ist als ANNAHME markiert und nennt
// die Messung, mit der er am Prototyp verifiziert wird
// (Ablauf: docs/04-aufbauanleitung.md, Abschnitt „Inbetriebnahme“).
#pragma once

// ---------------------------------------------------------------------------
// Firmware-/Protokollversion
// ---------------------------------------------------------------------------
#define FW_VERSION "0.2.0"
// bei jeder Änderung an docs/06-software.md §1 erhöhen. 2 (19.11.2026): neue
// Meldung $US mit den Ultraschallsensoren; STAT und INFO unverändert.
#define PROTO_VERSION 2

// ---------------------------------------------------------------------------
// Geometrie und Antrieb
// ---------------------------------------------------------------------------
// Rad Pololu #3690, 80 × 10 mm (pololu.com/product/3690). Verifizieren:
// 10 Radumdrehungen abrollen, Strecke / (10·π) — Gummi gibt unter Last nach.
#define WHEEL_DIAMETER_MM 80.0f

// Spurweite = Abstand der Radmitten laut CAD (cad/BERICHT.md: 173 mm).
// Verifizieren: 10 Drehungen auf der Stelle fahren, Soll/Ist-Winkel
// vergleichen und Wert korrigieren (Spurweite ∝ gefahrener/gemessener Winkel).
#define TRACK_WIDTH_MM 173.0f

// Pololu #4863 (20.4:1 25D MP 12 V, 48-CPR-Encoder): 48 Zählschritte pro
// Motorumdrehung bei Auswertung beider Flanken beider Kanäle — genau unsere
// Vierfachauswertung ⇒ 48 · 20,4 = 979,62 pro Radumdrehung (pololu.com/product/4863).
// Sparvariante JGA25-370: 11 Impulse · 4 · Übersetzung (Datenblatt) eintragen.
// Verifizieren: Rad von Hand genau 10 Umdrehungen drehen, Zählerstand aus
// der STAT-Meldung / 10.
#define ENCODER_COUNTS_PER_REV 979.62f

// Leerlaufdrehzahl 380 U/min bei 12 V (Pololu #4863) ⇒ Leerlauf-
// Radgeschwindigkeit v0 = n · π · d / 60 = 380 · π · 0,080 / 60 ≈ 1,592 m/s.
// Dient nur der Vorsteuerung (Feed-forward); Fehler hier fängt der
// PI-Regler auf. Verifizieren: aufgebockt mit 100 % PWM fahren, v ablesen.
#define V_NOLOAD_MM_S 1592.0f
#define V_NOMINAL_MV 12000.0f  // Spannung, zu der V_NOLOAD_MM_S gehört

// Drehrichtungen: hängen davon ab, wie die Motoren eingebaut und die Kabel
// angeschlossen sind. Inbetriebnahme: Fahrzeug aufbocken, CMD v>0 senden —
// beide Räder müssen vorwärts drehen UND beide Encoder positiv zählen.
// Prototyp 16.11.2026: Handdrehen vorwärts zählt beidseits positiv,
// bisheriger positiver Fahrbefehl drehte beide Räder rückwärts. Deshalb nur
// die Motorpolarität beidseits umkehren; Encoderzuordnung bleibt erhalten.
#define MOTOR_L_INVERT 0
#define MOTOR_R_INVERT 1
#define ENCODER_L_INVERT 1
#define ENCODER_R_INVERT 0

// ---------------------------------------------------------------------------
// Grenzwerte (Sicherheit) — gelten unabhängig davon, was der Pi schickt
// ---------------------------------------------------------------------------
#define V_MAX_MM_S 1200            // Betrag Fahrgeschwindigkeit Fahrzeugmitte
#define WHEEL_V_MAX_MM_S 1250      // Betrag je Rad (knapp unter Leerlauf v0)
#define OMEGA_MAX_MRAD_S 4000      // Gierrate, ≈ 229 °/s
#define ACC_MAX_MM_S2 1500         // Beschleunigen
#define DEC_MAX_MM_S2 3000         // Verzögern (darf schneller sein)
#define ALPHA_MAX_MRAD_S2 12000    // Gierbeschleunigung

// Watchdog: Kommt in RUN länger als WDG_TIMEOUT_MS kein gültiges CMD, geht
// das Fahrzeug in STANDBY und bleibt dort, bis der Pi erneut ARM schickt.
// 300 ms ≈ 6 verpasste Zyklen bei 20 Hz Regeltakt auf dem Pi.
#define WDG_TIMEOUT_MS 300

// Hardware-Watchdog des RP2040/RP2350 (rp2040.wdt_begin/wdt_reset aus
// arduino-pico). Hängt die Firmware selbst (Endlosschleife, blockierte
// Schnittstelle, Absturz), liefe die PWM sonst mit dem letzten Tastgrad
// weiter — der Pi-Watchdog oben hilft dann nicht, denn er ist Teil der
// hängenden Firmware. Wird der Watchdog länger als HW_WDT_MS nicht
// gefüttert (einmal je loop()-Durchlauf), setzt er den Chip zurück:
//   * Danach läuft setup() wie nach dem Einschalten: Supervisor in STANDBY,
//     Motoren gebremst, Fahren erst nach neuem ARM vom Pi. Es gibt also
//     keinen Wiederanlauf ohne ARM. Der Pico meldet sich am USB neu an
//     (/dev/ttyACM0 verschwindet kurz). Das Flag RELAIS-FEHLER geht dabei
//     verloren, wird aber beim nächsten Drücken des Not-Aus neu erkannt.
//   * ANNAHME: Während des Resets sind alle GPIOs hochohmige Eingänge mit
//     Pull-down (Reset-Zustand der Pads) ⇒ die Motortreiber-Eingänge liegen
//     auf LOW ⇒ Motoren aus. Prüfen am Prototyp (aufgebockt): Testbuild mit
//     absichtlicher Endlosschleife in loop() nach 5 s Fahrt — die Räder
//     müssen nach spätestens HW_WDT_MS stehen, danach blinkt die LED langsam
//     (STANDBY) und STAT kommt wieder.
// Wahl 200 ms: höchstens 1,2 m/s · 0,2 s = 0,24 m ungebremste Weiterfahrt,
// weniger als beim Pi-Watchdog (300 ms). Längste loop()-Durchläufe bei
// I2C_CLOCK_HZ = 100 kHz (1 Byte + ACK = 90 µs):
//   * Messzyklus (dataReady + read): 6 Übertragungen, ≈ 35 Byte ≈ 3,5 ms;
//     der erste nach der Initialisierung mit Kalibrierung ≈ 6 ms.
//   * Neuinitialisierung des VL53L1X (tof_init): ≈ 75 Übertragungen à
//     ≈ 0,45 ms + 1 ms Wartezeit + Bootzeit ≈ 40 ms. ANNAHME, am Prototyp
//     mit micros() um tof_init() messen.
//   * Hängender Bus: I2C_TIMEOUT_MS begrenzt jede Übertragung. Dauerhaft
//     gestört scheitert schon die erste Leseübertragung (dataReady bzw.
//     Kennung in init()) ⇒ 2 · 10 ms = 20 ms je Durchlauf. Hängt der Bus
//     mitten im Messzyklus: höchstens 6 · 10 ms = 60 ms, im ersten Zyklus
//     nach der Initialisierung ≈ 15 · 10 ms = 150 ms.
// USB-Seriell blockiert nicht (send_stat prüft den Sendepuffer, die
// Empfangsschleife ist begrenzt). Alle Fälle bleiben unter 200 ms, der
// Normalbetrieb (≤ 40 ms) mit 5× Reserve. Grenzfall: Antwortet der Sensor
// auf die Kennung und hängt danach jede Übertragung bis zum Timeout, dauert
// die Neuinitialisierung länger als HW_WDT_MS ⇒ Watchdog-Reset ⇒ STANDBY.
// Das ist sicher, nur lästig; die Ursache dann am I²C-Kabel suchen.
#define HW_WDT_MS 200
// I²C-Takt zum VL53L1X: 100 kHz (Standard-Mode) statt 400 kHz. Eine Messung
// je 40 ms belegt den Bus nur ≈ 3,5 ms; die langsameren Flanken sind über
// ≈ 25 cm Einzellitze neben den PWM-Leitungen robuster (Review-Befund N11).
#define I2C_CLOCK_HZ 100000
// Höchstdauer einer I²C-Übertragung (Wire.setTimeout). Ohne Aufruf wartet
// Wire in arduino-pico 6.3.0 bis zu 1000 ms (Vorgabe von Stream::_timeout,
// ArduinoCore-API/api/Stream.h) ⇒ ein hängender Bus löste den Watchdog aus.
// Längste Übertragung im Betrieb: 17 Byte lesen + Adresse ≈ 18 · 9 Bit /
// 100 kHz ≈ 1,6 ms ⇒ 10 ms lassen 6× Reserve (der Timeout gilt für die
// ganze Übertragung, nicht je Byte).
#define I2C_TIMEOUT_MS 10
// Höchstens so viele Bytes vom Pi je loop()-Durchlauf lesen. Sonst könnte
// ein Pi, der ohne Pause sendet, die Schleife beliebig lange festhalten.
// 256 = Empfangspuffer des USB-Seriell-Treibers (CFG_TUD_CDC_RX_BUFSIZE in
// arduino-pico 6.3.0, include/tusb_config.h); 20 CMD/s brauchen ≈ 0,5 kB/s.
#define SERIAL_RX_MAX_PER_LOOP 256

// Abstandsstopp (VL53L1X nach vorn): Vorwärtsfahrt sperren unter
// TOF_STOP_MM, wieder freigeben über TOF_RELEASE_MM (Hysterese gegen
// Flattern). Bremsweg bei 1,2 m/s und 3 m/s²: v²/(2a) = 0,24 m — plus
// 40 ms Messzyklus · 1,2 m/s ≈ 0,05 m ⇒ 0,29 m < 0,40 m.
#define TOF_STOP_MM 400
#define TOF_RELEASE_MM 500
// Liefert der Sensor länger als TOF_TIMEOUT_MS keinen Messwert, gilt die
// Vorwärtsfahrt als gesperrt (fail-safe: kein Sensor ⇒ kein Vorwärts).
// Welcher Sensorstatus als Abstand, als „kein Ziel“ oder gar nicht gemeldet
// wird, steht in src/tof_status.h.
#define TOF_TIMEOUT_MS 250
// Kam so lange kein meldefähiger Messwert (Sensor fehlt, Hardwarefehler,
// I²C gestört), wird der Sensor neu initialisiert; misslingt das, nach
// jeweils weiteren TOF_REINIT_MS erneut.
#define TOF_REINIT_MS 1000

// Ultraschallsensoren HC-SR04 schräg vorn links und rechts (Fabian
// 19.11.2026): NUR MESSWERTE für Teamaufgaben (Umfahren, Wandfolgen), keine
// Schutzfunktion. Die Abstandssperre bleibt beim Front-ToF. Der Pico misst
// und meldet in $US (docs/06-software.md §1); die Logik steht in src/sonar.h.
// Beide Sensoren werden abwechselnd ausgelöst, damit keiner das Echo des
// anderen hört: je Zeitfenster einer ⇒ je Seite alle 2 · US_SLOT_MS.
// ANNAHME: 60 ms Abstand zwischen zwei Messungen reichen gegen Nachhall
// (gängige HC-SR04-Datenblätter empfehlen „über 60 ms“). Prüfen am
// Prototyp: beide Sensoren auf dieselbe Wand in 1 m richten, Werte dürfen
// nicht springen.
#define US_SLOT_MS 60
#define US_TRIGGER_US 10  // Triggerimpuls (Berrybase-Produktseite: 10 µs TTL)
// Nach dem Trigger muss das Echo-Signal steigen. Bleibt es länger LOW,
// fehlt der Sensor (lose Steckbrücke, keine 5 V): Zustand „Sensorfehler“.
// ANNAHME: Der Sensor beginnt spätestens nach 10 ms (typisch < 1 ms).
#define US_ECHO_START_MAX_US 10000
// Messbereich laut Berrybase-Produktseite HC-SR04 (geöffnet 19.11.2026):
// 2 bis 450 cm. Kürzere Echos ⇒ „zu nah“, längere ⇒ „kein Ziel“.
#define US_MIN_MM 20
#define US_MAX_MM 4500
// Schallgeschwindigkeit bei 20 °C: 343 m/s = 343 mm/ms. Bei 0 °C wären es
// 331 m/s (Abstand ≈ 3,5 % zu groß gemeldet), für die Teamaufgaben genug.
#define US_SOUND_MM_PER_MS 343

// Not-Aus-Erkennung über die Motorspannung HINTER dem Relais (der erste
// Öffner NC1 des Not-Aus-Tasters schaltet die Relaisspule).
// Unter VMOT_ESTOP_MV ⇒ ESTOP. Zurück nach STANDBY erst, wenn die Spannung
// VMOT_OK_DEBOUNCE_MS lang über VMOT_OK_MV liegt. Danach braucht es ARM.
#define VMOT_ESTOP_MV 7000
#define VMOT_OK_MV 9000
#define VMOT_OK_DEBOUNCE_MS 500
// Zweiter Kanal: zweiter Öffner (NC2) des Not-Aus-Tasters zwischen
// PIN_ESTOP_FB und GND, externer Pull-up 2,2 kΩ nach 3V3 (zusätzlich
// INPUT_PULLUP). Nicht gedrückt ⇒ Kontakt zu ⇒ LOW. Gedrückt ODER
// Drahtbruch ⇒ HIGH ⇒ ESTOP, auch wenn die Motorspannung (noch) da ist.
// Nur Öffner sind zwangsöffnend; ein Drahtbruch am früheren Schließer wäre
// unbemerkt geblieben. Auswertung: sup::estop_pressed_from_pin().
// Der externe Pull-up (1,5 mA Kontaktstrom) macht den Eingang gegen
// Störungen der Motorleitungen unempfindlicher als der interne (≈ 50 kΩ).
// ANNAHME: 1,5 mA bei 3,3 V reichen den Kontakten als Mindeststrom; am
// Prototyp 100 × betätigen, GP15 muss jedes Mal sauber LOW zeigen.
// Rest-Risiko: Schluss der GP15-Leitung nach GND bleibt unerkannt (Kanal 1
// wirkt weiter).
// Ist der Taster gedrückt (bzw. die NC2-Leitung unterbrochen), die
// Motorspannung aber RELAY_FAULT_MS lang noch vorhanden, widersprechen sich
// die Kanäle: Relais klebt, Verdrahtung falsch oder NC2-Leitung gebrochen
// ⇒ Flag RELAIS-FEHLER; es bleibt bis zum Neustart des Pico gesetzt und
// sperrt ARM.
// Teams ohne zweiten Öffner setzen ESTOP_FEEDBACK_ENABLED auf 0 (ein offener
// GP15 läse sonst „gedrückt“ und hielte das Fahrzeug dauerhaft in ESTOP).
#define ESTOP_FEEDBACK_ENABLED 1
#define RELAY_FAULT_MS 300
// Warnschwelle Akku (3S Li-Ion, 3,4 V/Zelle). Nur Hinweis an den Pi;
// die Tiefentladeabschaltung macht das BMS des Akkupacks.
#define VMOT_LOW_MV 10200
#define VMOT_LOW_DEBOUNCE_MS 2000

// Encoder-Plausibilität. Auf dem Steckbrett kann sich im Betrieb eine
// Encoderleitung lösen. Dann misst der Radregler Drehzahl 0 und regelt hoch;
// die Strombegrenzung (u ≤ u_EMK + CURRENT_LIMIT_FRACTION, u_EMK aus der
// GEMESSENEN Drehzahl) hält das Rad für blockiert und lässt 50 % PWM zu
// ⇒ das Rad läuft ungeregelt mit bis zu ≈ 0,5 · 1,59 m/s ≈ 0,8 m/s weg.
// Prüfung je Rad (sup::EncoderCheck, jeder Regeltakt): Liegt ENC_FAULT_MS
// lang ein Stellwert |PWM| ≥ ENC_FAULT_DUTY an und kommt der Zählerstand
// dabei nicht um ENC_FAULT_MIN_COUNTS voran ⇒ STANDBY (beide Motoren sofort
// aus) + Flag ENCODER-FEHLER. Weiter erst nach neuem ARM.
//   * Eine gelöste Motorleitung und ein wirklich blockiertes Rad (Hindernis)
//     liefern ebenfalls keine Impulse ⇒ auch dann Stopp. Gewollt: Blockieren
//     kostet halben Blockierstrom je Motor und ist ohnehin zu vermeiden.
//   * Mindestens 4 Zählschritte statt 1: Fehlt nur EIN Kanal (eine Brücke
//     ab), zählt jede Flanke des anderen abwechselnd +1 und −1 (quad.h). Der
//     Zähler pendelt, der Regler sieht Drehzahl 0 und regelt ebenso hoch.
//     4 Zählschritte = ein voller Quadraturzyklus ≈ 1 mm am Rad.
//   * 300 ms: Nach dem Radmodell des Host-Simulators (PT1, 80 ms, ANNAHME)
//     liefert ein Rad mit 30 % PWM aus dem Stand die ersten 4 Zählschritte
//     nach 19 ms (12,6 V) bis 25 ms (7,1 V) ⇒ Reserve für Anlaufen gegen
//     Haftreibung und Fahrzeugmasse. Im selben Modell steht das Fahrzeug bei
//     Radsollwert ≥ 0,2 m/s 310–390 ms nach dem Lösen der Leitung; das Rad
//     fährt bis dahin 0,2–0,3 m ungeregelt.
//   * ENC_FAULT_DUTY < CURRENT_LIMIT_FRACTION, sonst würde die Schwelle bei
//     gemessener Drehzahl 0 nie erreicht. ENC_FAULT_DUTY ≤ WHEEL_I_LIMIT:
//     Ohne Encoder treibt dann der I-Anteil jeden Sollwert ≠ 0 über die
//     Schwelle (beides static_assert in supervisor.cpp).
//   * Rest-Risiko kleiner Sollwert: Bis der I-Anteil die Schwelle erreicht,
//     vergeht Zeit. Stopp im Modell nach ≈ 0,75 s bei 0,1 m/s und ≈ 2,5 s
//     bei 0,03 m/s (Host-Test test_encoderfehler_auch_bei_langsamer_fahrt);
//     so lange läuft das Rad mit < 30 % PWM (≤ 0,55 m/s) ungeregelt, bei
//     0,03 m/s bis ≈ 0,75 m Weg.
// ANNAHME: Mit ≥ 30 % PWM dreht ein Rad auf dem Boden sicher an, auch bei
// fast leerem Akku (10,2 V ⇒ ≥ 3 V am Motor). Prüfen am Prototyp:
//   1. Aufgebockt mit 0,3 m/s fahren, je Rad nacheinander Encoder-Kanal A,
//      Kanal B und die Encoder-Versorgung abziehen ⇒ binnen ≈ 0,5 s STANDBY
//      mit Flag ENCODER-FEHLER, das Rad steht.
//   2. Auf dem Boden mit ≈ 10,5 V Akku langsam fahren (0,05 m/s), auf der
//      Stelle drehen, an einer Teppichkante anfahren ⇒ kein ENCODER-FEHLER.
//      Kommt er doch, zuerst ENC_FAULT_MS verlängern; ENC_FAULT_DUTY nur
//      zusammen mit WHEEL_I_LIMIT erhöhen.
#define ENC_FAULT_MS 300
#define ENC_FAULT_DUTY 0.30f
#define ENC_FAULT_MIN_COUNTS 4

// Spannungsteiler am ADC: R_oben 22 kΩ, R_unten 6,8 kΩ
// ⇒ Faktor (22 + 6,8) / 6,8 = 4,235; 12,6 V ⇒ 2,975 V am ADC (< 3,3 V).
#define VMOT_DIVIDER 4.235f
#define ADC_REF_MV 3300.0f
#define ADC_MAX_COUNTS 4095.0f

// ---------------------------------------------------------------------------
// Regelung und Takt
// ---------------------------------------------------------------------------
#define CTRL_PERIOD_MS 10       // Radregler 100 Hz
#define STAT_PERIOD_MS 40       // STAT-Meldung 25 Hz
#define INFO_EVERY_N_STAT 50    // INFO alle 50 · 40 ms = 2 s
#define WHEEL_KP 0.0008f        // PWM-Anteil je mm/s Regelabweichung
#define WHEEL_KI 0.0040f        // PWM-Anteil je mm Regelabweichung (·s)
#define WHEEL_I_LIMIT 0.30f     // Begrenzung des I-Anteils (Anti-Windup)
// Strombegrenzung ohne Stromsensor: Motorstrom I = (u·U − U_EMK) / R. Mit
// I_block = U / R (Blockierstrom) und U_EMK/U = v / v_leerlauf folgt
//   I / I_block = u − v / v_leerlauf.
// Begrenzt man diese Differenz auf CURRENT_LIMIT_FRACTION, ist der Strom
// je Motor ≤ CURRENT_LIMIT_FRACTION · I_block — auch bei blockiertem Rad.
// I_block = 1,8 A bei 12 V (Pololu #4863, Herstellerangabe, extrapoliert)
// ⇒ 0,5 · 1,8 A = 0,9 A je Motor; mit Pi 3B (≈ 0,5 A bei 12 V) zusammen
// ≈ 2,3 A < 5 A Abschaltgrenze des Akku-BMS (Ansmann 2447-0107).
// Drehmoment bei halbem Blockierstrom ≈ 1,6 kg·cm je Rad ⇒ reicht für
// 1,5 m/s² bei ≈ 1 kg Fahrzeugmasse (cad/BERICHT.md) mit großer Reserve.
#define CURRENT_LIMIT_FRACTION 0.5f
#define SPEED_FILTER_ALPHA 0.3f // Tiefpass der Encoder-Geschwindigkeit
#define PWM_FREQ_HZ 20000       // unhörbar; MDD3A erlaubt bis 20 kHz
#define PWM_RANGE 1000

// ---------------------------------------------------------------------------
// Pinbelegung Raspberry Pi Pico / Pico 2 (siehe docs/05-elektrik.md)
// ---------------------------------------------------------------------------
// Encoder der Pololu-Motoren brauchen 3,5–20 V und liefern Signale mit
// Versorgungspegel ⇒ mit 5 V versorgen und über den Puffer CD74HC4050
// (VCC 3,3 V, Eingänge 5-V-fest) an den Pico. Nie 5-V-Signale direkt an den Pico!
#define PIN_ENC_L_A 2
#define PIN_ENC_L_B 3
#define PIN_I2C_SDA 4   // VL53L1X
#define PIN_I2C_SCL 5
#define PIN_ENC_R_A 6
#define PIN_ENC_R_B 7
#define PIN_MOT_L_A 10  // MDD3A M1A  bzw. TB6612 AIN1
#define PIN_MOT_L_B 11  // MDD3A M1B  bzw. TB6612 AIN2
#define PIN_MOT_R_A 12  // MDD3A M2A  bzw. TB6612 BIN1
#define PIN_MOT_R_B 13  // MDD3A M2B  bzw. TB6612 BIN2
#define PIN_BUMPER 14   // optional: Taster gegen GND, intern Pull-up
#define PIN_ESTOP_FB 15 // Not-Aus-Öffner NC2 gegen GND, Pull-up 2,2 kΩ + intern
#define PIN_DRV_EN 16   // nur TB6612: STBY (high = aktiv); MDD3A: unbenutzt
#define PIN_VMOT_ADC 26 // ADC0, Spannungsteiler Motorspannung
// Ultraschallsensoren HC-SR04 (5 V): Echo über den CD74HC4050 (5A → 5Y,
// 6A → 6Y), nie direkt an den Pico. Den Trigger schaltet der Pico mit 3,3 V
// direkt. ANNAHME: Der HC-SR04 erkennt 3,3 V als HIGH (TTL-Eingang,
// Schwelle typ. 2 V). Prüfen am Prototyp: $US muss Abstände zeigen.
#define PIN_US_L_ECHO 18  // CD74HC4050 5Y
#define PIN_US_R_ECHO 19  // CD74HC4050 6Y
#define PIN_US_L_TRIG 20
#define PIN_US_R_TRIG 21

// Optionaler Stoßfänger-Taster (Teams können ihn als eigenes Druckteil bauen).
#define BUMPER_ENABLED 0
