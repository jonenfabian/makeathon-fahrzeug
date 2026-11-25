// supervisor.h — Zustandsmaschine und Sicherheitslogik des Pico.
//
// Der Supervisor entscheidet, was von den Pi-Sollwerten an den Motoren
// ankommt. Er ist die Software-Hälfte der Stoppkette; die Hardware-Hälfte
// ist der Not-Aus-Öffner, der die Motorspannung physisch trennt
// (docs/05-elektrik.md). Grundsatz wie bei der großen Feldplattform:
// Das Python-Programm auf dem Pi ist NIE sicherheitskritisch. Stürzt es ab,
// hängt es oder schickt es Unsinn, hält der Supervisor das Fahrzeug an.
//
// Zustände:
//   STANDBY  Motoren gebremst. Wartet auf ARM.
//   RUN      Sollwerte vom Pi werden (begrenzt, gerampt) umgesetzt.
//   ESTOP    Motorspannung fehlt (Not-Aus gedrückt, Sicherung, Schalter).
//
// Übergänge:
//   STANDBY --ARM--> RUN            (abgelehnt, wenn ESTOP oder Stoßfänger)
//   RUN --kein CMD > WDG_TIMEOUT_MS--> STANDBY + Flag WDG
//   RUN --STOP--> STANDBY
//   RUN --Stoßfänger--> STANDBY + Flag BUMPER
//   RUN --Encoder unplausibel (EncoderCheck)--> STANDBY + Flag ENCODER
//   jeder --Motorspannung < VMOT_ESTOP_MV ODER Not-Aus-Öffner NC2 offen--> ESTOP
//   ESTOP --Spannung VMOT_OK_DEBOUNCE_MS lang ok und Taster frei--> STANDBY
// Not-Aus ist damit zweikanalig: Kanal 1 Hardware (Öffner NC1 → Relais
// trennt die Motorspannung), Kanal 2 Software (Öffner NC2 → Pico bremst).
// Beide Kanäle nutzen zwangsöffnende Öffner; ein Drahtbruch wirkt wie
// Drücken. Widersprechen sich die Kanäle (gedrückt, aber Motorspannung
// bleibt: Relais klebt oder NC2-Leitung gebrochen), setzt der Vergleich das
// Flag RELAIS.
// Es gibt keinen automatischen Wiederanlauf: Nach jedem Stopp braucht es
// ein neues ARM vom Pi.
//
// Reines C++ ohne Arduino-Abhängigkeit ⇒ Host-Tests in firmware/host_tests.
#pragma once
#include <stdint.h>

namespace sup {

enum class State : uint8_t { kStandby = 0, kRun = 1, kEstop = 2 };

// Bitmaske der Zustands-Flags in der STAT-Meldung.
enum Flag : uint16_t {
  kFlagWdg = 0x01,          // letzter Stopp durch Watchdog
  kFlagTofBlock = 0x02,     // Vorwärtsfahrt gerade durch Abstandssensor gesperrt
  kFlagTofFail = 0x04,      // Abstandssensor liefert keine Messwerte
  kFlagEstop = 0x08,        // Motorspannung fehlt
  kFlagLowBatt = 0x10,      // Akkuspannung unter Warnschwelle
  kFlagBumper = 0x20,       // letzter Stopp durch Stoßfänger
  kFlagArmRejected = 0x40,  // letztes ARM abgelehnt
  kFlagRelayFault = 0x80,   // Kanal 2 meldet Not-Aus, Motorspannung bleibt (Relais klebt, NC2-Leitung gebrochen)
  kFlagEncoderFault = 0x100,  // letzter Stopp: Stellwert ohne Encoderimpulse (Leitung ab, Rad blockiert)
};

const char* state_name(State s);

constexpr uint16_t kTofNoTarget = 0xFFFF;  // Sensor sieht kein Ziel in Reichweite

// Pegel an PIN_ESTOP_FB → „Not-Aus gedrückt“ (Kanal 2). Der zweite Öffner
// NC2 des Tasters liegt zwischen Pin und GND, Pull-up nach 3V3:
//   nicht gedrückt  Kontakt zu    ⇒ LOW
//   gedrückt        Kontakt offen ⇒ HIGH (Zwangsöffnung)
//   Drahtbruch      wie gedrückt  ⇒ HIGH ⇒ Not-Aus (fail-safe)
// HIGH heißt also immer „anhalten“.
constexpr bool estop_pressed_from_pin(bool pin_high) { return pin_high; }

// Rückmeldung der Räder für die Encoder-Plausibilität, je Regeltakt.
struct WheelFeedback {
  float duty_l = 0.0f;  // Tastgrad, der seit dem letzten Takt links anlag [-1, +1]
  float duty_r = 0.0f;
  int32_t ticks_l = 0;  // Encoder-Zählerstand links, in diesem Takt gelesen
  int32_t ticks_r = 0;
};

// Encoder-Plausibilität eines Rads (Parameter und Begründung: config.h,
// ENC_FAULT_*). Liegt ENC_FAULT_MS lang ein Stellwert |duty| ≥
// ENC_FAULT_DUTY an, ohne dass der Zählerstand um ENC_FAULT_MIN_COUNTS
// vorankommt, ist das Rad unplausibel: Encoderleitung ab, ein Kanal fehlt,
// Motorleitung ab oder Rad blockiert. Ohne diese Prüfung regelte der
// Radregler hoch und das Rad liefe ungeregelt weg.
class EncoderCheck {
 public:
  // Je Regeltakt aufrufen. duty: seit dem letzten Aufruf angelegter
  // Tastgrad; ticks: aktueller Zählerstand. Rückgabe true = unplausibel.
  bool update(float duty, int32_t ticks, uint32_t now_ms);
  void reset() { active_ = false; }

 private:
  bool active_ = false;    // Stellwert über der Schwelle, Prüffenster läuft
  uint32_t since_ms_ = 0;  // Beginn des Prüffensters
  int32_t ref_ticks_ = 0;  // Zählerstand zu Beginn des Prüffensters
};

class Supervisor {
 public:
  // --- Ereignisse vom Pi ---------------------------------------------------
  void on_cmd(uint16_t seq, int32_t v_mm_s, int32_t omega_mrad_s, uint32_t now_ms);
  void on_arm(uint16_t seq, uint32_t now_ms);
  void on_stop(uint16_t seq);

  // --- Ereignisse von Sensoren -----------------------------------------------
  // Neuer Messwert des Abstandssensors (kTofNoTarget = nichts in Reichweite).
  void on_tof(uint16_t distance_mm, uint32_t now_ms);

  // --- Takt ----------------------------------------------------------------
  // Alle CTRL_PERIOD_MS aufrufen. vmot_mv: Motorspannung hinter dem
  // Relais; estop_pressed: Kanal 2 des Not-Aus (estop_pressed_from_pin);
  // wheels: zuletzt angelegte Tastgrade und aktuelle Zählerstände
  // (Encoder-Plausibilität).
  void tick(uint32_t now_ms, int32_t vmot_mv, bool bumper_pressed, bool estop_pressed,
            const WheelFeedback& wheels);

  // --- Ausgaben ------------------------------------------------------------
  State state() const { return state_; }
  uint16_t flags() const { return flags_; }
  uint16_t last_seq() const { return last_seq_; }
  uint16_t tof_mm() const { return tof_mm_; }
  // Gerampte und begrenzte Sollwerte (0 außerhalb von RUN).
  float v_cmd_mm_s() const { return v_cmd_; }
  float omega_cmd_mrad_s() const { return omega_cmd_; }
  // Radsollwerte nach Kinematik und Radbegrenzung.
  float wheel_left_mm_s() const { return wheel_l_; }
  float wheel_right_mm_s() const { return wheel_r_; }
  bool motors_enabled() const { return state_ == State::kRun; }

 private:
  void enter_standby(uint16_t reason_flag);
  void update_power(uint32_t now_ms, int32_t vmot_mv, bool estop_pressed);
  void update_tof(uint32_t now_ms);
  void update_ramps(float dt_s, float v_set, float omega_set);

  State state_ = State::kStandby;
  uint16_t flags_ = kFlagTofFail;  // bis zum ersten Messwert: kein Sensor
  uint16_t last_seq_ = 0;

  // Letzter Fahrbefehl
  int32_t cmd_v_ = 0;
  int32_t cmd_omega_ = 0;
  uint32_t last_cmd_ms_ = 0;

  // Abstandssensor
  uint16_t tof_mm_ = kTofNoTarget;
  uint32_t last_tof_ms_ = 0;
  bool tof_seen_ = false;
  bool tof_blocked_ = true;

  // Stoßfänger (letzter Zustand aus tick)
  bool bumper_pressed_ = false;

  // Spannungsüberwachung
  uint32_t vmot_ok_since_ms_ = 0;
  bool vmot_ok_pending_ = false;
  uint32_t vmot_low_since_ms_ = 0;
  bool vmot_low_pending_ = false;
  uint32_t relay_fault_since_ms_ = 0;
  bool relay_fault_pending_ = false;

  // Encoder-Plausibilität je Rad
  EncoderCheck enc_l_;
  EncoderCheck enc_r_;

  // Rampen und Ausgänge
  uint32_t last_tick_ms_ = 0;
  bool ticked_ = false;
  float v_cmd_ = 0.0f;
  float omega_cmd_ = 0.0f;
  float wheel_l_ = 0.0f;
  float wheel_r_ = 0.0f;
};

}  // namespace sup
