// sonar.h — Ultraschallsensoren HC-SR04 schräg vorn links/rechts auswerten.
//
// NUR MESSWERTE, KEINE SCHUTZFUNKTION (Fabian 19.11.2026): Der Supervisor
// sieht diese Werte nicht. Sie gehen in der Meldung $US an den Pi und sind
// dort Daten für Teamaufgaben wie Umfahren und Wandfolgen.
//
// Ablauf je Zeitfenster (US_SLOT_MS, abwechselnd links und rechts):
//   1. Echo-Pegel prüfen. Ist er schon HIGH, hängt der Sensor oder die
//      Leitung: Zustand Fehler, kein Trigger.
//   2. Trigger (10 µs HIGH, in followme_mcu.ino). Der Sensor sendet und
//      zieht das Echo-Signal HIGH, bis das Echo zurückkommt.
//   3. Auswerten, ohne zu warten: Die Flanken stempelt ein Interrupt mit
//      micros(), step() rechnet bei jedem loop()-Durchlauf nach.
//        - kein Anstieg in US_ECHO_START_MAX_US  ⇒ Fehler (Sensor fehlt)
//        - Impuls länger als US_MAX_MM            ⇒ kein Ziel
//        - Impuls kürzer als US_MIN_MM            ⇒ zu nah
//        - sonst                                  ⇒ Abstand in mm
// Ein fehlender Sensor endet sicher als Fehler: Der Pull-down 10 kΩ am
// Eingang des CD74HC4050 hält das Echo LOW (docs/05-elektrik.md).
//
// Kein pulseIn(): Das würde bis zu 60 ms blockieren, länger als der
// Hardware-Watchdog (HW_WDT_MS) und viel länger als der Regeltakt.
//
// Reines C++ ohne Arduino-Abhängigkeit ⇒ Host-Tests in firmware/host_tests;
// der Host-Simulator speist simulierte Flanken in dieselbe Logik.
#pragma once
#include <stdint.h>

#include "../config.h"

namespace sonar {

constexpr int kLeft = 0;
constexpr int kRight = 1;
constexpr int kCount = 2;

enum class State : uint8_t {
  kNone,      // noch keine Messung seit dem Start
  kDistance,  // Abstand gültig
  kTooClose,  // Objekt unter dem Messbereich (distance_mm = 0)
  kNoTarget,  // nichts in Reichweite
  kFault,     // Sensor antwortet nicht oder Echo hängt
};

// Zustand als ein Zeichen in $US (docs/06-software.md §1).
inline char state_char(State s) {
  switch (s) {
    case State::kDistance: return 'D';
    case State::kTooClose: return 'C';
    case State::kNoTarget: return 'N';
    case State::kFault: return 'F';
    case State::kNone: break;
  }
  return 'U';
}

struct Sample {
  State state = State::kNone;
  uint16_t distance_mm = 0;  // nur bei kDistance > 0
  uint32_t captured_ms = 0;  // Pico-Zeit der Auswertung
};

// Flanken des gerade ausgelösten Sensors seit seinem Trigger (aus dem
// Interrupt, atomar gelesen).
struct Edges {
  bool rose = false;
  uint32_t rise_us = 0;
  bool fell = false;
  uint32_t fall_us = 0;
};

// Echo-Dauer → Abstand: Der Schall läuft hin und zurück.
inline uint32_t pulse_to_mm(uint32_t pulse_us) {
  return (pulse_us * static_cast<uint32_t>(US_SOUND_MM_PER_MS) + 1000u) / 2000u;
}

// Längste Echo-Dauer, die noch als Abstand gilt (US_MAX_MM).
constexpr uint32_t kMaxPulseUs =
    (static_cast<uint32_t>(US_MAX_MM) * 2000u + US_SOUND_MM_PER_MS - 1u) / US_SOUND_MM_PER_MS;
static_assert(US_ECHO_START_MAX_US + kMaxPulseUs < US_SLOT_MS * 1000u,
              "Messung muss in ein Zeitfenster passen");

inline Sample classify(uint32_t pulse_us, uint32_t now_ms) {
  Sample s;
  s.captured_ms = now_ms;
  if (pulse_us > kMaxPulseUs) {
    s.state = State::kNoTarget;
    return s;
  }
  const uint32_t mm = pulse_to_mm(pulse_us);
  if (mm < US_MIN_MM) {
    s.state = State::kTooClose;
  } else {
    s.state = State::kDistance;
    s.distance_mm = static_cast<uint16_t>(mm);
  }
  return s;
}

class Cycle {
 public:
  // Bei jedem loop()-Durchlauf aufrufen. echo_high: aktueller Echo-Pegel je
  // Sensor; edges: Flanken des aktiven Sensors seit seinem Trigger.
  // Rückgabe: Sensor, der JETZT einen Triggerimpuls bekommen soll (der
  // Aufrufer setzt vorher seine Flanken zurück), sonst -1.
  int step(uint32_t now_us, uint32_t now_ms, const bool echo_high[kCount], const Edges& edges) {
    if (active_ >= 0) {
      const uint32_t elapsed_us = now_us - slot_start_us_;
      if (pending_) {
        if (edges.rose && edges.fell) {
          finish(classify(edges.fall_us - edges.rise_us, now_ms));
        } else if (edges.rose && now_us - edges.rise_us > kMaxPulseUs) {
          finish(classify(kMaxPulseUs + 1u, now_ms));  // läuft noch: weiter als US_MAX_MM
        } else if (!edges.rose && elapsed_us > US_ECHO_START_MAX_US) {
          finish(fault(now_ms));
        }
      }
      if (elapsed_us < US_SLOT_MS * 1000u) return -1;
      if (pending_) finish(fault(now_ms));  // Fenster vorbei ohne Ergebnis (nicht erwartet)
      active_ = -1;
    }
    // Neues Zeitfenster für den nächsten Sensor.
    active_ = next_;
    next_ = (next_ + 1) % kCount;
    slot_start_us_ = now_us;
    if (echo_high[active_]) {
      // Echo hängt HIGH: Ein Trigger würde nichts messen.
      samples_[active_] = fault(now_ms);
      pending_ = false;
      return -1;
    }
    pending_ = true;
    return active_;
  }

  const Sample& sample(int sensor) const { return samples_[sensor]; }
  int active() const { return active_; }

 private:
  static Sample fault(uint32_t now_ms) {
    Sample s;
    s.state = State::kFault;
    s.captured_ms = now_ms;
    return s;
  }

  void finish(const Sample& s) {
    samples_[active_] = s;
    pending_ = false;
  }

  Sample samples_[kCount];
  int active_ = -1;
  int next_ = kLeft;
  bool pending_ = false;
  uint32_t slot_start_us_ = 0;
};

}  // namespace sonar
