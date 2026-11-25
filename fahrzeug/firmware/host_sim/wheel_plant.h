// wheel_plant.h — Radmodell mit Encoder-Nachbildung für Host-Simulator und Host-Tests.
//
// Rad: Verzögerungsglied 1. Ordnung (PT1). Die Radgeschwindigkeit läuft mit
// Zeitkonstante tau auf den Wert zu, den der Tastgrad bei der aktuellen
// Spannung im Leerlauf ergäbe. ANNAHME tau = 80 ms für Motor + Rad +
// anteilige Fahrzeugmasse; am Prototyp per Sprungantwort (STAT-Mitschnitt)
// zu prüfen.
//
// Encoder: Aus der wahren Radstellung entstehen die Pegel A und B (Gray-Folge
// wie in quad.h), daraus zählt quad::step — dieselbe Auswertung wie im
// Interrupt der Zielfirmware. So lässt sich eine gelöste Steckbrücke
// nachbilden: Ein offener Eingang liest wegen INPUT_PULLUP HIGH.
//   kOk          beide Kanäle angeschlossen
//   kOff         beide Kanäle ab (oder Encoder-Versorgung ab): Zähler steht
//   kChannelAOff nur Kanal A ab: jede B-Flanke zählt abwechselnd +1/−1,
//                der Zähler pendelt um einen Wert
#pragma once
#include <math.h>
#include <stdint.h>

#include "../followme_mcu/config.h"
#include "../followme_mcu/src/kinematics.h"
#include "../followme_mcu/src/quad.h"

namespace plant {

enum class EncLink : uint8_t { kOff = 0, kOk = 1, kChannelAOff = 2 };

struct Wheel {
  EncLink link = EncLink::kOk;
  bool blocked = false;  // Rad steht fest (Hindernis)
  float v_mm_s = 0.0f;   // wahre Radgeschwindigkeit

  // Zählerstand, den die Firmware sieht (Interrupt-Auswertung).
  int32_t ticks() const { return seen_; }

  void step(float duty, int32_t vmot_mv, float dt_s) {
    const float tau_s = 0.08f;
    const float v_target = duty * V_NOLOAD_MM_S * (static_cast<float>(vmot_mv) / V_NOMINAL_MV);
    v_mm_s = blocked ? 0.0f : v_mm_s + (v_target - v_mm_s) * (dt_s / tau_s);
    pos_ += v_mm_s * dt_s / (kin::kPi * WHEEL_DIAMETER_MM) * ENCODER_COUNTS_PER_REV;
    // Ein geänderter Anschluss ändert die Pegel sofort (auch das ist eine Flanke).
    edge(count_);
    // Jede Zählstelle einzeln durchlaufen, wie die Flanken am Pico eintreffen.
    const int32_t target = static_cast<int32_t>(floor(pos_));
    while (count_ != target) {
      count_ += (target > count_) ? 1 : -1;
      edge(count_);
    }
  }

 private:
  // Pegel A|B an den Pico-Eingängen bei wahrer Zählstelle c.
  uint8_t pins(int32_t c) const {
    static const uint8_t kGray[4] = {0, 1, 3, 2};  // vorwärts 00 → 01 → 11 → 10
    const uint8_t s = kGray[static_cast<uint32_t>(c) & 3u];
    switch (link) {
      case EncLink::kOff: return 3;                       // beide offen ⇒ HIGH
      case EncLink::kChannelAOff: return s | 2;           // A offen ⇒ HIGH
      case EncLink::kOk: break;
    }
    return s;
  }
  void edge(int32_t c) {
    const uint8_t s = pins(c);
    seen_ += quad::step(seen_state_, s);
    seen_state_ = s;
  }

  double pos_ = 0.0;         // wahre Stellung in Zählschritten (Bruchteile sammeln)
  int32_t count_ = 0;        // floor(pos_), bis hierhin ausgewertet
  int32_t seen_ = 0;         // Zählerstand der Firmware
  uint8_t seen_state_ = 0;   // zuletzt gesehene Pegel A|B (Start: 00)
};

}  // namespace plant
