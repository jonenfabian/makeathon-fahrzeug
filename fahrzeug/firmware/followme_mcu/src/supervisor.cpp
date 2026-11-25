// supervisor.cpp — Implementierung der Zustandsmaschine (siehe supervisor.h).
#include "supervisor.h"

#include <math.h>

#include "../config.h"
#include "kinematics.h"

// Encoder-Plausibilität (config.h): Bei gemessener Drehzahl 0 begrenzt die
// Strombegrenzung des Radreglers den Tastgrad auf CURRENT_LIMIT_FRACTION —
// liegt die Schwelle darüber, löst die Prüfung nie aus. Und erst wenn der
// I-Anteil allein (bis WHEEL_I_LIMIT) die Schwelle erreicht, treibt ein
// fehlender Encoder jeden Sollwert ≠ 0 über sie.
static_assert(ENC_FAULT_DUTY < CURRENT_LIMIT_FRACTION, "ENC_FAULT_DUTY muss unter CURRENT_LIMIT_FRACTION liegen");
static_assert(ENC_FAULT_DUTY <= WHEEL_I_LIMIT, "ENC_FAULT_DUTY darf WHEEL_I_LIMIT nicht übersteigen");
static_assert(ENC_FAULT_MIN_COUNTS >= 2, "ein fehlender Encoder-Kanal pendelt um ±1");

namespace sup {

const char* state_name(State s) {
  switch (s) {
    case State::kStandby: return "STANDBY";
    case State::kRun: return "RUN";
    case State::kEstop: return "ESTOP";
  }
  return "?";
}

void Supervisor::on_cmd(uint16_t seq, int32_t v_mm_s, int32_t omega_mrad_s, uint32_t now_ms) {
  // Die Folgenummer wird immer quittiert (auch in STANDBY), damit der Pi die
  // Verbindung prüfen kann. Fahrwirkung hat ein CMD nur in RUN — ein CMD
  // nach einem Watchdog-Stopp startet das Fahrzeug also NICHT wieder.
  last_seq_ = seq;
  if (state_ != State::kRun) return;
  cmd_v_ = v_mm_s;
  cmd_omega_ = omega_mrad_s;
  last_cmd_ms_ = now_ms;
}

void Supervisor::on_arm(uint16_t seq, uint32_t now_ms) {
  last_seq_ = seq;
  if (state_ == State::kRun) return;  // schon scharf: nichts zu tun
  // Bei fehlender Motorspannung oder gedrücktem Stoßfänger wird nicht
  // scharfgeschaltet. Das Flag sagt dem Pi, dass sein ARM verworfen wurde.
  // Ein Relaisfehler sperrt ARM bis zum Neustart des Pico: Der
  // Hardware-Kanal des Not-Aus ist dann nicht mehr vertrauenswürdig.
  if (state_ == State::kEstop || (BUMPER_ENABLED && bumper_pressed_) || (flags_ & kFlagRelayFault)) {
    flags_ |= kFlagArmRejected;
    return;
  }
  state_ = State::kRun;
  flags_ &= static_cast<uint16_t>(~(kFlagWdg | kFlagBumper | kFlagArmRejected | kFlagEncoderFault));
  // Mit Sollwert 0 starten: das Fahrzeug fährt erst los, wenn ein CMD kommt.
  cmd_v_ = 0;
  cmd_omega_ = 0;
  last_cmd_ms_ = now_ms;
  // Encoder-Prüffenster frisch beginnen (die Motoren standen bis eben).
  enc_l_.reset();
  enc_r_.reset();
}

void Supervisor::on_stop(uint16_t seq) {
  last_seq_ = seq;
  if (state_ == State::kRun) enter_standby(0);
}

void Supervisor::on_tof(uint16_t distance_mm, uint32_t now_ms) {
  tof_mm_ = distance_mm;
  last_tof_ms_ = now_ms;
  tof_seen_ = true;
}

void Supervisor::enter_standby(uint16_t reason_flag) {
  // Sofortiger Stopp ohne Rampe: Die Motoren werden gebremst, Sollwerte und
  // Rampenzustand auf 0 gesetzt, damit nach einem neuen ARM nicht mit dem
  // alten Sollwert losgefahren wird.
  state_ = State::kStandby;
  flags_ |= reason_flag;
  cmd_v_ = 0;
  cmd_omega_ = 0;
  v_cmd_ = omega_cmd_ = wheel_l_ = wheel_r_ = 0.0f;
}

void Supervisor::update_power(uint32_t now_ms, int32_t vmot_mv, bool estop_pressed) {
  const bool pressed = ESTOP_FEEDBACK_ENABLED && estop_pressed;

  // --- Kanalvergleich: Kanal 2 meldet gedrückt, Motorspannung bleibt --------
  // (Relais klebt, falsch verdrahtet oder NC2-Leitung gebrochen)
  if (pressed && vmot_mv >= VMOT_OK_MV) {
    if (!relay_fault_pending_) {
      relay_fault_pending_ = true;
      relay_fault_since_ms_ = now_ms;
    } else if (now_ms - relay_fault_since_ms_ >= RELAY_FAULT_MS) {
      flags_ |= kFlagRelayFault;  // bleibt bis zum Neustart gesetzt
    }
  } else {
    relay_fault_pending_ = false;
  }

  // --- Not-Aus: Motorspannung hinter dem Relais ODER Öffner NC2 offen -------
  if (vmot_mv < VMOT_ESTOP_MV || pressed) {
    if (state_ != State::kEstop) {
      enter_standby(0);
      state_ = State::kEstop;
    }
    flags_ |= kFlagEstop;
    vmot_ok_pending_ = false;
  } else if (state_ == State::kEstop) {
    // Zurück nach STANDBY erst nach stabiler Spannung (Entprellen: ein
    // Wackelkontakt soll nicht zwischen ESTOP und STANDBY flattern).
    if (vmot_mv >= VMOT_OK_MV) {  // hier ist der Taster sicher frei (s. o.)
      if (!vmot_ok_pending_) {
        vmot_ok_pending_ = true;
        vmot_ok_since_ms_ = now_ms;
      } else if (now_ms - vmot_ok_since_ms_ >= VMOT_OK_DEBOUNCE_MS) {
        state_ = State::kStandby;
        flags_ &= static_cast<uint16_t>(~kFlagEstop);
        vmot_ok_pending_ = false;
      }
    } else {
      vmot_ok_pending_ = false;
    }
  }

  // --- Akkuwarnung (nur Hinweis, keine Abschaltung) --------------------------
  if (vmot_mv >= VMOT_ESTOP_MV && vmot_mv < VMOT_LOW_MV) {
    if (!vmot_low_pending_) {
      vmot_low_pending_ = true;
      vmot_low_since_ms_ = now_ms;
    } else if (now_ms - vmot_low_since_ms_ >= VMOT_LOW_DEBOUNCE_MS) {
      flags_ |= kFlagLowBatt;
    }
  } else {
    vmot_low_pending_ = false;
    // Hysterese 300 mV, damit die Warnung unter Last nicht flackert.
    if (vmot_mv >= VMOT_LOW_MV + 300) flags_ &= static_cast<uint16_t>(~kFlagLowBatt);
  }
}

void Supervisor::update_tof(uint32_t now_ms) {
  // Fail-safe: ohne frischen Messwert gilt die Vorwärtsfahrt als gesperrt.
  const bool fail = !tof_seen_ || (now_ms - last_tof_ms_ > TOF_TIMEOUT_MS);
  if (fail) {
    flags_ |= kFlagTofFail;
    tof_blocked_ = true;
  } else {
    flags_ &= static_cast<uint16_t>(~kFlagTofFail);
    // Hysterese: sperren unter STOP, freigeben erst über RELEASE.
    if (tof_mm_ < TOF_STOP_MM) {
      tof_blocked_ = true;
    } else if (tof_mm_ > TOF_RELEASE_MM) {
      tof_blocked_ = false;
    }
  }
  if (tof_blocked_) {
    flags_ |= kFlagTofBlock;
  } else {
    flags_ &= static_cast<uint16_t>(~kFlagTofBlock);
  }
}

void Supervisor::update_ramps(float dt_s, float v_set, float omega_set) {
  // Geschwindigkeitsrampe: Betrag verringern mit DEC_MAX, erhöhen mit
  // ACC_MAX. "Verringern" heißt: Änderung zeigt entgegen der aktuellen
  // Fahrtrichtung.
  const float dv = v_set - v_cmd_;
  const bool slowing = (v_cmd_ != 0.0f) && ((dv > 0.0f) != (v_cmd_ > 0.0f));
  const float a = slowing ? DEC_MAX_MM_S2 : ACC_MAX_MM_S2;
  v_cmd_ += kin::clampf(dv, a * dt_s);

  const float domega = omega_set - omega_cmd_;
  omega_cmd_ += kin::clampf(domega, ALPHA_MAX_MRAD_S2 * dt_s);
}

bool EncoderCheck::update(float duty, int32_t ticks, uint32_t now_ms) {
  // Kleiner Stellwert: Der Motor darf dabei stehen (Haftreibung) ⇒ keine
  // Aussage möglich, Fenster ruht. (NaN zählt als groß: fail-safe.)
  if (fabsf(duty) < ENC_FAULT_DUTY) {
    active_ = false;
    return false;
  }
  // Fortschritt seit Fensterbeginn, über uint32_t gerechnet: stimmt auch
  // beim Überlauf des Zählers, ohne undefiniertes Verhalten.
  const int32_t moved = static_cast<int32_t>(static_cast<uint32_t>(ticks) - static_cast<uint32_t>(ref_ticks_));
  if (!active_ || moved >= ENC_FAULT_MIN_COUNTS || moved <= -ENC_FAULT_MIN_COUNTS) {
    // Fenster (neu) beginnen: Der Stellwert liegt ab jetzt an bzw. das Rad
    // hat sich nachweislich bewegt. Die Richtung zählt nicht: Beim Umsteuern
    // läuft das Rad kurz gegen den Stellwert, das ist kein Fehler.
    active_ = true;
    since_ms_ = now_ms;
    ref_ticks_ = ticks;
    return false;
  }
  return now_ms - since_ms_ >= ENC_FAULT_MS;
}

void Supervisor::tick(uint32_t now_ms, int32_t vmot_mv, bool bumper_pressed, bool estop_pressed,
                      const WheelFeedback& wheels) {
  // Zeitschritt seit dem letzten Takt; begrenzt auf 50 ms, damit ein
  // verspäteter Takt (z. B. nach langer Serial-Ausgabe) keinen
  // Rampensprung erzeugt.
  float dt_s = 0.0f;
  if (ticked_) {
    uint32_t dt_ms = now_ms - last_tick_ms_;
    if (dt_ms > 50) dt_ms = 50;
    dt_s = static_cast<float>(dt_ms) / 1000.0f;
  }
  last_tick_ms_ = now_ms;
  ticked_ = true;

  bumper_pressed_ = bumper_pressed;
  update_power(now_ms, vmot_mv, estop_pressed);
  update_tof(now_ms);
  // Encoder-Prüffenster in jedem Takt nachführen. Außerhalb von RUN ist der
  // Tastgrad 0, dann ruhen sie.
  const bool enc_fault_l = enc_l_.update(wheels.duty_l, wheels.ticks_l, now_ms);
  const bool enc_fault_r = enc_r_.update(wheels.duty_r, wheels.ticks_r, now_ms);

  if (state_ == State::kRun) {
    if (BUMPER_ENABLED && bumper_pressed) {
      enter_standby(kFlagBumper);
    } else if (now_ms - last_cmd_ms_ > WDG_TIMEOUT_MS) {
      // Watchdog: Pi schweigt zu lange ⇒ sofort anhalten.
      enter_standby(kFlagWdg);
    } else if (enc_fault_l || enc_fault_r) {
      // Stellwert ohne Raddrehung (Encoderleitung ab, Kanal fehlt, Rad
      // blockiert) ⇒ sofort anhalten. Sonst regelt der Radregler hoch und
      // das Rad läuft ungeregelt weg. Beide Motoren gehen in diesem Takt aus
      // (motors_enabled() ist ab hier false).
      enter_standby(kFlagEncoderFault);
    }
  }

  if (state_ != State::kRun) {
    v_cmd_ = omega_cmd_ = wheel_l_ = wheel_r_ = 0.0f;
    return;
  }

  // Sollwerte begrenzen; der Abstandssensor sperrt nur VORWÄRTS — Drehen
  // und Zurücksetzen bleiben erlaubt, damit sich das Fahrzeug von einem
  // Hindernis lösen kann.
  float v_set = kin::clampf(static_cast<float>(cmd_v_), V_MAX_MM_S);
  const float omega_set = kin::clampf(static_cast<float>(cmd_omega_), OMEGA_MAX_MRAD_S);
  if (tof_blocked_ && v_set > 0.0f) v_set = 0.0f;
  // Wird gesperrt, während das Fahrzeug noch vorwärts rollt, bremst die
  // Rampe mit DEC_MAX (Bremswegrechnung in config.h bei TOF_STOP_MM).

  update_ramps(dt_s, v_set, omega_set);
  // Positive Restgeschwindigkeit bei Sperre wird nur noch abgebaut, nie
  // aufgebaut: die Rampe läuft ja Richtung v_set = 0.
  kin::diff_drive(v_cmd_, omega_cmd_, TRACK_WIDTH_MM, WHEEL_V_MAX_MM_S, &wheel_l_, &wheel_r_);
}

}  // namespace sup
