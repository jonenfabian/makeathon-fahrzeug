// wheel_ctrl.cpp — Implementierung des Radreglers (siehe wheel_ctrl.h).
#include "wheel_ctrl.h"

#include <math.h>

#include "../config.h"
#include "kinematics.h"

namespace wheel {

float WheelCtrl::update(float v_ref_mm_s, float v_meas_mm_s, float vmot_mv, float dt_s, bool enabled) {
  if (!enabled) {
    integral_ = 0.0f;
    return duty_ = 0.0f;
  }
  // Stillstand: Bei Sollwert 0 und (fast) stehendem Rad PWM ganz aus, statt
  // um 0 herum zu regeln — vermeidet Brummen und Kriechen im Stand.
  if (v_ref_mm_s == 0.0f && fabsf(v_meas_mm_s) < 30.0f) {
    integral_ = 0.0f;
    return duty_ = 0.0f;
  }

  // Vorsteuerung: Leerlaufgeschwindigkeit skaliert mit der Akkuspannung
  // (DC-Motor: Drehzahl ≈ proportional zur Spannung). Untergrenze 6 V,
  // damit bei einem Messausreißer nicht durch ~0 geteilt wird.
  const float v_noload = V_NOLOAD_MM_S * (fmaxf(vmot_mv, 6000.0f) / V_NOMINAL_MV);
  const float u_ff = v_ref_mm_s / v_noload;

  const float err = v_ref_mm_s - v_meas_mm_s;
  const float u_unsat = u_ff + WHEEL_KP * err + integral_;

  // Stellbereich: PWM-Grenzen ±1 und Strombegrenzung (config.h,
  // CURRENT_LIMIT_FRACTION): u darf höchstens um diesen Anteil von der
  // Gegen-EMK-Spannung (∝ gemessener Geschwindigkeit) abweichen.
  const float u_emf = v_meas_mm_s / v_noload;
  const float hi = fminf(1.0f, u_emf + CURRENT_LIMIT_FRACTION);
  const float lo = fmaxf(-1.0f, u_emf - CURRENT_LIMIT_FRACTION);

  // Bedingte Integration: nur integrieren, wenn der Ausgang nicht am
  // Anschlag ist ODER der Fehler aus dem Anschlag herausführt.
  const bool saturated_high = u_unsat >= hi && err > 0.0f;
  const bool saturated_low = u_unsat <= lo && err < 0.0f;
  if (!saturated_high && !saturated_low) {
    integral_ = kin::clampf(integral_ + WHEEL_KI * err * dt_s, WHEEL_I_LIMIT);
  }

  const float u = u_ff + WHEEL_KP * err + integral_;
  duty_ = u > hi ? hi : (u < lo ? lo : u);
  return duty_;
}

}  // namespace wheel
