// wheel_ctrl.h — Drehzahlregler je Rad (PI mit Vorsteuerung und Anti-Windup).
//
// PI-Regler: Stellgröße = Kp · Fehler + Ki · ∫Fehler dt. Der P-Anteil
// reagiert sofort, der I-Anteil beseitigt bleibende Abweichungen (z. B.
// durch Reibung oder Last).
// Vorsteuerung (Feed-forward): Der Großteil der PWM kommt direkt aus dem
// Sollwert (PWM ≈ v_soll / v_leerlauf), der PI-Regler korrigiert nur den
// Rest. Das macht den Regler schnell, ohne Kp hoch drehen zu müssen.
// Anti-Windup: Der I-Anteil wird begrenzt und wächst nicht weiter, solange
// die Stellgröße am Anschlag ist — sonst "lädt" er sich bei blockiertem Rad
// auf und das Fahrzeug schießt danach über.
#pragma once

namespace wheel {

class WheelCtrl {
 public:
  // Ein Regelschritt. Rückgabe: PWM-Tastgrad in [-1, +1].
  //   v_ref_mm_s   Sollgeschwindigkeit des Rads
  //   v_meas_mm_s  gemessene (gefilterte) Radgeschwindigkeit
  //   vmot_mv      aktuelle Motorspannung (für die Vorsteuerung)
  //   dt_s         Zeitschritt
  //   enabled      false ⇒ Ausgang 0, Integrator gelöscht
  float update(float v_ref_mm_s, float v_meas_mm_s, float vmot_mv, float dt_s, bool enabled);
  void reset() { integral_ = 0.0f; }
  float integral() const { return integral_; }
  // Zuletzt von update() ausgegebener Tastgrad, also der, der seitdem am
  // Motor anliegt (Rückmeldung für die Encoder-Plausibilität im Supervisor).
  float last_duty() const { return duty_; }

 private:
  float integral_ = 0.0f;
  float duty_ = 0.0f;
};

}  // namespace wheel
