// kinematics.h — Differentialantrieb: Fahrzeug-Sollwerte ⇄ Radgeschwindigkeiten.
//
// Differentialantrieb (engl. differential drive): zwei einzeln angetriebene
// Räder auf einer Achse; Kurven entstehen durch unterschiedliche
// Radgeschwindigkeiten. Für Geschwindigkeit v der Achsmitte und Gierrate ω
// (Drehgeschwindigkeit um die Hochachse, positiv = Linkskurve) gilt mit
// Spurweite b:
//   v_links  = v − ω · b/2
//   v_rechts = v + ω · b/2
// Einheiten hier: v in mm/s, ω in mrad/s, b in mm
//   ⇒ ω · b/2 = ω[mrad/s] · 1e-3 · b[mm] / 2 = ω · b / 2000  [mm/s].
#pragma once
#include <math.h>
#include <stdint.h>

namespace kin {

constexpr float kPi = 3.14159265f;

// Rechnet (v, ω) in Radgeschwindigkeiten um und begrenzt beide Räder auf
// wheel_max_mm_s. Begrenzt wird durch gemeinsames Skalieren: so bleibt das
// Verhältnis v_l/v_r — also der Kurvenradius — erhalten, und eine zu
// schnelle Kurve wird langsamer statt gerader (sonst würde das Fahrzeug
// bei Sättigung von der Person wegdriften).
inline void diff_drive(float v_mm_s, float omega_mrad_s, float track_mm, float wheel_max_mm_s,
                       float* left_mm_s, float* right_mm_s) {
  const float w = omega_mrad_s * track_mm / 2000.0f;
  float vl = v_mm_s - w;
  float vr = v_mm_s + w;
  const float m = fmaxf(fabsf(vl), fabsf(vr));
  if (m > wheel_max_mm_s && m > 0.0f) {
    const float s = wheel_max_mm_s / m;
    vl *= s;
    vr *= s;
  }
  *left_mm_s = vl;
  *right_mm_s = vr;
}

// Encoder-Zählschritte → zurückgelegter Weg in mm.
// Weg je Zählschritt = Radumfang / Zählschritte pro Radumdrehung.
inline float ticks_to_mm(int32_t ticks, float wheel_diameter_mm, float counts_per_rev) {
  return static_cast<float>(ticks) * kPi * wheel_diameter_mm / counts_per_rev;
}

// Hilfsfunktion: Wert auf [-limit, +limit] begrenzen.
inline float clampf(float x, float limit) {
  return x > limit ? limit : (x < -limit ? -limit : x);
}

}  // namespace kin
