// tof_status.h — Messwert des Abstandssensors VL53L1X für den Supervisor
// aufbereiten.
//
// Der Sensor liefert zu jeder Messung einen Abstand und einen Status. Nicht
// jeder Status heißt „nichts in Reichweite“: Ein Objekt UNTER dem
// Messbereich (zu nah!) meldet MinRangeFail bzw. RangeValidMinRangeClipped,
// ein defekter Sensor HardwareFail. Früher wurde jeder Status außer
// RangeValid zu kTofNoTarget und gab die Sperre frei — genau dann, wenn
// jemand direkt vor dem Fahrzeug steht oder der Sensor kaputt ist.
//
// Vier Klassen (Status-Nummern siehe unten):
//   Abstand    0, 6         Messwert gilt              ⇒ range_mm melden
//   sehr nah   3, 13        Objekt unter Messbereich   ⇒ 0 melden (sperrt)
//   kein Ziel  1, 2, 4, 7, 9 nichts in Reichweite      ⇒ kTofNoTarget melden
//   ungültig   5, 10, 255, unbekannt                   ⇒ NICHT melden;
//              bleibt das TOF_TIMEOUT_MS lang so, sperrt der Supervisor
//              (fail-safe), nach TOF_REINIT_MS wird neu initialisiert.
//
// Zahlenwerte: enum VL53L1X::RangeStatus der Pololu-Bibliothek VL53L1X
// 1.3.1 (VL53L1X.h, geprüft 14.10.2026). followme_mcu.ino sichert jeden
// Wert per static_assert gegen die Bibliothek ab.
//
// ANNAHME (wie bisher): SigmaFail, SignalFail, OutOfBoundsFail,
// WrapTargetFail und XtalkSignalFail bedeuten „kein Ziel in Reichweite“ und
// geben frei. Prüfen am Prototyp: Hand und dunkle Jacke in 30 cm vor den
// Sensor halten, auch bei Gegenlicht — STAT muss Werte < 400 zeigen, nie
// 65535. Hand direkt auf den Sensor legen ⇒ STAT zeigt 0 (sehr nah).
//
// Reines C++ ohne Arduino-Abhängigkeit ⇒ Host-Tests in firmware/host_tests.
#pragma once
#include <stdint.h>

#include "../config.h"
#include "supervisor.h"

namespace tof {

// VL53L1X::RangeStatus (Pololu VL53L1X 1.3.1)
constexpr uint8_t kStatusRangeValid = 0;
constexpr uint8_t kStatusSigmaFail = 1;
constexpr uint8_t kStatusSignalFail = 2;
constexpr uint8_t kStatusRangeValidMinRangeClipped = 3;
constexpr uint8_t kStatusOutOfBoundsFail = 4;
constexpr uint8_t kStatusHardwareFail = 5;
constexpr uint8_t kStatusRangeValidNoWrapCheckFail = 6;
constexpr uint8_t kStatusWrapTargetFail = 7;
constexpr uint8_t kStatusXtalkSignalFail = 9;
constexpr uint8_t kStatusSynchronizationInt = 10;
constexpr uint8_t kStatusMinRangeFail = 13;
constexpr uint8_t kStatusNone = 255;

// Ergebnis der Aufbereitung: report = an den Supervisor melden?
struct Sample {
  bool report;
  uint16_t distance_mm;  // nur gültig, wenn report
};

// Bildet Status und Rohabstand auf den Wert für Supervisor::on_tof ab.
inline Sample classify(uint8_t range_status, uint16_t range_mm) {
  switch (range_status) {
    case kStatusRangeValid:
    case kStatusRangeValidNoWrapCheckFail:
      return {true, range_mm};
    case kStatusRangeValidMinRangeClipped:
    case kStatusMinRangeFail:
      return {true, 0};  // Objekt unter dem Messbereich: so nah wie möglich
    case kStatusSigmaFail:
    case kStatusSignalFail:
    case kStatusOutOfBoundsFail:
    case kStatusWrapTargetFail:
    case kStatusXtalkSignalFail:
      return {true, sup::kTofNoTarget};
    default:  // HardwareFail, SynchronizationInt, None, unbekannt
      return {false, 0};
  }
}

// true, wenn der Sensor neu initialisiert werden soll: seit dem letzten
// gemeldeten Wert (bzw. dem letzten Initialisierungsversuch) sind mehr als
// TOF_REINIT_MS vergangen.
inline bool reinit_due(uint32_t last_report_ms, uint32_t now_ms) {
  return now_ms - last_report_ms > TOF_REINIT_MS;
}

}  // namespace tof
