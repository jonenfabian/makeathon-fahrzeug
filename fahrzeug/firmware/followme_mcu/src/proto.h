// proto.h — Zeilenprotokoll Pi ⇄ Pico (Spezifikation: docs/06-software.md §1).
//
// Format angelehnt an NMEA 0183 (das Satzformat von GPS-Empfängern):
//   $TYP,feld1,feld2,...*HH\n
// HH = XOR aller Bytes zwischen '$' und '*' als zwei Hex-Ziffern.
// Vorteil für den Makeathon: mit jedem seriellen Terminal lesbar, und ein
// gekipptes Bit fällt über die Prüfsumme auf.
//
// Hier steht nur die Richtung Pi → Pico (CMD, ARM, STOP, SIM). Die Meldungen
// Pico → Pi (STAT, INFO, ab Protokoll 2 auch US) formatiert app.cpp.
//
// Reines C++ ohne Arduino-Abhängigkeit ⇒ auf dem Host testbar.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace proto {

constexpr size_t kMaxLine = 96;  // längere Zeilen werden verworfen

enum class MsgType : uint8_t { kInvalid, kCmd, kArm, kStop, kSim };

// Ergebnis einer geparsten Zeile vom Pi.
struct Message {
  MsgType type = MsgType::kInvalid;
  uint16_t seq = 0;          // Folgenummer (CMD/ARM/STOP), läuft über
  int32_t v_mm_s = 0;        // CMD: Sollgeschwindigkeit Fahrzeugmitte
  int32_t omega_mrad_s = 0;  // CMD: Soll-Gierrate (positiv = links drehen)
  char sim_key[12] = {0};    // SIM (nur Host-Simulator): Schlüssel
  int32_t sim_value = 0;     // SIM: Wert
};

// XOR-Prüfsumme über [begin, end).
uint8_t checksum(const char* begin, const char* end);

// Parst eine Zeile ohne Zeilenende. Gibt false zurück bei falscher
// Prüfsumme, unbekanntem Typ, falscher Feldzahl oder Zahlen außerhalb des
// Wertebereichs. Wertebereich ist bewusst großzügig (±100000) — die
// fachlichen Grenzen setzt der Supervisor, nicht der Parser.
bool parse(const char* line, Message* out);

// Baut eine vollständige Zeile "$<body>*HH\n" in buf. body ohne '$'.
// Gibt die Länge zurück oder 0, wenn buf zu klein ist.
size_t frame(const char* body, char* buf, size_t buf_len);

// Zeilenpuffer für den Empfang: sammelt Bytes bis '\n'.
class LineReader {
 public:
  // Gibt true zurück, sobald eine vollständige Zeile in line() liegt.
  // Zu lange Zeilen werden verworfen und als Fehler gezählt.
  bool feed(char c);
  const char* line() const { return buf_; }
  uint32_t overflow_count() const { return overflows_; }

 private:
  char buf_[kMaxLine + 1] = {0};
  size_t len_ = 0;
  bool discarding_ = false;
  uint32_t overflows_ = 0;
};

}  // namespace proto
