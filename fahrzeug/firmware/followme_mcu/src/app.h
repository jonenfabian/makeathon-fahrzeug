// app.h — gemeinsame Anwendungslogik von Zielfirmware und Host-Simulator.
//
// Zweck: Zielfirmware (followme_mcu.ino) und Host-Simulator
// (firmware/host_sim/mcu_sim.cpp) benutzen exakt dieselben Funktionen zum
// Verarbeiten von Zeilen und zum Formatieren der Meldungen. So kann der
// Simulator nicht unbemerkt vom echten Verhalten abweichen — eine Lehre aus
// der großen Feldplattform, deren Emulator zweimal von der Firmware
// abgewichen war.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "proto.h"
#include "sonar.h"
#include "supervisor.h"

namespace app {

// Inhalt einer STAT-Meldung (Pico → Pi, 25 Hz).
struct StatData {
  uint32_t t_ms = 0;          // Pico-Zeit seit Start
  sup::State state = sup::State::kStandby;
  uint16_t flags = 0;         // sup::Flag-Bitmaske
  uint16_t last_seq = 0;      // zuletzt empfangene Folgenummer
  int32_t ticks_l = 0;        // Encoder-Zählerstand links (vorzeichenrichtig)
  int32_t ticks_r = 0;
  int32_t v_l_mm_s = 0;       // gemessene Radgeschwindigkeit links
  int32_t v_r_mm_s = 0;
  uint16_t tof_mm = 0;        // Abstandssensor (65535 = kein Ziel)
  int32_t vmot_mv = 0;        // Motorspannung hinter dem Not-Aus
  uint32_t rx_errors = 0;     // verworfene Zeilen (Prüfsumme, Format, Länge)
};

// Formatiert eine vollständige STAT-Zeile inkl. Prüfsumme und '\n'.
size_t format_stat(const StatData& d, char* buf, size_t buf_len);

// Formatiert die INFO-Zeile (Version und Parameter, alle 2 s).
size_t format_info(char* buf, size_t buf_len);

// Formatiert die US-Zeile (Ultraschall links/rechts, mit jeder STAT-Meldung):
// $US,<t_ms>,<zustand_l>,<mm_l>,<alter_l_ms>,<zustand_r>,<mm_r>,<alter_r_ms>
// Zustand: U noch keine Messung, D Abstand, C zu nah, N kein Ziel,
// F Sensorfehler. mm nur bei D, sonst 0. Alter = t_ms − Messzeit, bei U 0.
size_t format_us(uint32_t t_ms, const sonar::Sample& left, const sonar::Sample& right, char* buf,
                 size_t buf_len);

// Verarbeitet eine empfangene Zeile: parsen und an den Supervisor geben.
// Rückgabe false = Zeile ungültig (zählt als Empfangsfehler).
// SIM-Zeilen werden nur geparst und in *msg zurückgegeben; auswerten muss
// sie der Aufrufer (nur der Host-Simulator tut das).
bool handle_line(const char* line, sup::Supervisor* s, uint32_t now_ms, proto::Message* msg);

}  // namespace app
