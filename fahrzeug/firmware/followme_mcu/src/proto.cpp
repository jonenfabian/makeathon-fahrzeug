// proto.cpp — Implementierung des Zeilenprotokolls (siehe proto.h).
#include "proto.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace proto {

namespace {

constexpr long kValueLimit = 100000;  // Plausibilitätsgrenze je Zahlenfeld
constexpr int kMaxFields = 6;

// Hex-Ziffer → Wert, -1 bei ungültigem Zeichen.
int hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Wandelt ein Feld streng in eine Ganzzahl: das ganze Feld muss eine Zahl
// sein ("12x" oder "" sind Fehler), und der Betrag muss ≤ kValueLimit sein.
bool to_int(const char* field, long* out) {
  if (field == nullptr || *field == '\0') return false;
  char* end = nullptr;
  long v = strtol(field, &end, 10);
  if (end == field || *end != '\0') return false;
  if (v > kValueLimit || v < -kValueLimit) return false;
  *out = v;
  return true;
}

}  // namespace

uint8_t checksum(const char* begin, const char* end) {
  uint8_t cs = 0;
  for (const char* p = begin; p < end; ++p) cs ^= static_cast<uint8_t>(*p);
  return cs;
}

bool parse(const char* line, Message* out) {
  *out = Message{};
  // Rahmen prüfen: '$' am Anfang, '*' + genau zwei Hex-Ziffern am Ende.
  // Ein '\r' (Windows-Terminal) am Ende wird toleriert.
  size_t len = strlen(line);
  if (len > 0 && line[len - 1] == '\r') --len;
  if (len < 5 || len > kMaxLine || line[0] != '$') return false;
  const char* star = line + len - 3;
  if (*star != '*') return false;
  int hi = hex_value(star[1]);
  int lo = hex_value(star[2]);
  if (hi < 0 || lo < 0) return false;
  if (checksum(line + 1, star) != static_cast<uint8_t>(hi * 16 + lo)) return false;

  // Nutzdaten zwischen '$' und '*' in einen lokalen Puffer kopieren und an
  // den Kommas in Felder zerlegen (strtok wird bewusst nicht benutzt: es
  // fasst leere Felder zusammen und würde ",," durchgehen lassen).
  char body[kMaxLine + 1];
  size_t body_len = static_cast<size_t>(star - (line + 1));
  memcpy(body, line + 1, body_len);
  body[body_len] = '\0';
  const char* fields[kMaxFields];
  int n = 0;
  fields[n++] = body;
  for (size_t i = 0; i < body_len; ++i) {
    if (body[i] == ',') {
      if (n == kMaxFields) return false;  // zu viele Felder
      body[i] = '\0';
      fields[n++] = &body[i + 1];
    }
  }

  long a = 0, b = 0, c = 0;
  if (strcmp(fields[0], "CMD") == 0) {
    // $CMD,<seq>,<v_mm_s>,<omega_mrad_s>
    if (n != 4 || !to_int(fields[1], &a) || !to_int(fields[2], &b) || !to_int(fields[3], &c)) return false;
    if (a < 0 || a > 65535) return false;
    out->type = MsgType::kCmd;
    out->seq = static_cast<uint16_t>(a);
    out->v_mm_s = static_cast<int32_t>(b);
    out->omega_mrad_s = static_cast<int32_t>(c);
    return true;
  }
  if (strcmp(fields[0], "ARM") == 0 || strcmp(fields[0], "STOP") == 0) {
    // $ARM,<seq> bzw. $STOP,<seq>
    if (n != 2 || !to_int(fields[1], &a) || a < 0 || a > 65535) return false;
    out->type = (fields[0][0] == 'A') ? MsgType::kArm : MsgType::kStop;
    out->seq = static_cast<uint16_t>(a);
    return true;
  }
  if (strcmp(fields[0], "SIM") == 0) {
    // $SIM,<schlüssel>,<wert> — nur der Host-Simulator wertet das aus; die
    // Zielfirmware verwirft SIM-Zeilen (siehe followme_mcu.ino).
    if (n != 3 || strlen(fields[1]) >= sizeof(out->sim_key) || !to_int(fields[2], &a)) return false;
    out->type = MsgType::kSim;
    strcpy(out->sim_key, fields[1]);
    out->sim_value = static_cast<int32_t>(a);
    return true;
  }
  return false;
}

size_t frame(const char* body, char* buf, size_t buf_len) {
  size_t body_len = strlen(body);
  // '$' + body + '*' + 2 Hex + '\n' + '\0'
  if (body_len + 6 > buf_len) return 0;
  uint8_t cs = checksum(body, body + body_len);
  int n = snprintf(buf, buf_len, "$%s*%02X\n", body, cs);
  return (n > 0 && static_cast<size_t>(n) < buf_len) ? static_cast<size_t>(n) : 0;
}

bool LineReader::feed(char c) {
  if (c == '\n') {
    bool complete = !discarding_ && len_ > 0;
    buf_[len_] = '\0';
    len_ = 0;
    discarding_ = false;
    return complete;
  }
  if (discarding_) return false;
  if (len_ >= kMaxLine) {
    // Zeile zu lang: Rest bis zum nächsten '\n' verwerfen.
    discarding_ = true;
    len_ = 0;
    ++overflows_;
    return false;
  }
  buf_[len_++] = c;
  return false;
}

}  // namespace proto
