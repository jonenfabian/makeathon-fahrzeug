// app.cpp — Implementierung der gemeinsamen Anwendungslogik (siehe app.h).
#include "app.h"

#include <math.h>
#include <stdio.h>

#include "../config.h"

namespace app {

size_t format_stat(const StatData& d, char* buf, size_t buf_len) {
  // Feldreihenfolge ist Protokoll (docs/06-software.md §1) — nur mit
  // PROTO_VERSION-Erhöhung ändern.
  char body[proto::kMaxLine];
  int n = snprintf(body, sizeof(body), "STAT,%lu,%s,%02X,%u,%ld,%ld,%ld,%ld,%u,%ld,%lu",
                   static_cast<unsigned long>(d.t_ms), sup::state_name(d.state),
                   static_cast<unsigned>(d.flags), static_cast<unsigned>(d.last_seq),
                   static_cast<long>(d.ticks_l), static_cast<long>(d.ticks_r),
                   static_cast<long>(d.v_l_mm_s), static_cast<long>(d.v_r_mm_s),
                   static_cast<unsigned>(d.tof_mm), static_cast<long>(d.vmot_mv),
                   static_cast<unsigned long>(d.rx_errors));
  if (n <= 0 || static_cast<size_t>(n) >= sizeof(body)) return 0;
  return proto::frame(body, buf, buf_len);
}

size_t format_info(char* buf, size_t buf_len) {
  // INFO,<proto>,<fw>,<v_max>,<omega_max>,<spurweite_mm>,<rad_d_mm>,<zählschritte/U>
  char body[proto::kMaxLine];
  int n = snprintf(body, sizeof(body), "INFO,%d,%s,%d,%d,%ld,%ld,%ld", PROTO_VERSION, FW_VERSION,
                   V_MAX_MM_S, OMEGA_MAX_MRAD_S, lroundf(TRACK_WIDTH_MM), lroundf(WHEEL_DIAMETER_MM),
                   lroundf(ENCODER_COUNTS_PER_REV));
  if (n <= 0 || static_cast<size_t>(n) >= sizeof(body)) return 0;
  return proto::frame(body, buf, buf_len);
}

namespace {

uint32_t age_ms(uint32_t t_ms, const sonar::Sample& s) {
  return s.state == sonar::State::kNone ? 0u : t_ms - s.captured_ms;
}

}  // namespace

size_t format_us(uint32_t t_ms, const sonar::Sample& left, const sonar::Sample& right, char* buf,
                 size_t buf_len) {
  // Feldreihenfolge ist Protokoll (docs/06-software.md §1) — nur mit
  // PROTO_VERSION-Erhöhung ändern.
  char body[proto::kMaxLine];
  const unsigned mm_l = left.state == sonar::State::kDistance ? left.distance_mm : 0u;
  const unsigned mm_r = right.state == sonar::State::kDistance ? right.distance_mm : 0u;
  int n = snprintf(body, sizeof(body), "US,%lu,%c,%u,%lu,%c,%u,%lu", static_cast<unsigned long>(t_ms),
                   sonar::state_char(left.state), mm_l, static_cast<unsigned long>(age_ms(t_ms, left)),
                   sonar::state_char(right.state), mm_r, static_cast<unsigned long>(age_ms(t_ms, right)));
  if (n <= 0 || static_cast<size_t>(n) >= sizeof(body)) return 0;
  return proto::frame(body, buf, buf_len);
}

bool handle_line(const char* line, sup::Supervisor* s, uint32_t now_ms, proto::Message* msg) {
  if (!proto::parse(line, msg)) return false;
  switch (msg->type) {
    case proto::MsgType::kCmd:
      s->on_cmd(msg->seq, msg->v_mm_s, msg->omega_mrad_s, now_ms);
      return true;
    case proto::MsgType::kArm:
      s->on_arm(msg->seq, now_ms);
      return true;
    case proto::MsgType::kStop:
      s->on_stop(msg->seq);
      return true;
    case proto::MsgType::kSim:
      return true;  // Auswertung beim Aufrufer
    case proto::MsgType::kInvalid:
      break;
  }
  return false;
}

}  // namespace app
