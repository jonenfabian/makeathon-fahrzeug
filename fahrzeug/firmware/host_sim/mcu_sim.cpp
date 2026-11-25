// mcu_sim.cpp — Host-Simulator des Pico: echte Firmwarelogik, simulierte Hardware.
//
// Zweck: Die Pi-Software (pi/followme) kann ohne Fahrzeug gegen genau die
// Logik getestet werden, die auf dem Pico läuft. Supervisor, Radregler,
// Parser und Meldungsformat sind dieselben Quelldateien wie in der
// Zielfirmware (src/*.cpp). Nur Motoren, Encoder, Abstandssensor und
// Spannungsmessung sind hier nachgebildet.
//
// Schnittstelle: Zeilen auf stdin (wie die USB-Seriell-Verbindung), STAT-
// und INFO-Zeilen auf stdout. Echtzeit (1 Simulationssekunde = 1 s).
// Zusätzliche SIM-Befehle (nur hier, die Zielfirmware ignoriert sie):
//   $SIM,VMOT,<mV>      Motorspannung setzen (z. B. 0 = Not-Aus gedrückt)
//   $SIM,TOF,<mm>       Abstand vor dem Fahrzeug (65535 = kein Ziel)
//   $SIM,TOFFAIL,<0|1>  Abstandssensor fällt aus / kommt zurück
//   $SIM,BUMPER,<0|1>   Stoßfänger gedrückt
//   $SIM,ESTOP,<0|1>    Not-Aus-Taster drücken/entriegeln (Relais fällt ab,
//                       Motorspannung → 0, Öffner NC2 öffnet ⇒ GP15 HIGH)
//   $SIM,RELAYWELD,<0|1> Relaiskontakt klebt (Spannung bleibt beim Drücken)
//   $SIM,ENC_L,<0|1|2>  Encoder links: 1 angeschlossen, 0 Leitung ab (beide
//   $SIM,ENC_R,<0|1|2>  Kanäle), 2 nur Kanal A ab (Steckbrücke gelöst)
//   $SIM,US_L,<wert>    Ultraschall schräg vorn links bzw. rechts: Abstand in
//   $SIM,US_R,<wert>    mm (> 0), 0 zu nah, -1 kein Ziel (Vorgabe), -2 Sensor
//                       fehlt (Echo bleibt LOW), -3 Echo hängt HIGH. Der
//                       simulierte Sensor antwortet mit Flanken auf den
//                       Trigger; ausgewertet wird mit dem echten src/sonar.h.
// Meldungen: STAT, US (je 25 Hz) und INFO wie die Zielfirmware.
// Ende: stdin schließen (EOF).
//
// Bauen: make -C firmware/host_sim
#include <fcntl.h>
#include <string.h>
#include <time.h>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#include <io.h>
#include <sys/stat.h>
#else
#include <poll.h>
#include <unistd.h>
#endif

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>
#ifdef __APPLE__
#include <mach/mach_time.h>
#endif

#include "../followme_mcu/config.h"
#include "../followme_mcu/src/app.h"
#include "../followme_mcu/src/kinematics.h"
#include "../followme_mcu/src/proto.h"
#include "../followme_mcu/src/sonar.h"
#include "../followme_mcu/src/supervisor.h"
#include "../followme_mcu/src/wheel_ctrl.h"
#include "wheel_plant.h"  // Radmodell (PT1) und Encoder-Nachbildung

namespace {

uint64_t elapsed_us() {
  using namespace std::chrono;
  static const auto t0 = steady_clock::now();
  return static_cast<uint64_t>(duration_cast<microseconds>(steady_clock::now() - t0).count());
}

uint32_t now_ms() { return static_cast<uint32_t>(elapsed_us() / 1000u); }
uint32_t now_us() { return static_cast<uint32_t>(elapsed_us()); }

// Simulierter HC-SR04: antwortet auf den Trigger mit Echo-Flanken wie der
// echte Sensor (Anstieg nach ≈ 0,46 ms, Dauer = Laufzeit hin und zurück).
struct SimSonar {
  static constexpr int32_t kNoTarget = -1, kMissing = -2, kStuck = -3;
  int32_t value = kNoTarget;
  bool triggered = false;
  uint32_t rise_at = 0, fall_at = 0;

  static bool reached(uint32_t t, uint32_t at) { return static_cast<int32_t>(t - at) >= 0; }

  void trigger(uint32_t t_us) {
    triggered = value != kMissing && value != kStuck;
    rise_at = t_us + 460u;
    uint32_t pulse_us = 38000u;  // kein Echo: der Sensor gibt nach ≈ 38 ms auf
    if (value == 0) pulse_us = 60u;
    if (value > 0) pulse_us = static_cast<uint32_t>(value) * 2000u / US_SOUND_MM_PER_MS;
    fall_at = rise_at + pulse_us;
  }
  bool high(uint32_t t) const {
    if (value == kStuck) return true;
    return triggered && reached(t, rise_at) && !reached(t, fall_at);
  }
  sonar::Edges edges(uint32_t t) const {
    sonar::Edges e;
    if (!triggered || !reached(t, rise_at)) return e;
    e.rose = true;
    e.rise_us = rise_at;
    e.fell = reached(t, fall_at);
    e.fall_us = e.fell ? fall_at : 0u;
    return e;
  }
};

void emit(const char* line) {
  fputs(line, stdout);
  fflush(stdout);
}

uint64_t monotonic_ns() {
#ifdef __APPLE__
  static const mach_timebase_info_data_t scale = [] {
    mach_timebase_info_data_t value{};
    mach_timebase_info(&value);
    return value;
  }();
  return static_cast<uint64_t>(static_cast<long double>(mach_absolute_time()) * scale.numer / scale.denom);
#elif defined(_WIN32)
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
#else
  timespec ts{};
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
#endif
}

#ifdef __APPLE__
constexpr const char* kTraceClock = "mach_absolute_time";
#elif defined(_WIN32)
constexpr const char* kTraceClock = "steady_clock";
#else
constexpr const char* kTraceClock = "CLOCK_MONOTONIC";
#endif

// Nur optionale Host-Diagnose: keine Datei-I/O/Allokation im Empfangspfad.
// Überlauf wird sichtbar; Schreiben erst nach EOF, niemals im Regelbetrieb.
class CmdTrace {
 public:
  static constexpr size_t kCapacity = 32768;
  bool open(const char* path) {
#ifdef _WIN32
    const int fd = ::_open(path, _O_WRONLY | _O_CREAT | _O_EXCL | _O_BINARY, _S_IREAD | _S_IWRITE);
    if (fd < 0) return false;
    file_ = _fdopen(fd, "w");
    if (!file_) { _close(fd); return false; }
#else
    const int fd = ::open(path, O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (fd < 0) return false;
    file_ = fdopen(fd, "w");
    if (!file_) { close(fd); return false; }
#endif
    events_.reserve(kCapacity);
    started_ = monotonic_ns();
    return true;
  }
  bool enabled() const { return file_ != nullptr; }
  void add(uint64_t read_ns, uint32_t read_ms, const proto::Message& msg) {
    const uint64_t accepted_ns = monotonic_ns();
    ++received_;
    if (events_.size() == kCapacity) { ++dropped_; return; }
    events_.push_back({read_ns, accepted_ns, read_ms, msg.seq, msg.v_mm_s, msg.omega_mrad_s});
  }
  bool finish() {
    if (!file_) return true;
    const uint64_t ended = monotonic_ns();
    bool ok = fprintf(file_, "{\"kind\":\"sim_trace\",\"format\":1,\"clock\":\"%s\","
                      "\"started_ns\":%llu,\"capacity\":%zu}\n",
                      kTraceClock, static_cast<unsigned long long>(started_), kCapacity) > 0;
    for (const auto& e : events_) {
      ok = fprintf(file_, "{\"kind\":\"cmd\",\"read_ns\":%llu,\"accepted_ns\":%llu,"
                   "\"firmware_ms\":%u,\"seq\":%u,\"v_mm_s\":%d,\"omega_mrad_s\":%d}\n",
                   static_cast<unsigned long long>(e.read_ns), static_cast<unsigned long long>(e.accepted_ns),
                   e.read_ms, e.seq, e.v, e.w) > 0 && ok;
    }
    ok = fprintf(file_, "{\"kind\":\"end\",\"ended_ns\":%llu,\"received\":%llu,\"written\":%zu,"
                 "\"dropped\":%llu}\n", static_cast<unsigned long long>(ended),
                 static_cast<unsigned long long>(received_), events_.size(),
                 static_cast<unsigned long long>(dropped_)) > 0 && ok;
    ok = fclose(file_) == 0 && ok;
    file_ = nullptr;
    return ok;
  }
 private:
  struct Event { uint64_t read_ns, accepted_ns; uint32_t read_ms; uint16_t seq; int32_t v, w; };
  FILE* file_ = nullptr;
  std::vector<Event> events_;
  uint64_t started_ = 0, received_ = 0, dropped_ = 0;
};

}  // namespace

// Nur Host-Ein-/Ausgabe: Der Firmwarekern und sein Regeltakt bleiben identisch.
// -2 bedeutet: nach einem Millisekunden-Takt noch keine neuen Eingabebytes.
static int input_bytes(char* buf, unsigned size) {
#ifdef _WIN32
  const HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
  DWORD available = 0;
  if (!PeekNamedPipe(handle, nullptr, 0, nullptr, &available, nullptr))
    return GetLastError() == ERROR_BROKEN_PIPE ? 0 : -1;
  if (available == 0) { Sleep(1); return -2; }
  return _read(_fileno(stdin), buf, available < size ? available : size);
#else
  pollfd pfd{0, POLLIN, 0};
  if (poll(&pfd, 1, 1) <= 0) return -2;
  return static_cast<int>(read(0, buf, size));
#endif
}

int main(int argc, char** argv) {
#ifdef _WIN32
  _setmode(_fileno(stdin), _O_BINARY);
  _setmode(_fileno(stdout), _O_BINARY);
#endif
  CmdTrace trace;
  if (argc != 1 && (argc != 3 || strcmp(argv[1], "--cmd-trace") != 0)) {
    fputs("Aufruf: mcu_sim [--cmd-trace NEUE_DATEI]\n", stderr);
    return 2;
  }
  if (argc == 3 && !trace.open(argv[2])) {
    perror("CMD-Diagnosedatei nicht exklusiv geöffnet");
    return 2;
  }
  sup::Supervisor s;
  wheel::WheelCtrl ctrl_l, ctrl_r;
  proto::LineReader reader;
  plant::Wheel plant_l, plant_r;
  uint32_t rx_errors = 0;

  int32_t vmot_mv = 12000;
  uint16_t tof_mm = sup::kTofNoTarget;
  bool tof_alive = true;
  bool bumper = false;
  bool estop = false;
  bool relay_weld = false;
  sonar::Cycle sonar_cycle;
  SimSonar us[sonar::kCount];

  uint32_t last_ctrl = now_ms(), last_stat = last_ctrl, last_tof = last_ctrl;
  uint32_t stat_count = 0;
  int32_t last_vmot = vmot_mv;
  int32_t prev_l = 0, prev_r = 0;
  float v_l = 0.0f, v_r = 0.0f;
  char line[proto::kMaxLine + 8];

  for (;;) {
    // --- Eingabe: stdin mit 1 ms Wartezeit abfragen ---------------------------
    char buf[256];
    const int n = input_bytes(buf, sizeof(buf));
    if (n != -2) {
      const uint64_t read_ns = trace.enabled() ? monotonic_ns() : 0;
      if (n <= 0) return trace.finish() && n == 0 ? 0 : 1;
      const uint32_t t = now_ms();
      for (int i = 0; i < n; ++i) {
        if (!reader.feed(buf[i])) continue;
        proto::Message msg;
        if (!app::handle_line(reader.line(), &s, t, &msg)) {
          ++rx_errors;
          continue;
        }
        if (trace.enabled() && msg.type == proto::MsgType::kCmd) trace.add(read_ns, t, msg);
        if (msg.type == proto::MsgType::kSim) {
          if (strcmp(msg.sim_key, "VMOT") == 0) vmot_mv = msg.sim_value;
          else if (strcmp(msg.sim_key, "TOF") == 0) tof_mm = static_cast<uint16_t>(msg.sim_value);
          else if (strcmp(msg.sim_key, "TOFFAIL") == 0) tof_alive = msg.sim_value == 0;
          else if (strcmp(msg.sim_key, "BUMPER") == 0) bumper = msg.sim_value != 0;
          else if (strcmp(msg.sim_key, "ESTOP") == 0) estop = msg.sim_value != 0;
          else if (strcmp(msg.sim_key, "RELAYWELD") == 0) relay_weld = msg.sim_value != 0;
          else if (strcmp(msg.sim_key, "ENC_L") == 0 && msg.sim_value >= 0 && msg.sim_value <= 2)
            plant_l.link = static_cast<plant::EncLink>(msg.sim_value);
          else if (strcmp(msg.sim_key, "ENC_R") == 0 && msg.sim_value >= 0 && msg.sim_value <= 2)
            plant_r.link = static_cast<plant::EncLink>(msg.sim_value);
          else if (strcmp(msg.sim_key, "US_L") == 0 && msg.sim_value >= SimSonar::kStuck && msg.sim_value < 65535)
            us[sonar::kLeft].value = msg.sim_value;
          else if (strcmp(msg.sim_key, "US_R") == 0 && msg.sim_value >= SimSonar::kStuck && msg.sim_value < 65535)
            us[sonar::kRight].value = msg.sim_value;
          else ++rx_errors;
        }
      }
    }

    const uint32_t t = now_ms();

    // --- Abstandssensor: alle 40 ms ein Messwert, solange "angeschlossen" ---
    if (t - last_tof >= 40) {
      last_tof = t;
      if (tof_alive) s.on_tof(tof_mm, t);
    }

    // --- Ultraschall: dieselbe Auswertung wie us_poll() im .ino --------------
    {
      const uint32_t tu = now_us();
      const bool high[sonar::kCount] = {us[sonar::kLeft].high(tu), us[sonar::kRight].high(tu)};
      const int act = sonar_cycle.active();
      const sonar::Edges e = act >= 0 ? us[act].edges(tu) : sonar::Edges{};
      const int trig = sonar_cycle.step(tu, t, high, e);
      if (trig >= 0) us[trig].trigger(tu);
    }

    // --- Regeltakt 10 ms: gleiche Reihenfolge wie control_step() im .ino ----
    if (t - last_ctrl >= CTRL_PERIOD_MS) {
      last_ctrl = t;
      const float dt_s = CTRL_PERIOD_MS / 1000.0f;
      const int32_t tl = plant_l.ticks(), tr = plant_r.ticks();
      v_l += SPEED_FILTER_ALPHA * (kin::ticks_to_mm(tl - prev_l, WHEEL_DIAMETER_MM, ENCODER_COUNTS_PER_REV) / dt_s - v_l);
      v_r += SPEED_FILTER_ALPHA * (kin::ticks_to_mm(tr - prev_r, WHEEL_DIAMETER_MM, ENCODER_COUNTS_PER_REV) / dt_s - v_r);
      prev_l = tl;
      prev_r = tr;
      // Relais: bei gedrücktem Not-Aus fällt es ab (Motorspannung 0), außer
      // der Kontakt klebt. Kanal 2: Der Öffner NC2 öffnet, der Pull-up zieht
      // GP15 auf HIGH; ausgewertet wie im .ino über estop_pressed_from_pin.
      const int32_t vmot_eff = (estop && !relay_weld) ? 0 : vmot_mv;
      const bool estop_pin_high = estop;
      sup::WheelFeedback fb;  // Encoder-Plausibilität: anliegender Tastgrad, Zählerstand
      fb.duty_l = ctrl_l.last_duty();
      fb.duty_r = ctrl_r.last_duty();
      fb.ticks_l = tl;
      fb.ticks_r = tr;
      s.tick(t, vmot_eff, bumper, sup::estop_pressed_from_pin(estop_pin_high), fb);
      const bool en = s.motors_enabled();
      const float duty_l = ctrl_l.update(s.wheel_left_mm_s(), v_l, static_cast<float>(vmot_eff), dt_s, en);
      const float duty_r = ctrl_r.update(s.wheel_right_mm_s(), v_r, static_cast<float>(vmot_eff), dt_s, en);
      // Gebremst (Tastgrad 0) läuft das Rad mit derselben Zeitkonstante aus.
      plant_l.step(duty_l, vmot_eff, dt_s);
      plant_r.step(duty_r, vmot_eff, dt_s);
      last_vmot = vmot_eff;
    }

    // --- Meldungen ------------------------------------------------------------
    if (t - last_stat >= STAT_PERIOD_MS) {
      last_stat = t;
      app::StatData d;
      d.t_ms = t;
      d.state = s.state();
      d.flags = s.flags();
      d.last_seq = s.last_seq();
      d.ticks_l = plant_l.ticks();
      d.ticks_r = plant_r.ticks();
      d.v_l_mm_s = lroundf(v_l);
      d.v_r_mm_s = lroundf(v_r);
      d.tof_mm = s.tof_mm();
      d.vmot_mv = last_vmot;
      d.rx_errors = rx_errors + reader.overflow_count();
      if (app::format_stat(d, line, sizeof(line)) > 0) emit(line);
      if (app::format_us(t, sonar_cycle.sample(sonar::kLeft), sonar_cycle.sample(sonar::kRight), line,
                         sizeof(line)) > 0)
        emit(line);
      if (++stat_count % INFO_EVERY_N_STAT == 0 && app::format_info(line, sizeof(line)) > 0) emit(line);
    }
  }
}
