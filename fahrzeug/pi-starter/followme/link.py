"""Verbindung zum Pico: Befehle senden, Statusmeldungen empfangen.

Die Verbindung ist transportunabhängig: Sie braucht nur ein Objekt mit
read()/write() für Bytes. Auf dem Fahrzeug ist das die USB-Seriell-
Schnittstelle (pyserial), in Tests der Host-Simulator firmware/host_sim
(echte Firmwarelogik, simulierte Hardware) über eine Pipe.

Wichtig: Der Pico hält das Fahrzeug an, wenn länger als 300 ms kein CMD
kommt (Watchdog). Das Programm auf dem Pi muss also in seiner Hauptschleife
regelmäßig send_cmd() aufrufen — auch wenn es gerade nichts zu tun hat
(dann eben mit 0, 0).
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import BinaryIO, Protocol

from . import protocol
from .protocol import Info, Status, Ultrasonic
from .sim_binary import bundled_binary

STATUS_MAX_AGE_S = 0.5  # ANNAHME für Pi-Betrieb; USB/Jitter am Prototyp messen.


class McuLink:
    """Gesprächspartner Pico. Thread-sicher für einen Sender und einen Empfänger."""

    def __init__(self, reader: BinaryIO, writer: BinaryIO, on_close: Callable[[], None] | None = None,
                 on_event: Callable[[dict], None] | None = None):
        self._reader = reader
        self._writer = writer
        self._on_close = on_close
        self.on_event = on_event
        self._seq = 0
        self._write_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._status: Status | None = None
        self._status_time = 0.0  # time.monotonic() beim Empfang
        self._info: Info | None = None
        self._ultrasonic: Ultrasonic | None = None
        self._ultrasonic_time = 0.0  # time.monotonic() beim Empfang
        self._error: str | None = None  # verriegelt bis zum bewussten Neuöffnen
        self._closed = False
        self._fault_stop_attempted = False  # unter _write_lock
        self.bad_lines = 0  # vom Pi verworfene Zeilen (Prüfsumme/Format)
        self._running = True
        self._thread = threading.Thread(target=self._rx_loop, name="mcu-rx", daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ Senden
    def _fail(self, message: str) -> None:
        with self._state_lock:
            if self._error is None:
                self._error = message

    def _write(self, line: str) -> None:
        data = line.encode("ascii")
        try:
            if self._closed:
                raise OSError("Verbindung geschlossen")
            if self._writer.write(data) != len(data):
                raise OSError("Unvollständige Sendung")
            if not hasattr(self._writer, "in_waiting"):
                self._writer.flush()
            # pyserial.write reicht die Bytes direkt weiter. Dessen POSIX-flush
            # wartet mit tcdrain ohne write_timeout; deshalb hier kein Drain.
        except (OSError, ValueError) as exc:
            self._fail(f"Sendefehler: {exc}")
            raise ConnectionError(self.error) from exc

    def _send_locked(self, build: Callable[[int], str]) -> int:
        self._seq = (self._seq + 1) & 0xFFFF
        line = build(self._seq)
        write_started_s = time.monotonic() if self.on_event else None
        self._write(line)
        if self.on_event:
            self.on_event({"kind": "tx", "line": line.strip(), "seq": self._seq, "time_s": time.monotonic(),
                           "write_started_s": write_started_s})
        return self._seq

    def _next(self, build: Callable[[int], str], *, guarded: bool = False) -> int:
        with self._write_lock:
            if guarded:
                if self.status_age_s > STATUS_MAX_AGE_S:
                    self._fail("Keine frische STAT-Meldung; Verbindung bewusst neu öffnen")
                if self.error is not None:
                    if not self._fault_stop_attempted:
                        self._fault_stop_attempted = True
                        try:
                            self._send_locked(protocol.stop_line)
                        except ConnectionError:
                            pass  # Kein weiterer CMD: der Pico-Watchdog bleibt unabhängig.
                    raise ConnectionError(self.error)
            return self._send_locked(build)

    def send_cmd(self, v_mps: float, omega_rad_s: float) -> int:
        """Fahrbefehl: v [m/s] vorwärts positiv, ω [rad/s] links positiv."""
        return self._next(lambda s: protocol.cmd_line(s, v_mps, omega_rad_s), guarded=True)

    def arm(self) -> int:
        """Motoren scharf schalten (nötig nach Start, Watchdog, Not-Aus)."""
        return self._next(protocol.arm_line, guarded=True)

    def stop(self) -> int:
        """Geordneter Stopp: Pico geht in STANDBY."""
        return self._next(protocol.stop_line)

    def send_sim(self, key: str, value: int) -> None:
        """Nur Host-Simulator: Hardwarezustand verstellen (z. B. Not-Aus)."""
        with self._write_lock:
            self._write(protocol.sim_line(key, value))

    # ---------------------------------------------------------------- Empfangen
    def _rx_loop(self) -> None:
        buf = b""
        discarding = False
        while self._running:
            try:
                if hasattr(self._reader, "in_waiting"):
                    chunk = self._read_serial()
                else:
                    # Pipe (ungepuffert): read() liefert, was gerade da ist.
                    chunk = getattr(self._reader, "read1", self._reader.read)(4096)
            except (OSError, ValueError) as exc:
                self._fail(f"Empfangsfehler: {exc}")
                break
            if not chunk:
                if not hasattr(self._reader, "in_waiting"):
                    self._fail("Empfang beendet (EOF)")
                    break  # EOF der Pipe
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                if discarding:
                    discarding = False
                    continue
                try:
                    if len(raw) > 512:
                        raise ValueError("Überlange Zeile")
                    self._handle(raw.decode("ascii"))
                except (UnicodeError, ValueError):
                    self.bad_lines += 1
            if len(buf) > 512:  # Müll ohne Zeilenende verwerfen
                buf = b""
                if not discarding:
                    self.bad_lines += 1
                discarding = True

    def _read_serial(self) -> bytes:
        # pyserial: alles lesen, was da ist, sonst kurz blockieren (Timeout
        # wird beim Öffnen gesetzt), damit der Thread nicht im Leerlauf rotiert.
        n = self._reader.in_waiting  # type: ignore[attr-defined]
        return self._reader.read(n if n > 0 else 1)

    def _handle(self, line: str) -> None:
        fields = protocol.unframe(line)
        if fields is None:
            self.bad_lines += 1
            return
        if self.on_event:
            self.on_event({"kind": "rx", "line": line, "time_s": time.monotonic()})
        if fields[0] == "STAT":
            st = protocol.parse_status(fields)
            if st is None or not 0 <= st.t_ms <= 0xFFFFFFFF or st.state not in ("STANDBY", "RUN", "ESTOP"):
                self.bad_lines += 1
                return
            with self._state_lock:
                if self._status is not None:
                    delta_ms = (st.t_ms - self._status.t_ms) & 0xFFFFFFFF
                    if delta_ms == 0:
                        return  # Wiederholte alte Meldung ist kein Lebenszeichen.
                    if delta_ms >= 0x80000000:
                        if self._error is None:
                            self._error = "Pico-Zeit rückwärts (Neustart/alte Daten); Verbindung neu öffnen"
                        return
                self._status = st
                self._status_time = time.monotonic()
        elif fields[0] == "INFO":
            info = protocol.parse_info(fields)
            if info is None:
                self.bad_lines += 1
                return
            with self._state_lock:
                self._info = info
        elif fields[0] == "US":
            us = protocol.parse_ultrasonic(fields)
            if us is None:
                self.bad_lines += 1
                return
            with self._state_lock:
                if self._ultrasonic is not None and us.t_ms == self._ultrasonic.t_ms:
                    return  # Wiederholte alte Meldung ist kein neuer Messstand.
                self._ultrasonic = us
                self._ultrasonic_time = time.monotonic()

    # ------------------------------------------------------------------ Abfragen
    @property
    def error(self) -> str | None:
        """Erster Verbindungsfehler; neue STAT-Meldungen löschen ihn niemals."""
        with self._state_lock:
            return self._error

    @property
    def status(self) -> Status | None:
        with self._state_lock:
            return self._status

    @property
    def status_age_s(self) -> float:
        """Alter der letzten STAT-Meldung in s (inf, wenn noch keine kam)."""
        with self._state_lock:
            return time.monotonic() - self._status_time if self._status else float("inf")

    def ultrasonic_snapshot(self) -> tuple[Ultrasonic | None, float | None]:
        """Letzte US-Meldung (Protokoll 2) und ihr Empfangszeitpunkt in time.monotonic().

        Nur Messwerte für Teamaufgaben; keine Fahrfreigabe. Mit Firmware der
        Version 1 bleibt beides None.
        """
        with self._state_lock:
            if self._ultrasonic is None:
                return None, None
            return self._ultrasonic, self._ultrasonic_time

    def status_snapshot(self) -> tuple[Status | None, float | None, str | None]:
        """Status, Empfangsalter und Fehler atomar; keine Einzelmesszeit des ToF."""
        with self._state_lock:
            age = time.monotonic() - self._status_time if self._status is not None else None
            return self._status, age, self._error

    @property
    def info(self) -> Info | None:
        with self._state_lock:
            return self._info

    def wait_status(self, predicate: Callable[[Status], bool] = lambda s: True, timeout_s: float = 2.0) -> Status:
        """Wartet auf passenden, frischen Status; Cache ist keine Empfangsbestätigung.

        Für einen *neuen* Prüfschritt zusätzlich t_ms/last_seq im Prädikat prüfen.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._state_lock:
                st, error = self._status, self._error
                fresh = st is not None and time.monotonic() - self._status_time <= STATUS_MAX_AGE_S
            if error:
                raise ConnectionError(error)
            if fresh and st is not None and predicate(st):
                return st
            time.sleep(0.01)
        raise TimeoutError(f"Keine passende STAT-Meldung in {timeout_s} s (letzte: {self.status})")

    def close(self) -> None:
        with self._write_lock:
            if self._closed:
                return
            self._closed = True
            self._running = False
            self._fail("Verbindung geschlossen")
            if self._writer is not self._reader:
                try:
                    self._writer.close()
                except OSError:
                    pass
        try:
            if self._on_close:
                self._on_close()
        finally:
            # Prozess/serieller Timeout (0,05 s) löst den blockierten Leseaufruf.
            # Erst join, dann lesen/schreiben gemeinsam genutzten Port schließen:
            # pyserial.close() parallel zu read() kann dessen fd auf None setzen.
            self._thread.join(timeout=.15)
            self._reader.close()
            self._thread.join(timeout=1.0)

    def __enter__(self):
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


class CommandSink(Protocol):
    """Der Sender benötigt nur den Fahrbefehl, nicht die gesamte Verbindung."""

    def send_cmd(self, v_mps: float, omega_rad_s: float) -> int: ...


class SetpointSender:
    """Sendet den jeweils letzten Sollwert mit fester Rate an den Pico.

    Warum: Auf dem Raspberry Pi 3B braucht die Personenerkennung pro Bild
    mehrere hundert Millisekunden. Würde die Bildschleife selbst senden,
    bekäme der Pico zeitweise > 300 ms kein CMD und hielte an (Watchdog).
    Dieser Thread hält die Verbindung nominell im 20-Hz-Takt am Leben. Nach
    verpassten Takten wird neu angesetzt, nicht per CMD-Nachholburst aufgeholt.

    Sicherheit bleibt erhalten:
      * Ist der Sollwert älter als max_age_s (Bildschleife hängt), sendet
        der Thread 0/0 — das Fahrzeug fährt nicht blind weiter.
      * Hängt oder stirbt der ganze Prozess, verstummt auch dieser Thread
        ⇒ der Watchdog des Pico greift wie bisher.
      * McuLink verriegelt bei fehlendem Rückkanal, versucht einmal STOP
        und lehnt weitere CMD/ARM ab. Der Sender meldet den Fehler über check().
    """

    def __init__(self, link: CommandSink, rate_hz: float = 20.0, max_age_s: float = 0.6,
                 *, on_tick: Callable[[dict], None] | None = None):
        if not all(math.isfinite(v) and v > 0 for v in (rate_hz, max_age_s)):
            raise ValueError("Senderate und maximales Alter müssen endlich und positiv sein")
        self._link = link
        self._period = 1.0 / rate_hz
        self.max_age_s = max_age_s
        self._lock = threading.Lock()
        self._v = 0.0
        self._w = 0.0
        self._t: float | None = None  # Zeit der Beobachtung, NICHT Ende der Erkennung
        self.stale_count = 0  # wie oft wegen veralteten Sollwerts 0 gesendet wurde
        self._running = False
        self._thread: threading.Thread | None = None
        self.error: str | None = None
        self.on_tick = on_tick  # optionale Host-Diagnose; kein Teil des Drahtprotokolls

    def check(self) -> None:
        """Senderfehler in der Bildschleife melden, statt lautlos weiterzurechnen."""
        if self.error is not None:
            raise ConnectionError(self.error)

    def set(self, v_mps: float, omega_rad_s: float, *, observed_at_s: float | None = None) -> None:
        now_s = time.monotonic()
        source_s = now_s if observed_at_s is None else observed_at_s
        if not all(math.isfinite(v) for v in (v_mps, omega_rad_s, source_s)) or source_s > now_s:
            self.invalidate()
            raise ValueError("Ungültiger Sollwert oder zukünftiger Beobachtungszeitpunkt")
        with self._lock:
            if self._t is not None and source_s < self._t:
                self._v, self._w = 0.0, 0.0
                return  # verspätetes Ergebnis darf keine neuere Beobachtung ersetzen
            self._v, self._w, self._t = v_mps, omega_rad_s, source_s

    def invalidate(self) -> None:
        with self._lock:
            self._v, self._w, self._t = 0.0, 0.0, None

    def current(self) -> tuple[float, float, bool]:
        """(v, ω, veraltet) — was jetzt gesendet würde."""
        with self._lock:
            stale = self._t is None or time.monotonic() - self._t > self.max_age_s
            return (0.0, 0.0, True) if stale else (self._v, self._w, False)

    def send_once(self) -> None:
        v, w, stale = self.current()
        if stale and self._t is not None:
            self.stale_count += 1
        self._link.send_cmd(v, w)

    def _loop(self) -> None:
        next_t = time.monotonic()
        while self._running:
            try:
                started_s = time.monotonic() if self.on_tick else None
                self.send_once()
                if self.on_tick:
                    self.on_tick({"kind": "sender_tick", "planned_s": next_t,
                                  "started_s": started_s, "time_s": time.monotonic()})
            except (OSError, ValueError) as exc:
                self.error = str(exc)
                self.invalidate()
                self._running = False
                return  # Verbindung weg: Watchdog des Pico übernimmt
            next_t += self._period
            now_s = time.monotonic()
            if next_t <= now_s:
                next_t = now_s + self._period  # verpasste Takte verwerfen, keine alten Slots abarbeiten
            time.sleep(next_t - now_s)

    def start(self) -> SetpointSender:
        self.check()  # Ein gescheiterter Sender wird nicht still neu gestartet.
        if self._thread is not None:
            raise RuntimeError("Sender wurde bereits gestartet")
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="sollwert-sender", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)


def open_serial(port: str = "/dev/ttyACM0") -> McuLink:
    """Verbindung über USB-Seriell (Pico als /dev/ttyACM0)."""
    import serial  # pyserial, BSD-3-Clause; erst hier importiert, damit Tests ohne laufen

    ser = serial.Serial(port, 115200, timeout=0.05, write_timeout=0.1)
    return McuLink(ser, ser)


def default_sim_binary() -> Path:
    return Path(__file__).resolve().parents[2] / "firmware" / "host_sim" / ("mcu_sim.exe" if os.name == "nt" else "mcu_sim")


def open_sim(binary: Path | None = None, *, cmd_trace: Path | None = None) -> McuLink:
    """Startet den Host-Simulator (baut ihn bei Bedarf mit make)."""
    if cmd_trace is not None:
        cmd_trace = Path(cmd_trace).absolute()
        if cmd_trace.exists() or cmd_trace.is_symlink():
            raise FileExistsError(cmd_trace)
        if not cmd_trace.parent.is_dir():
            raise FileNotFoundError(cmd_trace.parent)
    if binary is None:
        binary = default_sim_binary()
        # make prüft Zeitstempel auch bei vorhandenem Binary. Sonst liefen
        # lokale Integrationstests nach Firmwareänderungen gegen alten Code.
        ready = bundled_binary(binary.parent)
        if ready is not None:
            binary = ready
        elif os.name == "nt":
            subprocess.run([sys.executable, str(binary.parent / "build.py")], check=True)
        else:
            subprocess.run(["make", "-s", "-C", str(binary.parent)], check=True)
    command = [str(binary)] + (["--cmd-trace", str(cmd_trace)] if cmd_trace is not None else [])
    proc = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
    assert proc.stdin is not None and proc.stdout is not None

    def _cleanup() -> None:
        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

    return McuLink(proc.stdout, proc.stdin, on_close=_cleanup)
