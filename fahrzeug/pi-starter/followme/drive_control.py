"""Gemeinsame Bedienung für Browser und Terminal; Schutzfunktionen bleiben im Pico."""
from __future__ import annotations

import secrets
import threading
import time

from .link import STATUS_MAX_AGE_S
from .protocol import supported_protocol

BROWSER_TIMEOUT_S = 2.0  # Bedienverbindung; keine garantierte physische Stoppzeit.
# Bildschleife/Teamcode hängt: Fahrt beenden statt nach dem Hänger von selbst weiterzufahren.
# ANNAHME: 1 s liegt sicher über der realen Bildlatenz (fm01 19.11.: max. 350 ms); bei sehr langsamen
# Erkennern am Prototyp mit dem Fahrtenschreiber prüfen.
LOOP_TIMEOUT_S = 1.0
# Laden der Aufgabendatei beim Start, für Browser und Terminal gleich. Ein später fertig werdender
# Ladevorgang führt nie mehr zu ARM.
LOAD_TIMEOUT_S = 1.5


class DriveControl:
    def __init__(self, link, sender, prepare, initial_context, *, clear=lambda: None,
                 clock=time.monotonic, browser_timeout_s=BROWSER_TIMEOUT_S, loop_timeout_s=LOOP_TIMEOUT_S,
                 task_error=None):
        self.link, self.sender, self.prepare = link, sender, prepare
        self.context, self.clear, self.clock = initial_context, clear, clock
        self.browser_timeout_s = browser_timeout_s
        self.loop_timeout_s = loop_timeout_s
        self.task_error = task_error
        self.last_frame_s = 0.
        self.lock = threading.RLock()
        self.instance = secrets.token_urlsafe(18)
        self.revision = 0
        self.generation = 0
        self.active = False
        self.owner = None
        self.lease = None
        self.deadline_s = 0.
        self.started_s = 0.
        self.confirmed_run = False
        self.reason = ("Fehler in der Aufgabendatei. Datei korrigieren, dann Fahrt starten." if task_error
                       else "Bereit. Fahrt bewusst starten.")
        self.error = None
        self._closed = threading.Event()
        self._monitor = threading.Thread(target=self._watch, name="fahrbedienung", daemon=True)
        self._monitor.start()

    def capture(self):
        with self.lock:
            return self.generation, self.context

    def _ready(self):
        self.sender.check()
        st, age, error = self.link.status_snapshot()
        if self.error or error:
            raise ValueError(self.error or error)
        if st is None or age is None or not 0 <= age <= STATUS_MAX_AGE_S:
            raise ValueError("Kein frischer Pico-Status. Verbindung prüfen.")
        if st.state != "STANDBY" or abs(st.v_l_mm_s) > 2 or abs(st.v_r_mm_s) > 2:
            raise ValueError("Zuerst stoppen und Stillstand im Pico-Status prüfen.")
        if self.link.info is not None and not supported_protocol(self.link.info.proto_version):
            raise ValueError("Nicht unterstützte Pico-Protokollversion.")

    def start(self, *, instance=None, revision=None, browser=True):
        with self.lock:
            if self._closed.is_set():
                raise ValueError("Fahrprogramm beendet.")
            if browser and (instance != self.instance or type(revision) is not int or revision != self.revision):
                raise ValueError("Ansicht veraltet. Zustand laden und bewusst neu starten.")
            if self.active:
                raise ValueError("Eine Fahrt läuft bereits. Alle können sie stoppen.")
            self._ready()
            expected_revision = self.revision
            requested_s = self.clock()
        # Studierendencode darf STOPP nicht blockieren. Noch keine Motorfreigabe.
        try:
            context = self.prepare()
        except KeyboardInterrupt:
            raise
        except BaseException as exc:  # auch sys.exit() im Teamcode ist nur ein Ladefehler.
            with self.lock:
                self.task_error = str(exc) or type(exc).__name__
                self.reason = "Fehler in der Aufgabendatei. Datei korrigieren, dann Fahrt starten."
            raise ValueError("Aufgabendatei konnte nicht geladen werden: " + self.task_error) from exc
        with self.lock:
            if self._closed.is_set() or expected_revision != self.revision or self.active:
                raise ValueError("Start durch eine neuere Bedienung verworfen.")
            if self.clock() - requested_s > LOAD_TIMEOUT_S:
                # Gilt auch für das Terminal: kein ARM Sekunden nach dem Tastendruck.
                self.task_error = ("Aufgabe lädt zu lange (über 1,5 s). Aufwendige Arbeit nicht beim Import "
                                   "erledigen, dann bewusst neu starten.")
                self.reason = "Fehler in der Aufgabendatei. Datei korrigieren, dann Fahrt starten."
                raise ValueError(self.task_error)
            self._ready()
            self.sender.invalidate()
            self.task_error = None
            self.context = context
            self.generation += 1
            self.revision += 1
            self.clear()
            try:
                self.link.arm()
            except (OSError, ValueError) as exc:
                self._stop("Start fehlgeschlagen: " + str(exc))
                raise ValueError(self.reason) from None
            self.active = True
            self.owner = "browser" if browser else "terminal"
            self.lease = secrets.token_urlsafe(24) if browser else None
            self.started_s = self.clock()
            self.last_frame_s = self.started_s
            self.deadline_s = self.started_s + self.browser_timeout_s
            self.confirmed_run = False
            self.reason = "Start gesendet. Warte auf RUN vom Pico."
            return {"lease": self.lease, "revision": self.revision, "instance": self.instance}

    def _stop(self, reason):
        self.active, self.owner, self.lease = False, None, None
        self.generation += 1
        self.revision += 1
        self.sender.invalidate()
        self.clear()
        self.reason = reason
        try:
            self.link.stop()
        except (OSError, ValueError) as exc:
            self.error = "STOPP nicht gesendet: " + str(exc)
            self.reason = self.error

    def stop(self):
        with self.lock:
            self._stop("Stopp gesendet. Stillstand im Pico-Status prüfen.")

    def frame_done(self):
        """Die Bildschleife hat ein Bild samt Teamcode fertig verarbeitet."""
        with self.lock:
            self.last_frame_s = self.clock()

    def task_failed(self, message, fallback_context):
        """Fehler im Teamcode: Fahrt stoppen, Meldung für alle Seiten, Programm läuft weiter.

        Bis zum nächsten Start (der die Datei neu lädt) rechnet die Schleife mit ``fallback_context``,
        damit die fehlerhafte Aufgabe nicht in jedem Bild erneut aufgerufen wird.
        """
        with self.lock:
            self.task_error = message
            self.context = fallback_context
            self.generation += 1  # Ergebnisse aus der fehlerhaften Aufgabe nie mehr senden.
            if self.active:
                self._stop("Fahrt gestoppt: Fehler in der Aufgabendatei. Datei korrigieren, dann neu starten.")
            else:
                self.reason = "Fehler in der Aufgabendatei. Datei korrigieren, dann Fahrt starten."

    def heartbeat(self, lease):
        with self.lock:
            self._tick()
            if not isinstance(lease, str) or not self.lease or not secrets.compare_digest(lease, self.lease):
                raise ValueError("Browserfahrt beendet. Bewusst neu starten.")
            self.deadline_s = self.clock() + self.browser_timeout_s

    def release(self, lease):
        with self.lock:
            # Eine alte Seite darf eine inzwischen neu gestartete Fahrt nicht beenden.
            if isinstance(lease, str) and self.lease and secrets.compare_digest(lease, self.lease):
                self._stop("Bedienseite verlassen. Für die nächste Fahrt neu starten.")

    def _tick(self):
        if not self.active:
            return
        now = self.clock()
        st, age, error = self.link.status_snapshot()
        if self.owner == "browser" and now >= self.deadline_s:
            self._stop("Browserverbindung unterbrochen. Bewusst neu starten.")
        elif now - self.last_frame_s > self.loop_timeout_s:
            self._stop("Fahrt gestoppt: Bildverarbeitung oder Aufgabe hängt. Danach bewusst neu starten.")
        elif error or self.sender.error or st is None or age is None or not 0 <= age <= STATUS_MAX_AGE_S:
            self._stop("Pico-Verbindung gestört. Verbindung und Stillstand prüfen.")
        elif st.state == "RUN":
            self.confirmed_run = True
            self.reason = "Fahrt läuft."
        elif self.confirmed_run or st.state == "ESTOP" or now - self.started_s >= 1.0:
            self._stop("Pico hat die Fahrt beendet oder den Start abgelehnt. Status prüfen.")

    def _watch(self):
        while not self._closed.wait(.05):
            with self.lock:
                self._tick()

    def snapshot(self):
        with self.lock:
            st, age, error = self.link.status_snapshot()
            fresh = st is not None and age is not None and 0 <= age <= STATUS_MAX_AGE_S and not error
            stopped = bool(fresh and st.state == "STANDBY" and abs(st.v_l_mm_s) <= 2 and abs(st.v_r_mm_s) <= 2)
            v, w, stale = self.sender.current()
            return {"enabled": True, "instance": self.instance, "revision": self.revision,
                    "active": self.active, "owner": self.owner, "stopped_confirmed": stopped,
                    "reason": self.reason, "error": self.error, "task_error": self.task_error,
                    "command": {"v_mps": v if self.active else 0.,
                                "omega_rad_s": w if self.active else 0., "stale": stale}}

    def close(self):
        self._closed.set()
        with self.lock:
            self._stop("Fahrprogramm beendet.")
        self._monitor.join(timeout=1)


class TerminalControl:
    """Tasten r, s, q aus dem Pi-Terminal. Stopp und Ende wirken sofort, auch während eine Aufgabe lädt.

    ``r`` lädt die Aufgabe in einem eigenen Thread. So blockiert langsamer oder hängender Teamcode weder
    die Tastatur noch ``s``/``q``. Ein Stopp entwertet einen noch ladenden Start (neue Revision in
    DriveControl), die Ladegrenze verhindert ein spätes ARM.
    """

    def __init__(self, controls, *, on_quit, on_fatal, say=print, clock=time.monotonic):
        self.controls, self.on_quit, self.on_fatal, self.say, self.clock = controls, on_quit, on_fatal, say, clock
        self._lock = threading.Lock()
        self._starting: threading.Thread | None = None
        self._since_s = 0.

    def handle(self, key: str) -> None:
        """Nie eine Ausnahme nach außen: Der Tastatur-Thread muss weiterlaufen."""
        try:
            if key == "r":
                self._start()
            elif key in ("s", "q"):
                self.controls.stop()
                if key == "q":
                    self.on_quit()
        except ValueError as exc:
            self.say("Bedienung: " + str(exc))
        except OSError as exc:
            self.on_fatal(str(exc))

    def _start(self) -> None:
        with self._lock:
            if (self._starting is not None and self._starting.is_alive()
                    and self.clock() - self._since_s <= LOAD_TIMEOUT_S):
                self.say("Bedienung: Start läuft bereits, die Aufgabe wird geladen.")
                return
            # Ein länger hängender Ladevorgang wird nicht abgewartet. Sein Ergebnis verwirft DriveControl.
            self._since_s = self.clock()
            self._starting = threading.Thread(target=self._run, name="terminalstart", daemon=True)
            self._starting.start()

    def _run(self) -> None:
        try:
            self.controls.start(browser=False)
        except ValueError as exc:
            self.say("Bedienung: " + str(exc))
        except OSError as exc:
            self.on_fatal(str(exc))
        except BaseException as exc:  # noqa: BLE001 - Thread darf nie still enden
            self.say("Bedienung: Start fehlgeschlagen: " + (str(exc) or type(exc).__name__))

    def wait(self, timeout_s: float = 2.0) -> None:
        """Nur für Tests: laufenden Start abwarten."""
        thread = self._starting
        if thread is not None:
            thread.join(timeout_s)
