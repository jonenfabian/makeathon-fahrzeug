"""Geführtes Inbetriebnahme-Protokoll; sendet niemals ARM, CMD oder STOP.

Alle aktiven Prüfungen führt die prüfende Person nach docs/09 selbst durch.
Ein Simulatorlauf kann keine physische Prüfung bestehen oder freigeben.
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import subprocess
import tempfile
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from .config import load_config
from .link import open_serial, open_sim
from .protocol import supported_protocol

CHECKS = (
    ("elektrik", "Elektrische Erstprüfung",
     "Steckplan, Versorgung, Polarität, Crimps und elektrische Erstprüfung nach docs/05 abgeschlossen?"),
    ("aufgebockt", "Räder frei",
     "Fahrzeug aufgebockt, Räder ohne Bodenkontakt, Not-Aus erreichbar, andere Pi-Programme beendet?"),
    ("referenz", "Software und Firmware",
     ("Sauberer Git-Stand, Hosttests und Pi-Tests grün, Pico-2-UF2/SHA256 geprüft? Commit und SHA256 notieren. "
      "Nach Firmwareänderung beide UF2 neu bauen und erneut prüfen.")),
    ("encoder", "Motoren und Encoder",
     ("Motor-/Encoder-Richtung, Auflösung und Kalibrierung nach docs/04 geprüft? "
      "Encoderunterbrechung stoppt, Wiederanschluss startet nicht ohne ARM? Messwerte notieren.")),
    ("notaus", "Not-Aus und ARM-Pflicht",
     ("Pflicht-Sicherheitsabnahme docs/07: beide Not-Aus-Kanäle geprüft, Motoren stehen, "
      "Entriegeln oder CMD allein startet nicht, ARM bei gedrücktem Not-Aus abgelehnt?")),
    ("watchdog", "Programm-/Verbindungsausfall",
     ("Teleop-Prozessabbruch/USB-Unterbrechung stoppt durch Pico-Watchdog; "
      "nach Rückkehr kein Wiederanlauf ohne ARM? Gemessene Zeit notieren.")),
    ("abstand", "Abstandssensor",
     ("Hand vor ToF und Sensorausfall sperren Vorwärtsfahrt gemäß docs/07? "
      "Messwerte und Ergebnis des Pflichttests notieren.")),
    ("relais", "Relais-Abfallzeit",
     ("Abfallzeit nach Not-Aus gemessen und RELAY_FAULT_MS mindestens dreimal so groß? "
      "Beide Zeiten in ms und ggf. neuen Firmwarestand notieren. Kein Softwaretest ersetzt die Messung.")),
    ("lasttest", "Wiederholungen unter Last",
     ("100 Not-Aus-Auslösungen unter sinnvoller Motorlast ohne Fehlfunktion; "
      "Einschalt-/Entriegelungstests nach docs/08 ohne BMS-/Pi-Ausfälle? Protokollpfad notieren.")),
    ("kamera", "Kamera und Erkennung",
     ("Sensorzeit, Livebild, Markerabstände und FPS/Latenz gemessen, Benchmark und Tag-Reichweite abgelegt? "
      "Bei Coral zusätzlich echte USB-Inferenz und Dauerprüfung aus CORAL.md. Dateien notieren.")),
    ("leds", "LED-Anzeige",
     "Beide LED-Streifen zeigen stabile richtige Farben? SPI/Takt und Flackern dokumentieren."),
    ("fahrt", "Freigegebener Bodenversuch",
     ("Erst nach bestandenen Sicherheitsprüfungen: Spurweite über zehn Drehungen kalibriert, "
      "Teleop geradeaus/STOP geprüft, erste Folgefahrt auf freier Fläche mit Mindestabstand? Werte notieren.")),
    ("masterimage", "Abschluss durch die Orga",
     ("Alle Punkte aus docs/09 einschließlich SD-Konfiguration, Änderungen, Messdateien und "
      "offenen Befunden geprüft? Entscheidung zu Testmontage/Klonen separat dort unterschreiben.")),
)


def new_report(values, inspector, mode):
    root = Path(__file__).resolve().parents[2]
    commit = None
    dirty = None
    try:
        result = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"],
                                check=False, capture_output=True, text=True, timeout=3)
        commit = result.stdout.strip() if result.returncode == 0 else None
        if commit:
            status = subprocess.run(["git", "-C", str(root), "status", "--porcelain"],
                                    check=False, capture_output=True, text=True, timeout=3)
            dirty = bool(status.stdout) if status.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        pass  # Das ausgepackte Team-Paket braucht kein Git.
    return {"format": 1, "vehicle": values["fahrzeug"], "inspector": inspector,
            "created_utc": datetime.now(timezone.utc).isoformat(), "git_commit": commit, "git_dirty": dirty,
            "configuration": dict(values),
            "platform": platform.platform(), "mode": mode, "ready_for_drive": False,
            "notice": "Nur Prüfprotokoll, keine automatische Freigabe. Physische Abnahme: docs/09.",
            "probe": {"status": "pending"}, "workflow": "pending",
            "checks": [{"id": key, "title": title, "instruction": instruction,
                        "status": "pending", "evidence": ""} for key, title, instruction in CHECKS]}


def checkpoint(path, report):
    """Nach jeder Antwort atomar ersetzen; Ziel wurde vorher exklusiv angelegt."""
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                     prefix=f".{path.name}.", delete=False) as file:
        temp = Path(file.name)
        try:
            json.dump(report, file, ensure_ascii=False, indent=2, allow_nan=False)
            file.write("\n")
            file.flush()
            os.fsync(file.fileno())
        except BaseException:
            temp.unlink(missing_ok=True)
            raise
    try:
        os.replace(temp, path)
    finally:
        temp.unlink(missing_ok=True)


def passive_probe(values, simulated):
    """Verbindung nur kurz lesen, vor manuellen Tests wieder schließen."""
    with (open_sim() if simulated else open_serial(values["port"])) as link:
        status = link.wait_status(timeout_s=3)
        deadline_s = time.monotonic() + 3
        while link.info is None and time.monotonic() < deadline_s:
            time.sleep(.02)
        info = link.info
        latest = link.status
        ok = (latest is not None and latest.state == "STANDBY" and latest.flags == 0 and link.status_age_s < .3
              and info is not None and supported_protocol(info.proto_version))
        return {"status": "ok" if ok else "error", "simulated": simulated,
                "initial_status": asdict(status), "status_data": asdict(latest) if latest else None,
                "info": asdict(info) if info else None,
                "detail": "Nur STAT/INFO gelesen. Erwartet: frisches STANDBY und Protokoll 1 oder 2."}


def ask_check(check, simulated, ask=None):
    ask = ask or input
    print(f"\n{check['title']}: {check['instruction']}")
    while True:
        answer = ask("Ergebnis [j=bestanden, n=Fehler, o=offen, q=beenden]: ").strip().lower()
        if answer in ("j", "n", "o", "q"):
            break
        print("Bitte j, n, o oder q eingeben.")
    if answer == "q":
        return False
    evidence = ask("Messwerte/Dateipfad/Beobachtung (bei j/n erforderlich): ").strip()
    while answer in ("j", "n") and not evidence:
        evidence = ask("Bitte einen Nachweis oder eine Beobachtung eintragen: ").strip()
    check.update(status={"j": "simulated" if simulated else "ok", "n": "error", "o": "pending"}[answer],
                 evidence=evidence, recorded_utc=datetime.now(timezone.utc).isoformat())
    return True


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--konfiguration", type=Path)
    ap.add_argument("--pruefer")
    ap.add_argument("--ausgabe", type=Path, required=True, help="Neues JSON-Protokoll; wird nie überschrieben")
    ap.add_argument("--interaktiv", action="store_true", help="Manuelle Prüfungen anleiten und protokollieren")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--sim", action="store_true", help="STAT/INFO nur am Firmware-Host-Simulator lesen")
    mode.add_argument("--hardware", action="store_true", help="Nach zwei Bestätigungen nur Pico-Status lesen")
    args = ap.parse_args(argv)
    if args.hardware and not args.interaktiv:
        ap.error("--hardware erfordert --interaktiv und bestätigte elektrische Erstprüfung/aufgebockte Räder")
    if args.interaktiv and (not args.pruefer or not args.pruefer.strip()):
        ap.error("Interaktiver Lauf erfordert --pruefer NAME")
    values = load_config(args.konfiguration)
    report = new_report(values, args.pruefer or "nicht angegeben",
                        "sim" if args.sim else "hardware" if args.hardware else "offline")
    # Vor jeder Verbindung sicherstellen, dass keine fremde Datei verloren geht.
    with args.ausgabe.open("x", encoding="utf-8") as file:
        json.dump(report, file, ensure_ascii=False, indent=2)
    try:
        for index, check in enumerate(report["checks"]):
            if args.interaktiv:
                if not ask_check(check, args.sim):
                    report["workflow"] = "aborted"
                    break
                checkpoint(args.ausgabe, report)
                if check["status"] in ("pending", "error"):
                    # Keine späteren Motorprüfungen über offene/rote Gates hinweg.
                    break
            if index == 1 and (args.sim or args.hardware):
                report["probe"] = passive_probe(values, args.sim)
                checkpoint(args.ausgabe, report)
                if report["probe"]["status"] != "ok":
                    break
        if report["workflow"] != "aborted":
            checks = report["checks"]
            if report["probe"]["status"] == "error" or any(c["status"] == "error" for c in checks):
                report["workflow"] = "failed"
            elif all(c["status"] == "simulated" for c in checks):
                report["workflow"] = "simulated"
            elif all(c["status"] == "ok" for c in checks):
                report["workflow"] = "documented"  # Selbstbericht, ausdrücklich KEINE Freigabe
    except (EOFError, KeyboardInterrupt):
        report["workflow"] = "aborted"
    except (OSError, RuntimeError, ValueError, TimeoutError, subprocess.SubprocessError) as exc:
        report.update(workflow="failed", error=str(exc))
    finally:
        checkpoint(args.ausgabe, report)
    print(f"Protokoll: {args.ausgabe} · Stand: {report['workflow']} · Keine automatische Fahrfreigabe.")
    return 1 if report["workflow"] in ("failed", "aborted") else 0


if __name__ == "__main__":
    raise SystemExit(main())
