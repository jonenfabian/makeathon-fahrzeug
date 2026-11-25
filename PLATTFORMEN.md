# Geprüfter Start am 25.11.2026

Alle fünf Kombinationen haben in
[GitHub Actions](https://github.com/jonenfabian/makeathon-fahrzeug/actions/runs/45571830987)
bestanden. Die dabei gebauten Simulatoren sind mit Quell-/Binärprüfsummen im Repository enthalten
(Quellprüfsumme `2c2dbfe4…cf997f6`, Firmware 0.2.0).

| System im Test | Architektur | Python | Fahrzeug |
|---|---|---|---|
| Windows Server 2022 | x64 | 3.11 | Marker, Fahrt, STOPP bestanden |
| macOS 15 | Intel x64 | 3.11 | bestanden |
| macOS 15 | Apple Silicon ARM64 | 3.11 | bestanden |
| Ubuntu 24.04 | x64 | 3.11 | bestanden |
| Ubuntu 24.04 | ARM64 | 3.11 | bestanden |

Die Prüfung baut den Firmware-Host, installiert `requirements.txt` und führt `smoke.py` aus
(Produktions-AprilTag erkannt, Fahrt, bestätigter STOP). Ein Test mit echter Hardware ist davon
unabhängig. Andere Betriebssystemversionen und Architekturen werden durch diese Tabelle nicht
automatisch als geprüft ausgewiesen.

Bei Änderungen an Firmwarequellen die Builds erneut ausführen und passende Simulatoren
übernehmen. Das Python-Programm lehnt Simulatoren mit abweichendem Quellstand ab.
