# Geprüfter Start

AUSSTEHEND: erster Lauf im Repository makeathon-fahrzeug. Nach grünem Lauf hier Datum und Link zum
Lauf eintragen. Bis dahin enthält das Repository keine geprüften vorgebauten Simulatoren.

| System im Test | Architektur | Python | Fahrzeug |
|---|---|---|---|
| Windows Server 2022 | x64 | 3.11 | ausstehend |
| macOS 15 | Intel x64 | 3.11 | ausstehend |
| macOS 15 | Apple Silicon ARM64 | 3.11 | ausstehend |
| Ubuntu 24.04 | x64 | 3.11 | ausstehend |
| Ubuntu 24.04 | ARM64 | 3.11 | ausstehend |

Die Prüfung baut den Firmware-Host, installiert `requirements.txt` und führt `smoke.py` aus
(Produktions-AprilTag erkannt, Fahrt, bestätigter STOP). Ein Test mit echter Hardware ist davon
unabhängig. Andere Betriebssystemversionen und Architekturen werden durch diese Tabelle nicht
automatisch als geprüft ausgewiesen.

Bei Änderungen an Firmwarequellen die Builds erneut ausführen und passende Simulatoren
übernehmen. Das Python-Programm lehnt Simulatoren mit abweichendem Quellstand ab.
