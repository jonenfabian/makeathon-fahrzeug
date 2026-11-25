"""Follow-me-Bausatz — Software für den Raspberry Pi.

Module:
  protocol    Zeilenprotokoll zum Pico (Prüfsumme, Meldungen)
  link        Verbindung zum Pico (seriell oder Host-Simulator)
  geometry    Kameramodell: Pixel → Richtung/Entfernung
  detect      Zielerkennung (ArUco, AprilTags der Fahrzeuge, Farbe, Person) und Zielauswahl
  controller  Folgeregler (Teamaufgabe: Regelgesetz)
  sim2d       2D-Simulator für den Folgeregler
  camera      Bildquelle (Pi-Kamera, Webcam, Video)
  stream      Live-Bild im Browser
  follow      Hauptprogramm
  teleop      Fernsteuerung per Tastatur
  leds        RGB-LEDs (WS2812B über SPI): zeigen, wer mit wem Daten austauscht
"""
