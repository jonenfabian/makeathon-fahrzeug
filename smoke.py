"""Installierten Team-Einstieg ohne Geräte oder Browser prüfen."""
import time
from pathlib import Path

from followme.sim_binary import bundled_binary
from followme.sim_lab import Lab

ROOT = Path(__file__).resolve().parent
assert bundled_binary(ROOT / "fahrzeug/firmware/host_sim") is not None
lab = Lab()
try:
    deadline = time.monotonic() + 10
    while not lab.snapshot()["perception"]["target"] and time.monotonic() < deadline:
        time.sleep(.1)
    assert lab.snapshot()["perception"]["target"], lab.snapshot()
    lease = lab.start(lab.revision)["lease"]
    deadline = time.monotonic() + 4
    while lab.pose.x_m < .03 and time.monotonic() < deadline:
        lab.heartbeat(lease)
        time.sleep(.1)
    assert lab.pose.x_m >= .03, lab.snapshot()
    lab.stop()
    assert lab.link.status.state == "STANDBY"
    print("Fahrzeug: Produktions-AprilTag, Fahrt und bestätigter STOP bestanden.")
finally:
    lab.close()
