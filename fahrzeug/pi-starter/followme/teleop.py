"""Fernsteuerung per Tastatur (über SSH nutzbar) — Stufe 1 der Aufgabe.

    python -m followme.teleop            # echter Pico an /dev/ttyACM0
    python -m followme.teleop --sim      # Host-Simulator

Tasten:  Pfeile oder W/A/S/D fahren · Leertaste Stopp · R scharf schalten
         X Pico-STOP · Q Ende
Totmann-Prinzip: Wird 0,3 s lang keine Fahrtaste gedrückt (Taste
losgelassen), gehen die Sollwerte auf 0. Hält man die Taste, wiederholt das
Terminal sie automatisch.
"""

from __future__ import annotations

import argparse
import curses
import math
import time

from .link import McuLink, open_serial, open_sim

V_STEP_MPS = 0.4
W_STEP_RAD_S = 1.2
DEADMAN_S = 0.3


def validate_steps(v_step_mps: float, w_step_rad_s: float) -> None:
    if not math.isfinite(v_step_mps) or not 0 < v_step_mps <= V_STEP_MPS:
        raise ValueError(f"Geschwindigkeit muss > 0 und <= {V_STEP_MPS} m/s sein")
    if not math.isfinite(w_step_rad_s) or not 0 < w_step_rad_s <= W_STEP_RAD_S:
        raise ValueError(f"Drehrate muss > 0 und <= {W_STEP_RAD_S} rad/s sein")


def run(stdscr: curses.window, link: McuLink,
        v_step_mps: float = V_STEP_MPS, w_step_rad_s: float = W_STEP_RAD_S) -> None:
    validate_steps(v_step_mps, w_step_rad_s)
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(50)  # getch wartet höchstens 50 ms ⇒ 20 Hz Sendetakt
    v = w = 0.0
    last_key = 0.0
    while True:
        k = stdscr.getch()
        now = time.monotonic()
        if k in (ord("q"), ord("Q")):
            break
        if k in (ord("r"), ord("R")):
            v = w = 0.0
            link.arm()
        if k in (ord("x"), ord("X")):
            v = w = 0.0
            link.stop()
        moves = {
            curses.KEY_UP: (v_step_mps, 0.0), ord("w"): (v_step_mps, 0.0),
            curses.KEY_DOWN: (-v_step_mps, 0.0), ord("s"): (-v_step_mps, 0.0),
            curses.KEY_LEFT: (0.0, w_step_rad_s), ord("a"): (0.0, w_step_rad_s),
            curses.KEY_RIGHT: (0.0, -w_step_rad_s), ord("d"): (0.0, -w_step_rad_s),
        }
        if k in moves:
            v, w = moves[k]
            last_key = now
        if k == ord(" ") or now - last_key > DEADMAN_S:
            v = w = 0.0
        link.send_cmd(v, w)

        st = link.status
        stdscr.erase()
        stdscr.addstr(0, 0, "Follow-me Fernsteuerung  (R scharf, Pfeile fahren, Leertaste Stopp, Q Ende)")
        if st:
            stdscr.addstr(2, 0, f"Zustand: {st.state:8s}  Flags: {' '.join(st.flag_names()) or '-'}")
            stdscr.addstr(3, 0, f"Soll:    v={v:+.2f} m/s  w={w:+.2f} rad/s")
            stdscr.addstr(4, 0, f"Räder:   links {st.v_l_mm_s:+5d} mm/s  rechts {st.v_r_mm_s:+5d} mm/s")
            tof = "kein Ziel" if st.tof_mm == 0xFFFF else f"{st.tof_mm} mm"
            stdscr.addstr(5, 0, f"Abstand: {tof:10s}  Motorspannung: {st.vmot_mv / 1000:.2f} V")
            stdscr.addstr(6, 0, f"Encoder: {st.ticks_l:8d} {st.ticks_r:8d}   Empfangsfehler Pico: {st.rx_errors}")
        else:
            stdscr.addstr(2, 0, "Warte auf Pico ...")
        stdscr.refresh()


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description="Fernsteuerung per Tastatur")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--geschwindigkeit", type=float, default=V_STEP_MPS, help="Sollwert in m/s, maximal 0,4")
    ap.add_argument("--drehrate", type=float, default=W_STEP_RAD_S, help="Sollwert in rad/s, maximal 1,2")
    args = ap.parse_args(argv)
    try:
        validate_steps(args.geschwindigkeit, args.drehrate)
    except ValueError as exc:
        ap.error(str(exc))
    link = open_sim() if args.sim else open_serial(args.port)
    try:
        st = link.wait_status(timeout_s=3)
        if st.state == "RUN":
            raise RuntimeError("Pico ist bereits RUN. STOP wird versucht; Zustand prüfen und bewusst neu starten.")
        curses.wrapper(run, link, args.geschwindigkeit, args.drehrate)
    finally:
        try:
            link.stop()
        finally:
            link.close()


if __name__ == "__main__":
    main()
