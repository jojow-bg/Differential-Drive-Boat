"""
Pilotage manuel du DDBoat au clavier (en SSH).

Procédure de secours : couper main.py (Ctrl+C), puis lancer
    python3 manual_control.py
depuis le dossier du projet (le même que main.py).

Touches (clavier AZERTY) :
    Z / flèche haut     avant
    S / flèche bas      arrière
    Q / flèche gauche   virage à gauche en avançant
    D / flèche droite   virage à droite en avançant
    A                   rotation sur place à gauche
    E                   rotation sur place à droite
    Espace              arrêt
    + / -               vitesse
    P ou Ctrl+C         quitter (moteurs coupés)

Sécurité : un ordre n'est maintenu que tant que la touche est enfoncée
(répétition clavier). Sans touche pendant HOLD_TIMEOUT, ou si la
connexion SSH tombe, les moteurs sont coupés.
"""

import os
import select
import signal
import sys
import termios
import time
import tty

sys.path.append(os.path.abspath(os.path.dirname(__file__)))

from drivers.arduino_driver_v2 import ArduinoIO


# --------------------------------------------------
# CONFIGURATION
# --------------------------------------------------

SPEED_START = 60
SPEED_STEP = 10
SPEED_MIN = 20
SPEED_MAX = 120          # à adapter à la limite de ton driver Arduino
HOLD_TIMEOUT = 0.7       # s : arrêt si plus aucune touche reçue
LOOP_PERIOD = 0.1        # s : période d'envoi des commandes

# mouvement -> (coefficient gauche, coefficient droit)
# Mêmes conventions que nav_to_coords_gps : (0, pwm) tourne à gauche.
MOVES = {
    "stop":     (0, 0),
    "avant":    (1, 1),
    "arriere":  (-1, -1),
    "gauche":   (0, 1),
    "droite":   (1, 0),
    "pivot_g":  (-1, 1),
    "pivot_d":  (1, -1),
}

KEYMAP = {
    "z": "avant", "ESC[A": "avant",
    "s": "arriere", "ESC[B": "arriere",
    "q": "gauche", "ESC[D": "gauche",
    "d": "droite", "ESC[C": "droite",
    "a": "pivot_g",
    "e": "pivot_d",
    " ": "stop",
}

HELP = (
    "Pilotage manuel DDBoat\n"
    "  Z/↑ avant   S/↓ arrière   Q/← gauche   D/→ droite\n"
    "  A pivot gauche   E pivot droit   Espace stop\n"
    "  +/- vitesse      P ou Ctrl+C quitter\n"
)


# --------------------------------------------------
# OUTILS
# --------------------------------------------------

def parse_keys(data):
    """Découpe les octets reçus en touches ('z', ' ', 'ESC[A'...)."""
    keys = []
    i = 0
    while i < len(data):
        c = data[i]
        if c == "\x1b":
            if data[i + 1:i + 2] == "[" and i + 2 < len(data):
                keys.append("ESC[" + data[i + 2])
                i += 3
            else:
                i += 1                      # Échap seul : ignoré
        else:
            keys.append(c.lower())
            i += 1
    return keys


def _terminate(signum, frame):
    """SSH coupé (SIGHUP) ou kill : on passe par le finally."""
    raise SystemExit(0)


# --------------------------------------------------
# PROGRAMME
# --------------------------------------------------

def main():
    if not sys.stdin.isatty():
        print("Ce programme doit être lancé dans un terminal interactif.")
        sys.exit(1)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    ard = ArduinoIO()

    signal.signal(signal.SIGHUP, _terminate)
    signal.signal(signal.SIGTERM, _terminate)

    speed = SPEED_START
    move = "stop"
    last_key = 0.0

    try:
        tty.setcbreak(fd)       # lecture touche par touche, Ctrl+C actif
        print(HELP)

        while True:
            ready, _, _ = select.select([fd], [], [], LOOP_PERIOD)

            if ready:
                data = os.read(fd, 64).decode("utf-8", "ignore")
                if data == "":                      # terminal fermé
                    break

                for key in parse_keys(data):
                    if key == "p":
                        return
                    if key in KEYMAP:
                        move = KEYMAP[key]
                        last_key = time.monotonic()
                    elif key in ("+", "="):
                        speed = min(SPEED_MAX, speed + SPEED_STEP)
                    elif key in ("-", "6"):
                        speed = max(SPEED_MIN, speed - SPEED_STEP)

            # Plus de touche => arrêt
            if move != "stop" and time.monotonic() - last_key > HOLD_TIMEOUT:
                move = "stop"

            coef_l, coef_r = MOVES[move]
            left, right = int(speed * coef_l), int(speed * coef_r)
            ard.send_arduino_cmd_motor(left, right)

            sys.stdout.write(
                "\r\033[K{:<8}  gauche {:>4}  droit {:>4}  vitesse {:>3}".format(
                    move, left, right, speed))
            sys.stdout.flush()

    except KeyboardInterrupt:
        pass

    finally:
        try:
            ard.send_arduino_cmd_motor(0, 0)
        except Exception:
            pass
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\nMoteurs arrêtés.")


if __name__ == "__main__":
    main()
