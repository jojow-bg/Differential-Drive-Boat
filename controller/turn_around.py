import math
import time
import numpy as np

import os
import sys

# Ajoute la racine du projet au chemin de recherche
dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)

from ddboat import ddboat
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.gps_driver_v2 import GpsIO
from drivers.imu9_driver_v2 import Imu9IO

ard = ArduinoIO()
gps = GpsIO()
imu = Imu9IO()

boat = ddboat(ard, imu, gps)

def read_heading(imu):
    """Lit le cap calibré du bateau, en radians."""
    raw_mag = np.array(imu.read_mag_raw()).reshape(3, 1)
    return ddboat.get_calibrated_heading(raw_mag)


def angle_difference(theta, previous_theta):
    """Variation angulaire comprise entre -pi et pi."""
    return (theta - previous_theta + math.pi) % (2 * math.pi) - math.pi


def get_quadrant(theta):
    """Retourne le cadran trigonométrique, de 1 à 4."""
    angle = theta % (2 * math.pi)

    if angle < math.pi / 2:
        return 1
    elif angle < math.pi:
        return 2
    elif angle < 3 * math.pi / 2:
        return 3
    else:
        return 4


def turn_around(boat, imu, speed=60, timeout=60.0):
    """Effectue et vérifie une rotation complète du DDBoat."""

    target_angle = 2 * math.pi

    theta_previous = read_heading(imu)
    angle_turned = 0.0
    rotation_sign = None

    quadrants_visited = {get_quadrant(theta_previous)}
    start_time = time.monotonic()

    print("Début de la rotation.")

    try:
        # Rotation sur place : sens à vérifier sur le bateau
        boat.control_motor(speed, -speed)

        while time.monotonic() - start_time < timeout:
            # Le magnétomètre fonctionne à environ 10 Hz
            time.sleep(0.1)

            theta = read_heading(imu)

            # Variation réelle du cap, corrigée au passage de +/- pi
            delta = angle_difference(theta, theta_previous)
            theta_previous = theta

            # Détermine le sens de rotation observé
            if rotation_sign is None:
                if abs(delta) > math.radians(0.2):
                    rotation_sign = 1 if delta > 0 else -1

            # Cumule la rotation dans le sens initial observé
            if rotation_sign is not None:
                angle_turned += rotation_sign * delta

            # Enregistre les cadrans traversés
            quadrants_visited.add(get_quadrant(theta))

            print("\rRotation : {:.1f}° / 360° | Cadrans : {}".format(math.degrees(angle_turned), sorted(quadrants_visited)))

            # Un tour complet a été mesuré
            if angle_turned >= target_angle:
                print()

                if len(quadrants_visited) == 4:
                    print("Tour complet validé : 360° et 4 cadrans parcourus.")
                    return True

                print(
                    "360° mesurés, mais les 4 cadrans n'ont pas tous "
                    "été détectés. Vérifie la fréquence de lecture."
                )
                return False

        print()
        print("Échec : délai dépassé avant la validation du tour.")
        return False

    finally:
        # Arrêt même en cas de timeout ou d'exception
        boat.stop_motor()


def main():
    success = turn_around(boat, imu)

    if success:
        print("La rotation a été validée.")
    else:
        print("La rotation n'a pas été entièrement validée.")


if __name__ == "__main__":
    main()
