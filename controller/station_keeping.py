import sys
import time
import os
import numpy as np

# Ajoute la racine du projet au chemin de recherche
dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)

import drivers.arduino_driver_v2 as arddrv
from navigation.navigation_utils import (
    sawtooth,
    nmea_to_radians,
    latlong2local
)
from navigation.nav_to_coords_gps import go_to_waypoint_gps

def in_circle(waypoint_xy, r_max, position_xy):
    distance_centre = np.sqrt((waypoint_xy[0] - position_xy[0])**2 + (waypoint_xy[1] - position_xy[1])**2)
    return distance_centre <= r_max

def station(boat, gps, imu, telemetry, waypoint, r_max):
    """
    Station keeping :
    - Rejoint le waypoint initial.
    - Surveille la distance au waypoint.
    - Si le bateau sort du cercle, rejoint le waypoint.

    waypoint : (latitude_rad, longitude_rad)
    r_max : rayon autorisé en mètres
    """

    waypoint_lat, waypoint_lon = waypoint

    # Origine du repère local, identique à celle de update_position()
    lat_origin = np.deg2rad(48.198960)
    lon_origin = np.deg2rad(-3.015091)

    # Coordonnées locales du waypoint
    waypoint_x, waypoint_y = latlong2local(waypoint_lat, waypoint_lon, lat_origin, lon_origin)

    waypoint_xy = (waypoint_x, waypoint_y)

    try:
        # Rejoindre le waypoint initial
        reached = go_to_waypoint_gps(boat, gps, imu, telemetry, waypoint_lat, waypoint_lon)
        print(reached)
        if not reached:
            print("Navigation interrompue. Station keeping annulé.")
            return False

        print("Waypoint atteint. Station keeping activé.")

        # Surveillance continue
        while True:
            boat.update_position(lat_origin, lon_origin)
            boat.update_gps()

            # Vérifier que la position GPS est disponible
            if boat.lat_rad is None or boat.lon_rad is None:
                boat.stop_motor()
                print("Position GPS indisponible.")
                time.sleep(0.5)
                continue

            position_xy = (boat.x, boat.y)

            # 3. Vérifier si le bateau est dans le cercle
            if not in_circle(waypoint_xy, r_max, position_xy):
                print("Cercle dépassé : retour au waypoint.")

                reached = go_to_waypoint_gps(boat,gps,imu, telemetry, waypoint_lat, waypoint_lon)

                if not reached:
                    print("Retour interrompu.")
                    return False

                print("Waypoint rejoint. Reprise de la surveillance.")

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nStation keeping interrompu.")

        return False

    finally:
        boat.stop_motor()

if __name__ == "__main__" :
    waypoint = (np.deg2rad(48.198863), np.deg2rad(-3.015070))
    #point A