
import math
import time
import csv
from pathlib import Path
import os
import sys

import numpy as np

# Ajoute la racine du projet au chemin de recherche
dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)

from controller.ddboat import ddboat
from navigation.navigation_utils import (
    sawtooth,
    nmea_to_radians,
    latlong2local
)

local_coordinates = latlong2local

from drivers.arduino_driver_v2 import ArduinoIO
from drivers.gps_driver_v2 import GpsIO
from drivers.imu9_driver_v2 import Imu9IO
from communication.telemetry import TelemetryServer


# --------------------------------------------------
# CONFIGURATION
# --------------------------------------------------

R_EARTH = 6371000.0
PWM_MOTOR = 160
GPS_TIMEOUT = 0.01
LOOP_PERIOD = 0.1
WAYPOINT_TOLERANCE = 2.0

# Waypoint existant dans ton programme, en radians
TARGET_LAT = np.deg2rad(48.198863)
TARGET_LON = np.deg2rad(-3.016258)

# Fichier de trajectoire, dans le dossier du script
TRAJECTORY_FILE = "communication/trajectory.txt"


# --------------------------------------------------
# CALCULS DE NAVIGATION
# --------------------------------------------------


def sawtooth(alpha):
    """Ramène un angle dans [-pi, pi[."""
    return (alpha + np.pi) % (2 * np.pi) - np.pi


def get_bearing(lat1_rad, lon1_rad, lat2_rad, lon2_rad):
    """Calcule le cap désiré en radians, depuis le nord."""

    dlon = lon2_rad - lon1_rad

    y = math.sin(dlon) * math.cos(lat2_rad)

    x = (
        math.cos(lat1_rad) * math.sin(lat2_rad)
        - math.sin(lat1_rad) * math.cos(lat2_rad)
        * math.cos(dlon)
    )

    return math.atan2(y, x)

def get_distance_with_gps(lat1_rad, lon1_rad, lat2_rad, lon2_rad):
    """Distance entre deux positions GPS, en mètres."""

    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1_rad) * math.cos(lat2_rad)
        * math.sin(dlon / 2) ** 2
    )

    # Évite les erreurs numériques dues aux arrondis
    a = max(0.0, min(1.0, a))

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R_EARTH * c


def latlong2cart(lx,ly,ρ=R_EARTH): 
    return ρ*np.array([[np.cos(ly)*np.cos(lx)],[np.cos(ly)*np.sin(lx)],[np.sin(ly)]])  


def nmea_to_radians(raw_coord, direction):
    """
    Convertit DDMM.MMMM en radians.
    Exemple : 4807.038 N -> latitude en radians.
    """

    degrees = int(raw_coord / 100)
    minutes = raw_coord - degrees * 100

    decimal_degrees = degrees + minutes / 60.0

    if direction in ("S", "W"):
        decimal_degrees = -decimal_degrees

    return math.radians(decimal_degrees)

# --------------------------------------------------
# ENREGISTREMENT DE LA TRAJECTOIRE
# --------------------------------------------------

def save_trajectory(positions, filename=TRAJECTORY_FILE):
    """
    Enregistre :
    x (m), y (m), latitude (degrés), longitude (degrés).
    """

    if not positions:
        print("Aucune position GPS à enregistrer.")
        return

    with open(filename, "w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)

        writer.writerow([
            "x_m",
            "y_m",
            "latitude_deg",
            "longitude_deg"
        ])

        writer.writerows(positions)

    print("Trajectoire enregistrée dans : {}".format(filename))
    print("Nombre de positions : {}".format(len(positions)))


# --------------------------------------------------
# NAVIGATION + TÉLÉMÉTRIE
# --------------------------------------------------

def go_to_waypoint_gps(
    boat,
    gps,
    imu,
    telemetry,
    target_lat,
    target_lon,
    tol=WAYPOINT_TOLERANCE,
    abort_if_outside_zone=False
):
    """
    Navigue vers un waypoint et transmet les données à l'IHM.

    La trajectoire est enregistrée à chaque nouvelle position
    GPS valide, puis sauvegardée à la fin du programme.
    """

    origin_lat = np.deg2rad(48.198863)
    origin_lon = np.deg2rad(-3.015070)

    # Lignes : x, y, latitude en degrés, longitude en degrés
    positions = []

    reached = False

    try:
        print("Navigation démarrée.")

        while True:
            loop_start = time.monotonic()
            new_gps = False
            # ------------------------------------------
            # 1. Lecture GPS
            # ------------------------------------------
            boat.update_gps()
            x, y = local_coordinates(
                boat.lat_rad,
                boat.lon_rad)

            positions.append((
                x,
                y,
                math.degrees(boat.lat_rad),
                math.degrees(boat.lon_rad)
            ))

            if abort_if_outside_zone and new_gps:

                if not boat.check_zone():
                    print("\nZone dépassée ! Retour à la base.")
                    boat.stop_motor()
                    return False

            # Lecture du cap IMU, même sans GPS
            raw_mag = np.array(
                imu.read_mag_raw()
            ).reshape(3, 1)

            theta = boat.get_calibrated_heading(raw_mag)

            # ------------------------------------------
            # 2. Pas encore de position GPS
            # ------------------------------------------

            if boat.lat_rad is None or boat.lon_rad is None:
                boat.stop_motor()
                print(math.degrees(theta))
                telemetry.send({
                    "latitude": None,
                    "longitude": None,
                    "x": None,
                    "y": None,
                    "heading_deg": math.degrees(theta),
                    "motor_left": 0,
                    "motor_right": 0,
                    "distance_m": None,
                    "new_gps": False,
                    "target_reached": False
                })

                time.sleep(LOOP_PERIOD)
                continue

            # ------------------------------------------
            # 3. Lecture du cap
            # ------------------------------------------

            raw_mag = np.array(
                imu.read_mag_raw()
            ).reshape(3, 1)

            theta = boat.get_calibrated_heading(raw_mag)

            # ------------------------------------------
            # 4. Distance et consignes moteurs
            # ------------------------------------------

            distance = get_distance_with_gps(
                boat.lat_rad,
                boat.lon_rad,
                target_lat,
                target_lon
            )

            reached = distance < tol

            if reached:
                cmd_left, cmd_right = 0, 0

            else:
                desired_heading = get_bearing(
                    boat.lat_rad,
                    boat.lon_rad,
                    target_lat,
                    target_lon
                )

                error_theta = sawtooth(
                    desired_heading - theta
                )

                cmd_left, cmd_right = boat.compute_commands_TOR(
                    error_theta,
                    PWM_MOTOR
                )

            # ------------------------------------------
            # 5. Commande réelle des moteurs
            # ------------------------------------------

            boat.control_motor(cmd_left, cmd_right)

            # ------------------------------------------
            # 6. Envoi des données à l'IHM
            # ------------------------------------------

            x, y = local_coordinates(
                boat.lat_rad,
                boat.lon_rad)
            
            telemetry.send({
                "latitude": math.degrees(boat.lat_rad),
                "longitude": math.degrees(boat.lon_rad),
                "x": x,
                "y": y,
                "heading_deg": math.degrees(theta),
                "motor_left": cmd_left,
                "motor_right": cmd_right,
                "distance_m": distance,
                "new_gps": new_gps,
                "target_reached": reached
            })

            print(
                "\rDistance : {:.2f} m | Cap : {:.1f} deg | Moteurs : {}, {}".format(distance,math.degrees(theta), cmd_left, cmd_right)
            )

            # ------------------------------------------
            # 7. Fin de navigation
            # ------------------------------------------

            if reached:
                print("\nWaypoint atteint.")
                return True

            elapsed = time.monotonic() - loop_start

            if elapsed < LOOP_PERIOD:
                time.sleep(LOOP_PERIOD - elapsed)

    except KeyboardInterrupt:
        print("\nNavigation interrompue par l'utilisateur.")
        return False

    finally:
        boat.stop_motor()
        save_trajectory(positions)


# --------------------------------------------------
# POINT D'ENTRÉE
# --------------------------------------------------

if __name__ == "__main__":
    ard = None
    gps = None
    telemetry = None

    try:
        # Initialisation du matériel
        ard = ArduinoIO()
        gps = GpsIO()
        imu = Imu9IO()

        boat = ddboat(ard, imu, gps)

        # Le serveur attend le client sans bloquer la navigation
        telemetry = TelemetryServer(
            host="0.0.0.0",
            port=5002
        )

        go_to_waypoint_gps(
            boat=boat,
            gps=gps,
            imu=imu,
            telemetry=telemetry,
            target_lat=TARGET_LAT,
            target_lon=TARGET_LON
        )

    except KeyboardInterrupt:
        print("\nArrêt demandé.")

        if ard is not None:
            try:
                ard.send_arduino_cmd_motor(0, 0)
            except Exception:
                pass

        if telemetry is not None:
            try:
                telemetry.close()
            except Exception:
                pass

        if gps is not None:
            try:
                gps.close()
            except Exception:
                pass
