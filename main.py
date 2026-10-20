import sys
import time
import os
import numpy as np

from controller.ddboat import ddboat
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.gps_driver_v2 import GpsIO
from drivers.imu9_driver_v2 import Imu9IO
from communication.telemetry import TelemetryServer
from navigation.nav_to_coords_gps import go_to_waypoint_gps

def main():
    target_lat = np.deg2rad(48.196500)
    target_lon = np.deg2rad(-3.016900)

    ard = ArduinoIO()
    gps = GpsIO()
    imu = Imu9IO()
    boat = ddboat(ard, imu, gps)
    telemetry = TelemetryServer()

    try:
        result = go_to_waypoint_gps(
            boat,
            gps,
            imu,
            telemetry,
            target_lat,
            target_lon,
            tol=5,
            abort_if_outside_zone=True
        )

        if result is False:
            print("Navigation interrompue : retour à la base.")
        elif result is True:
            print("Waypoint atteint : retour à la base.")
        else:
            print("Résultat de navigation inattendu.")

        # Une seule séquence de retour
        boat.turn_around()
        boat.back_to_base()

    finally:
        boat.back_to_base()
        boat.stop_motor()

        try:
            gps.close()
        except Exception:
            pass

        try:
            telemetry.close()
        except Exception:
            pass

        try:
            if ard.arduino is not None:
                ard.arduino.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()