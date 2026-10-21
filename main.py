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
from controller.station_keeping import station

def main():
    target_lat = np.deg2rad(48.198944)
    target_lon = np.deg2rad(-3.015478)
    
    ard = ArduinoIO()
    gps = GpsIO()
    imu = Imu9IO()
    telemetry = TelemetryServer()
    boat = ddboat(ard, imu, gps, telemetry=telemetry)

    polygon = [
    (np.deg2rad(48.198960), np.deg2rad(-3.015091)),  # A
    (np.deg2rad(48.198944), np.deg2rad(-3.015478)),  # G
    (np.deg2rad(48.198124), np.deg2rad(-3.015355)),  # C
    (np.deg2rad(48.198259), np.deg2rad(-3.017114)),  # D
    (np.deg2rad(48.199027), np.deg2rad(-3.015887)),  # G'
    (np.deg2rad(48.199628), np.deg2rad(-3.016379)),  # E
    (np.deg2rad(48.199688), np.deg2rad(-3.015158)),  # F
    (np.deg2rad(48.198944), np.deg2rad(-3.015478)),  # G
]

    try:
        while (boat.lat_rad is None or boat.lon_rad is None):
            boat.update_gps()
            time.sleep(1)

        print("Connexion GPS réussi")
        #go_to_waypoint_gps(boat, gps, imu, telemetry, target_lat, target_lon, tol=5, abort_if_outside_zone=True)
        """    
        result = go_to_waypoint_gps(boat, gps, imu, telemetry, target_lat, target_lon, tol=5, abort_if_outside_zone=True)

        if result is False:
            print("Navigation interrompue : retour à la base.")
        elif result is True:
            print("Waypoint atteint : retour à la base.")
        else:
            print("Résultat de navigation inattendu.")+
        """
        #on test le station keeping
        target_lat_station = np.deg2rad(48.199027)
        target_lon_station = np.deg2rad(-3.015887)
        
        station(boat, gps, imu, telemetry, (target_lat_station, target_lon_station), r_max=5)

        #boat.visit_polygon(polygon)
        
    finally:
        boat.turn_around()
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