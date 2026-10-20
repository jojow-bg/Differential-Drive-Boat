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
    #point pour dépasser et back to base : 48.196500, -3.016900 (lat, long)

    #Instanciation des objets et de la classe ddboat
    ard = ArduinoIO()
    gps = GpsIO()
    imu = Imu9IO()
    boat = ddboat(ard, imu, gps)

    telemetry = TelemetryServer()
    target_lat = np.deg2rad(48.196500)
    target_lon = np.deg2rad(-3.016900)
    while True : 
        go_to_waypoint_gps(boat, gps, imu, telemetry, target_lat, target_lon, tol=5)


if __name__ == "__main__":
    main()