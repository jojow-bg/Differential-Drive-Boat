import sys
import time
import os

from controller.ddboat import ddboat
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.gps_driver_v2 import GpsIO
from drivers.imu9_driver_v2 import Imu9IO


def main(): 

    #Instanciation des objets et de la classe ddboat
    ard = ArduinoIO()
    gps = GpsIO()
    imu = Imu9IO()
    boat = ddboat(ard, imu, gps)

    
    IN_ZONE = boat.check_zone()
    if not IN_ZONE:
        boat.back_to_base()
        boat.stop_motor()


if __name__ == "__main__":
    main()