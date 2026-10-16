import sys
import time
import os
import drivers.arduino_driver_v2 as arddrv
import controller.ddboat as ddboat


def main(): 
    # Creation de l'objet Arduino
    ard = arddrv.ArduinoIO()

    ddboat.control_motor(0, 100, ard)

    time.sleep(2)

    ddboat.stop_motor(ard)

if __name__ == "__main__":
    main()