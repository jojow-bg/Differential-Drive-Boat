import math
import time
import numpy as np

import os
import sys

# Ajoute la racine du projet au chemin de recherche
dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)

from controller.ddboat import ddboat
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.gps_driver_v2 import GpsIO
from drivers.imu9_driver_v2 import Imu9IO

ard = ArduinoIO()
gps = GpsIO()
imu = Imu9IO()

boat = ddboat(ard, imu, gps)

