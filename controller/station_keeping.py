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

def in_circle(waypoint, r_max, position):
    distance_centre = np.sqrt((waypoint[0] - position[0])**2 + (waypoint[1] - position[1])**2)
    if distance_centre < r_max :
        return True
    return False

def station(boat, waypoint, r_max):
    
    pass

if __name__ == "__main__" :
    pass