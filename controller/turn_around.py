import serial
import numpy as np 
import os
import sys

# Ajoute le dossier parent (racine du projet) au chemin de recherche de Python
dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)

from ddboat import ddboat
from drivers.imu9_driver_v2 import Imu9IO

"""
Je veux que le ddboat fasse un tour sur lui même en passant par les 4 cadrans du cercle trigo
On utilise le cap, on le divise en cos et sin et ensuite on utilise les cadrans
"""

imu = Imu9IO()

def turn_around():
    #On extrait le magnétisme et calcul le cap du bateau grâce à la calibration
    raw_mag = np.array(imu.read_mag_raw()).reshape(3, 1)
    theta = ddboat.get_calibrated_heading(raw_mag)
    x, y = np.cos(theta), np.sin(theta)
    while (x < 0) & (y < 0):
        ddboat.control_motor(100,-100)

    if (x>0) & (y>0):
        while True :
            if (x < 0) & (y > 0):
                while True :
                    if (x < 0) & (y < 0):
                        while True :
                            if (x > 0) & (y < 0):
                                while True :
                                    if (x > 0) & (y < 0):
                                        while True :
                                            if (x > 0) & (y > 0):
                                                return True
                                ddboat.control_motor(100,-100)            
                            ddboat.control_motor(100,-100)        
                    ddboat.control_motor(100,-100)
            ddboat.control_motor(100,-100)        