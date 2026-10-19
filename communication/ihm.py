import numpy as np
import matplotlib.pyplot as plt

import os
import sys

# Ajoute la racine du projet au chemin de recherche
dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)
"""
from controller.ddboat import ddboat
from drivers.gps_driver_v2 import GpsIO
from drivers.imu9_driver_v2 import Imu9IO

ard = ArduinoIO()
gps = GpsIO()
imu = Imu9IO()

boat = ddboat(ard, imu, gps)"""

def save_trajectory(positions, filename="trajectory.txt"):
    if not (positions):
        print("Aucune position à enregistrer")
        return

    data = np.array(positions, dtype=float)

    np.savetxt(filename, data, delimiter=',', header='latitude,longitude', comments="", fmt="%.8f")


def nmea_to_degrees(raw_coord, direction):
    degrees = int(raw_coord/100)
    minutes = raw_coord - degrees*100

    decimal_degrees = degrees + minutes/60.0

    if direction in ("S","W"):
        decimal_degrees = - decimal_degrees

    return decimal_degrees

def display_trajectory(filename = "trajectory.txt"):
    #lecture du fichier texte
    data = np.loadtxt(filename, delimiter=",")

    latitude = data[:,0]
    longitude = data[:,1]

    #création du graphique
    fig, ax = plt.subplots()
    ax.plot(longitude, latitude, 'b.-', label="Trajectoire")
    ax.plot(longitude[0], latitude[0], 'go', label="Départ")
    ax.plot(longitude[-1], latitude[-1], 'ro', label="Arrivée")

    ax.set_title("Trajectoire DDBoat")
    ax.set_xlabel("Longitude (degrés)")
    ax.set_ylabel("Latitude (degrés)")
    ax.grid(True)
    ax.legend()
    ax.margins(0.1)

    plt.show()

if __name__ == "__main__":
    display_trajectory("D:/Differential Drive Boat/communication/trajectory.txt")