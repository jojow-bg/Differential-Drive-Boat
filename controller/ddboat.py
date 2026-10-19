import sys
import time
import os

# Ajoute le dossier parent (racine du projet) au chemin de recherche de Python
dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)

import drivers.arduino_driver_v2 as arddrv
from navigation.nav_to_coords_gps import nmea_to_radians
from navigation.nav_to_coords_gps import latlong2local

class ddboat:

    def __init__(self, ard, imu, gps):
        self.ard = ard
        self.imu = imu
        self.gps = gps
        self.x = 0
        self.y = 0
        self.lat_rad = 0
        self.lon_rad = 0

    def update_gps(self):
        msg, val = self.gps.read_gll_non_blocking()
        if msg and val[0]!=0:
            lat, dir_lat, lon, dir_lon = val
            lat_rad = nmea_to_radians(lat, dir_lat)
            lon_rad = nmea_to_radians(lon, dir_lon)
        return lat_rad, lon_rad

    def update_position(self, lat_origin, lon_origin):
        self.lat_rad, self.lon_rad = self.update_gps()
        self.x, self.y = latlong2local(self.lat_rad, self.lon_rad, lat_origin, lon_origin)

    def control_motor(self, left_speed, right_speed):
        # Vérifie que deux vitesses ont été fournies
        if len([left_speed, right_speed]) != 2:
            print("Utilisation : python3 prog.py <vitesse_gauche> <vitesse_droite>")
            print("Exemple      : python3 prog.py 20 20")
            sys.exit(1)

        try:
            left_speed = int(left_speed)
            right_speed = int(right_speed)
            
        except ValueError:
            print("Erreur : les vitesses doivent etre des nombres entiers.")
            print("Exemple : python3 prog.py 20 20")
            sys.exit(1)
        

        # Envoi des vitesses aux propulseurs
        self.ard.send_arduino_cmd_motor(left_speed, right_speed) 
        
        # Maintien de la commande pendant 2 secondes
        #time.sleep(2)

    def stop_motor(self):
        # Arret des propulseurs
        self.ard.send_arduino_cmd_motor(0, 0)
        print("Propulseurs arretes.")

    def back_to_base(self):
        pass #A faire avec les coordonnées GPS de la base
        # mettre un ralentissment dans les 5 dernières mètres