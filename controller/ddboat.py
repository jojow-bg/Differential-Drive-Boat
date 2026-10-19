import math
import sys
import time
import os
import numpy as np

from exemples.drones_exo.ddboatlib import sawtooth

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
        self.heading = 0

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

    def update_heading(self):
        raw_mag = np.array(self.imu.read_mag_raw()).reshape(3, 1)
        self.heading = self.get_calibrated_heading(raw_mag)

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

    def compute_commands_TOR(error_theta,tol_deg = 5, PWM_MOTOR=120):
        if error_theta > np.deg2rad(tol_deg):
            return 0, PWM_MOTOR
        elif error_theta < np.deg2rad(-tol_deg):
            return PWM_MOTOR, 0
        else:
            return PWM_MOTOR, PWM_MOTOR

    def follow_heading(self, target_heading, Kp=50.0):
        self.update_heading()
        diff = target_heading - self.heading
        e = math.atan2(math.sin(diff), math.cos(diff)) #si e>0, il on est a droite de l'objectif
        left_speed, right_speed = self.compute_commands_TOR(e)
        self.control_motor(left_speed, right_speed)


    def stop_motor(self):
        # Arret des propulseurs
        self.ard.send_arduino_cmd_motor(0, 0)
        print("Propulseurs arretes.")

    
    def back_to_base(self):
        current_pos =np.array([[self.x], [self.y]])

        lat_A = 
        lon_A =
        lat_B =
        lon_B =

        lat_origin =
        lon_origin =

        xA, yA = latlong2local(lat_A, lon_A, lat_origin, lon_origin)
        xB, yB = latlong2local(lat_B, lon_B, lat_origin, lon_origin)
        A = np.array([[xA],[yA]])
        dx = xB - xA
        dy = yB - yA
        norm = np.hypot(dx, dy)
        n = np.array([[-dy / norm], [dx / norm]])

        
        while np.norm(current_pos-A) > 1: 
            
            self.update_heading()
            self.update_position()
            current_pos =np.array([[self.x], [self.y]])

            #Attraction par ligne et par point A
            line_attraction_vector = -2*n@n.T@(current_pos-A)
            point_attraction_vector = -2*(current_pos-A)
            #Calcul du vecteur du champ
            target_vector = line_attraction_vector + point_attraction_vector
            #Calcul de son cap
            target_heading = np.atan2(target_vector[0][1], target_vector[0][0])

            self.follow_heading(target_heading, Kp=50.0)

        return True #Pour faire un if back_to_base ensuite