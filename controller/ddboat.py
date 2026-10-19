import math
import sys
import time
import os
import numpy as np
import math

# Ajoute le dossier parent (racine du projet) au chemin de recherche de Python
dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)

import drivers.arduino_driver_v2 as arddrv
from navigation.navigation_utils import (
    sawtooth,
    nmea_to_radians,
    latlong2local
)
from calibration.calibration_utils import load_calibration_json
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.gps_driver_v2 import GpsIO
from drivers.imu9_driver_v2 import Imu9IO


class ddboat:

    def __init__(self, ard, imu, gps):
        self.ard = ard
        self.imu = imu
        self.gps = gps
        self.x = None
        self.y = None
        self.lat_rad = None
        self.lon_rad = None
        self.heading = None
        self.INV_A, self.X_BAR = load_calibration_json("../calibration/mag_calib_params.json")

    def update_gps(self):
        msg, val = self.gps.read_gll_non_blocking()
        if msg and val[0]!=0:
            lat, dir_lat, lon, dir_lon = val[:4]
            lat_rad = nmea_to_radians(lat, dir_lat)
            lon_rad = nmea_to_radians(lon, dir_lon)
            self.lat_rad = lat_rad
            self.lon_rad = lon_rad

    def get_calibrated_heading(self, raw_mag):
        """Retourne le cap thêta du bateau à partir de la mesure brute."""
        y = self.INV_A @ (raw_mag - self.X_BAR)
        return -math.atan2(y[1, 0], y[0, 0]) 

    def update_position(self, lat_origin= np.deg2rad(48.198863), lon_origin = np.deg2rad(-3.015070)):
        self.update_gps()
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
    

    def compute_commands_TOR(self, error_theta,tol_deg = 5, PWM_MOTOR=120):
        if error_theta > np.deg2rad(tol_deg):
            return 0,PWM_MOTOR
        elif error_theta < np.deg2rad(-tol_deg):
            return PWM_MOTOR,0
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

        lat_A = np.deg2rad(48.198863)
        lon_A = np.deg2rad(-3.015070)
        lat_B =np.deg2rad(48.199072)
        lon_B =np.deg2rad(-3.016258)

        lat_origin = lat_A
        lon_origin =lon_A

        xA, yA = latlong2local(lat_A, lon_A, lat_origin, lon_origin)
        xB, yB = latlong2local(lat_B, lon_B, lat_origin, lon_origin)
        A = np.array([[xA],[yA]])
        dx = xB - xA
        dy = yB - yA
        norm = np.hypot(dx, dy)
        n = np.array([[-dy / norm], [dx / norm]])

        while self.lat_rad is None:
            time.sleep(0.2)
            self.update_gps()
            print("echec de la connexion au GPS")

        self.update_position()
        current_pos =np.array([[self.x], [self.y]])

        while np.linalg.norm(current_pos-A) > 3: 
            
            self.update_heading()
            self.update_position(lat_origin, lon_origin)

            current_pos =np.array([[self.x], [self.y]])

            #Attraction par ligne et par point A
            line_attraction_vector = -2*n@n.T@(current_pos-A)
            point_attraction_vector = -2*(current_pos-A)
            #Calcul du vecteur du champ
            target_vector = line_attraction_vector + point_attraction_vector
            #Calcul de son cap
            target_heading = np.arctan2(-target_vector[0][0], target_vector[1][0])

            self.follow_heading(target_heading, Kp=1.0)

            time.sleep(0.1)

        return True
