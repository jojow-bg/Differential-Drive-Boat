import time
import math
import numpy as np
import sys
import os

# Ajoute le dossier parent (racine du projet) au chemin de recherche de Python
dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)

from drivers.arduino_driver_v2 import ArduinoIO
from drivers.gps_driver_v2 import GpsIO
from drivers.imu9_driver_v2 import Imu9IO
import ddboat as ddboat
from calibration.magcalib import load_calibration_json
from drivers.ddboatlib import sawtooth


# --- FONCTIONS UTILITAIRES ---

def nmea_to_decimal(value, direction):
    """ Convertit les coordonnées GPS brutes NMEA (DDMM.MMMM) en degrés décimaux. """
    if value == 0.0:
        return 0.0
    degrees = int(value / 100)
    minutes = value - (degrees * 100)
    decimal = degrees + (minutes / 60.0)
    if direction in ['S', 'W']: # Sud ou Ouest sont négatifs
        decimal *= -1
    return decimal

def get_target(lat_curr, lon_curr, lat_target, lon_target):
    """ Calcule la distance (mètres) et le cap cible (radians, 0=Nord) """
    R_EARTH = 6371000.0
    lat_curr_rad = math.radians(lat_curr)
    
    # Projection locale plane
    dx = R_EARTH * math.radians(lon_target - lon_curr) * math.cos(lat_curr_rad)
    dy = R_EARTH * math.radians(lat_target - lat_curr)
    
    distance = math.sqrt(dx**2 + dy**2)
    # dx est sur l'axe Est/Ouest, dy sur l'axe Nord/Sud
    target_heading = math.atan2(dx, dy) 
    return distance, target_heading

def get_current_heading(imu, invA, xbar):
    """ Lit le magnétomètre et calcule le cap en radians """
    m_raw = np.array(imu.read_mag_raw()).reshape(3, 1)
    
    # Application de la calibration de magcalib.py
    m_calib = invA @ (m_raw - xbar)
    
    # Dans votre magcalib : X pointe au Nord, Y pointe à l'Ouest.
    # L'Est est donc -Y. Le cap (0=Nord, horloger) est atan2(Est, Nord)
    heading = math.atan2(-m_calib[1, 0], m_calib[0, 0])
    return heading


# --- BOUCLE PRINCIPALE ---

def main():
    print("Initialisation des composants...")
    ard = ArduinoIO()       # Moteurs
    gps = GpsIO()           # GPS
    imu = Imu9IO()          # Magnétomètre
    
    # Chargement des paramètres de calibration calculés par magcalib.py
    try:
        invA, xbar = load_calibration_json("../calibration/mag_calib_params.json")
        print("Calibration chargée avec succès.")
    except Exception as e:
        print("Erreur : Impossible de charger mag_calib_params.json. Avez-vous exécuté magcalib.py ?")
        sys.exit(1)
        
    # --- CONFIGURATION DU WAYPOINT ---
    # Remplacez par vos coordonnées Google Maps (en degrés décimaux)
    lat_target = 48,41829
    lon_target = -4.47290
    
    base_speed = 100  # Vitesse d'avancée de base (sur 255)
    Kp = 50          # Gain proportionnel pour corriger la trajectoire
    
    print("Démarrage de la navigation vers Lat: {}, Lon: {}".format(lat_target,lon_target))
    
    try:
        while True:
            # 1. Lecture asynchrone du GPS
            gps_ok, gps_data = gps.read_gll_non_blocking()
            
            if gps_ok and gps_data[0] != 0.0:
                lat_raw, ns, lon_raw, ew, _ = gps_data
                
                # Conversion NMEA -> Décimal
                lat_curr = nmea_to_decimal(lat_raw, ns)
                lon_curr = nmea_to_decimal(lon_raw, ew)
                
                # 2. Calcul de la distance et du cap cible
                dist, psi_target = get_target(lat_curr, lon_curr, lat_target, lon_target)
                
                # Condition d'arrêt : Si on est à moins de 3 mètres du point
                if dist < 3.0:
                    print("!!! POINT DE PASSAGE ATTEINT !!!")
                    ddboat.stop_motor(ard)
                    break
                    
                # 3. Lecture du cap actuel avec l'IMU calibrée
                psi_curr = get_current_heading(imu, invA, xbar)
                
                # 4. Calcul de l'erreur d'angle (entre -pi et pi grace à ddboatlib)
                error = sawtooth(psi_target - psi_curr)
                
                # 5. Asservissement des moteurs
                # Si erreur > 0 (cible à droite), cmd_turn positif -> moteur gauche accélère
                cmd_turn = Kp * error
                
                u_left = int(base_speed + cmd_turn)
                u_right = int(base_speed - cmd_turn)
                
                # Bornage des vitesses entre 0 et 255 pour éviter d'inverser les hélices ou planter l'Arduino
                u_left = max(0, min(255, u_left))
                u_right = max(0, min(255, u_right))
                
                # Envoi des commandes
                ddboat.control_motor(u_left, u_right, ard)
                
                # Affichage de débogage
                #print("Dist: {}m | Cap cible: {}° | Cap actuel: {}° | Err: {}° | Moteurs: ({}, {})".format(dist, math.degrees(psi_target).0f, math.degrees(psi_curr).0f, math.degrees(error).0f, u_left, u_right))
                
            time.sleep(0.1) # Boucle à 10 Hz
            
    except KeyboardInterrupt:
        print("\nArrêt manuel par l'utilisateur.")
        ddboat.stop_motor(ard)

if __name__ == "__main__":
    main()