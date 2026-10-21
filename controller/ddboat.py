import math
import sys
import time
import os
import numpy as np
import math

# Ajoute le dossier parent (racine du projet) au chemin de recherche de Python
dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)

from navigation.navigation_utils import (
    nmea_to_radians,
    latlong2local,
    angle_difference,
    get_quadrant
)

from navigation.navigation_utils import get_bearing_xy
from calibration.calibration_utils import load_calibration_json

racine_projet = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

chemin_calibration = os.path.join(racine_projet,"calibration","mag_calib_params.json")



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
        self.INV_A, self.X_BAR = load_calibration_json(chemin_calibration)
        self.zone_limit = [
            (np.deg2rad(48.198960), np.deg2rad(-3.015091)),
            (np.deg2rad(48.197408), np.deg2rad(-3.015060)),  
            (np.deg2rad(48.1970689), np.deg2rad(-3.017328)), 
            (np.deg2rad(48.199072), np.deg2rad(-3.016258)),  
            (np.deg2rad(48.199707), np.deg2rad(-3.016404)),  
            (np.deg2rad(48.199814), np.deg2rad(-3.015137)),  
        ]
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
        return math.atan2(y[1, 0], y[0, 0]) 

    def update_position(self, lat_origin= np.deg2rad(48.198960), lon_origin = np.deg2rad(-3.015091)):
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
    

    def compute_commands_TOR(self, error_theta, PWM_MOTOR=120,tol_deg = 5):
        if error_theta > np.deg2rad(tol_deg):
            return PWM_MOTOR,0
        elif error_theta < np.deg2rad(-tol_deg):
            return 0,PWM_MOTOR
        else:
            return PWM_MOTOR, PWM_MOTOR

    def compute_commands_diff(self, error_theta, PWM_MOTOR=120, Kp=80, PWM_MAX=191):
        """Commande différentielle proportionnelle.
        error_theta > 0 : bateau à gauche du cap cible, il faut tourner à droite."""
        v = PWM_MOTOR * 2 * max(0.0, math.cos(error_theta))

        # Correction de cap proportionnelle à l'erreur
        u = Kp * error_theta

        left = v + u
        right = v - u

        # Saturation haute en conservant l'écart entre les deux moteurs
        overflow = max(left, right) - PWM_MAX
        if overflow > 0:
            left -= overflow
            right -= overflow

        # Pas de marche arrière
        left = int(np.clip(left, 0, PWM_MAX))
        right = int(np.clip(right, 0, PWM_MAX))
        
        return left, right

    def follow_heading(self, target_heading, meth="TOR"):
        self.update_heading()
        diff = target_heading - self.heading
        e = math.atan2(math.sin(diff), math.cos(diff)) #si e>0, on est a gauche de l'objectif
        if meth=="DIFF":
            left_speed, right_speed = self.compute_commands_diff(e)
        else:
            left_speed, right_speed = self.compute_commands_TOR(e)

        self.control_motor(left_speed, right_speed)
        return [left_speed, right_speed]


    def stop_motor(self):
        # Arret des propulseurs
        self.ard.send_arduino_cmd_motor(0, 0)
        print("Propulseurs arretes.")

    
    def back_to_base(self):

        lat_A = np.deg2rad(48.198960)
        lon_A = np.deg2rad(-3.015091)
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
            print("distance au point A = ", np.linalg.norm(current_pos-A))

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

            self.follow_heading(target_heading)

            time.sleep(0.2)

        return True

    def check_zone(self):
        if (self.lat_rad is None or self.lon_rad is None):
            return None        
        inside = False
        n = len(self.zone_limit)

        j = n - 1
        for i in range(n):
            lat_i, lon_i = self.zone_limit[i]
            lat_j, lon_j = self.zone_limit[j]

            if ((lon_i > self.lon_rad) != (lon_j > self.lon_rad)) and (
                self.lat_rad < (lat_j - lat_i) * (self.lon_rad - lon_i) / (lon_j - lon_i) + lat_i
            ):
                inside = not inside
            j = i
        return inside

    def turn_around(self, speed=60, timeout=60.0):
        target_angle = 2 * math.pi
        self.update_heading()
        theta_previous = self.heading
        angle_turned = 0.0
        rotation_sign = None

        quadrants_visited = {get_quadrant(theta_previous)}
        start_time = time.monotonic()

        print("Début de la rotation.")

        try:
            # Rotation sur place : sens à vérifier sur le bateau
            self.control_motor(speed, -speed)

            while time.monotonic() - start_time < timeout:
                # Le magnétomètre fonctionne à environ 10 Hz
                time.sleep(0.1)
                self.update_heading()
                theta = self.heading

                # Variation réelle du cap, corrigée au passage de +/- pi
                delta = angle_difference(theta, theta_previous)
                theta_previous = theta

                # Détermine le sens de rotation observé
                if rotation_sign is None:
                    if np.abs(delta) > math.radians(0.2):
                        rotation_sign = -1 if delta > 0 else 1

                # Cumule la rotation dans le sens initial observé
                if rotation_sign is not None:
                    angle_turned -= rotation_sign * delta

                # Enregistre les cadrans traversés
                quadrants_visited.add(get_quadrant(theta))

                print("\rRotation : {:.1f}° / 360° | Cadrans : {}".format(math.degrees(angle_turned), sorted(quadrants_visited)))

                # Un tour complet a été mesuré
                if angle_turned >= target_angle:
                    if len(quadrants_visited) == 4:
                        print("Tour complet validé : 360° et 4 cadrans parcourus.")
                        return True

                    print(
                        "360° mesurés, mais les 4 cadrans n'ont pas tous "
                        "été détectés. Vérifie la fréquence de lecture."
                    )
                    return False

            print("Échec : délai dépassé avant la validation du tour.")
            return False

        finally:
            # Arrêt même en cas de timeout ou d'exception
            self.stop_motor() 

    def pass_half_plan(self, start_point,stop_point,cartesian = False):
        self.update_position()
        if not cartesian:
            x_start, y_start = latlong2local(start_point[0, 0], start_point[1, 0])
            x_stop, y_stop = latlong2local(stop_point[0, 0], stop_point[1, 0])
        if cartesian:
            x_start, y_start = start_point[0,0], start_point[1,0]
            x_stop, y_stop = stop_point[0,0], stop_point[1,0]

        if (x_stop-x_start)*(self.x-x_stop) + (y_stop-y_start) * (self.y-y_stop) > 0 :
            return True

    def visit_polygon(self, polygon, telemetry):
        
        pts = [np.array(latlong2local(lat, lon), dtype=float).reshape(2, 1) for lat, lon in polygon]
        pts = pts + [pts[0]]
        for k in range(len(pts) - 1):

            start_point, stop_point = pts[k], pts[k + 1]
            psi_segment = get_bearing_xy(start_point[0,0], start_point[1,0], stop_point[0,0], stop_point[1,0])
            vec_ab = (stop_point - start_point).flatten()
            norm_ab = np.linalg.norm(vec_ab)

            counter = 0
            while not self.pass_half_plan(start_point, stop_point, cartesian=True):
                vec_am = np.array([self.x - start_point[0, 0],self.y - start_point[1, 0]])
                det_2d = vec_ab[0] * vec_am[1] - vec_ab[1] * vec_am[0]

                # - pour avoir e > 0 quand le bateau est à gauche de AB
                e = -det_2d / norm_ab

                # Bateau à gauche : on augmente le cap pour revenir vers la droite
                psi_d = psi_segment + math.atan(e/8)
                cmd = self.follow_heading(psi_d, "DIFF")
                cmd_left, cmd_right = cmd[0], cmd[1]

                self.update_gps()
                self.update_position()
                self.update_heading()

                distance = np.hypot(self.x - stop_point[0,0], self.y - stop_point[1,0])
                reached = False 
                if counter % 5 ==0 :
                    telemetry.send({
                                    "latitude": math.degrees(self.lat_rad),
                                    "longitude": math.degrees(self.lon_rad),
                                    "x": self.x,
                                    "y": self.y,
                                    "heading_deg": math.degrees(self.heading),
                                    "motor_left": cmd_left,
                                    "motor_right": cmd_right,
                                    "distance_m": distance,
                                    "new_gps": True,
                                    "target_reached": reached
                                })
                time.sleep(0.1)
                counter+=1
            
        self.stop_motor()