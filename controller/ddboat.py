import sys
import time
import os

# Ajoute le dossier parent (racine du projet) au chemin de recherche de Python
dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)

import drivers.arduino_driver_v2 as arddrv

def control_motor(left_speed, right_speed, ard):
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
    ard.send_arduino_cmd_motor(left_speed, right_speed) 
    
    # Maintien de la commande pendant 2 secondes
    #time.sleep(2)

def stop_motor(ard):
    # Arret des propulseurs
    ard.send_arduino_cmd_motor(0, 0)
    print("Propulseurs arretes.")


if __name__ == "__main__":
    control_motor()