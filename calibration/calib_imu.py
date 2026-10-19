import os
import sys

# Ajoute le dossier parent (racine du projet) au chemin de recherche de Python
dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)


from drivers.imu9_driver_v2 import Imu9IO
import time
import numpy as np

imu = Imu9IO()


def mesure(n, pause=0.05):
    """Renvoie n mesures du magnétomètre sous forme d'un tableau (n,3)"""
    pts = []
    for i in range(n):
        pts.append(imu.read_mag_raw())
        time.sleep(pause)
    return np.array(pts, dtype=float)

# 1) Nuage de points : tourner le bateau dans tous les sens
print("ETAPE 1 : tourne le bateau dans TOUS les sens (60 s). Debut dans 4 s...")
time.sleep(4)
cloud = mesure(1200)
np.savetxt("magcloud.csv", cloud, delimiter=",")   # 3 colonnes, séparées par des virgules
print("magcloud.csv OK")

# 2) Nord
raw_input = input if str(type(input)).find("builtin") < 0 else input
print("ETAPE 2 : bateau A PLAT, avant vers le NORD. Appuie sur Entree puis ne bouge plus.")
input()
xnorth = mesure(100).mean(axis=0)

# 3) Ouest
print("ETAPE 3 : bateau A PLAT, avant vers l'OUEST. Appuie sur Entree puis ne bouge plus.")
input()
xwest = mesure(100).mean(axis=0)

# 6 valeurs sur une ligne (séparées par des espaces) : nord puis ouest
np.savetxt("magNorthWest.csv", np.concatenate([xnorth, xwest]).reshape(1, 6))
print("magNorthWest.csv OK")