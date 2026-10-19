import sys, time
import numpy as np
import os

# Ajoute le dossier parent (racine du projet) au chemin de recherche de Python
dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)

def generate_data_with_ddboat(tmax, dt):
   
    from drivers.imu9_driver_v2 import Imu9IO
    imu = Imu9IO()

    def generate_cloud_with_ddboat(tmax, dt):
        n = int(tmax / dt)
        data = np.zeros((n, 3))
        t0 = time.time()
        for i in range(n):
            x = np.array([imu.read_mag_raw()]).T
            data[i, :] = [x[0, 0], x[1, 0], x[2, 0]]
            time.sleep(dt)

        np.savetxt("magcloud.csv", data, fmt='%.4f', delimiter=',')

    generate_cloud_with_ddboat(tmax, dt)


tmax, dt = 60, 1 / 20
generate_data_with_ddboat(tmax, dt)