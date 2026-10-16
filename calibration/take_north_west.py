import sys, time
import numpy as np

from drivers.imu9_driver_v2 import Imu9IO
imu = Imu9IO()

n = 10

# --- NORTH ---
seconds = 120
while seconds > 0:
    print(f"{seconds} restantes avant north", end="\r")
    time.sleep(1)
    seconds -= 1

data_north = np.zeros((n, 3))
for i in range(n):
    data_north[i, :] = imu.read_mag_raw()
    time.sleep(0.5)

# Sauvegarde en CSV (séparateur virgule)
np.savetxt("x_north.csv", data_north, delimiter=',')


# --- WEST ---
seconds = 60
while seconds > 0:
    print(f"{seconds} restantes avant west", end="\r")
    time.sleep(1)
    seconds -= 1

data_west = np.zeros((n, 3))
for i in range(n):
    data_west[i, :] = imu.read_mag_raw()
    time.sleep(0.5)

# Sauvegarde en CSV (séparateur virgule)
np.savetxt("x_west.csv", data_west, delimiter=',')