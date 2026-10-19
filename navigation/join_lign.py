
import os
import sys
import numpy as np

dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)

from navigation.nav_to_coords_gps import latlong2local

def sawtooth(alpha):
    return (alpha + np.pi) % (2 * np.pi) - np.pi

def join_lign(ddboat):

    lat_A =
    lon_A =
    lat_B =
    lon_B =

    lat_origin =
    lon_origin =

    xA, yA = latlong2local(lat_A, lon_A, lat_origin, lon_origin)
    xB, yB = latlong2local(lat_B, lon_B, lat_origin, lon_origin)

    # ligne attractive
    dx = xB - xA
    dy = yB - yA
    norm = np.hypot(dx, dy)
    n = np.array([-dy / norm, dx / norm])

    #Cap du bateau
    phi = np.arctan2(dy, dx)

    #A CORRIGER
    det_2d = dx * (ddboat.y - yA) - dy * (ddboat.x - xA)
    e = det_2d / norm

    theta_bar = phi - np.arctan(e)
    u = sawtooth(theta_bar - theta)

    current_x = ddboat.x
    current_y = ddboat.y

    ddboat.control_motor(left_speed, right_speed) 
####################################################################


#quand ligne attractive atteinte

# point A attractif