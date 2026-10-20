import math
import numpy as np

R_EARTH = 6371000.0


def sawtooth(alpha):
    """Ramène un angle dans [-pi, pi[."""
    return (alpha + np.pi) % (2 * np.pi) - np.pi


def nmea_to_radians(raw_coord, direction):
    """Convertit une coordonnée NMEA DDMM.MMMM en radians."""
    degrees = int(raw_coord / 100)
    minutes = raw_coord - degrees * 100

    decimal_degrees = degrees + minutes / 60.0

    if direction in ("S", "W"):
        decimal_degrees = -decimal_degrees

    return math.radians(decimal_degrees)


def latlong2local(lat, lon, lat_origin = np.deg2rad(48.198960), lon_origin = np.deg2rad(-3.015091)):
    """Convertit latitude/longitude en coordonnées locales (mètres)."""
    x = -R_EARTH * math.cos(lat_origin) * (lon - lon_origin)
    y = R_EARTH * (lat - lat_origin)

    return x, y

def angle_difference(theta, previous_theta):
    """Variation angulaire comprise entre -pi et pi."""
    return (theta - previous_theta + math.pi) % (2 * math.pi) - math.pi


def get_quadrant(theta):
    """Retourne le cadran trigonométrique, de 1 à 4."""
    angle = theta % (2 * math.pi)

    if angle < math.pi / 2:
        return 1
    elif angle < math.pi:
        return 2
    elif angle < 3 * math.pi / 2:
        return 3
    else:
        return 4


def get_bearing_xy(x1, y1, x2, y2):
    """Cap désiré en radians, depuis le nord, sens horaire (comme get_bearing).
    Repère local : x vers l'ouest, y vers le nord."""
    d_east = -(x2 - x1)  # x vers l'ouest, donc l'est est -x
    d_north = y2 - y1
    return math.atan2(d_east, d_north)
