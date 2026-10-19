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


def latlong2local(lat, lon, lat_origin, lon_origin):
    """Convertit latitude/longitude en coordonnées locales (mètres)."""
    x = R_EARTH * math.cos(lat_origin) * (lon - lon_origin)
    y = R_EARTH * (lat - lat_origin)

    return x, y