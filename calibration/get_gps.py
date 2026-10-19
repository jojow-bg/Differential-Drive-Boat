from controller.ddboat import ddboat
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.gps_driver_v2 import GpsIO
from drivers.imu9_driver_v2 import Imu9IO
from navigation.nav_to_coords_gps import nmea_to_radians

ard = ArduinoIO()
gps = GpsIO()
imu = Imu9IO()
ddboat = ddboat(ard, imu, gps)

val = [0]
while val[0]==0: 
    msg, val = ddboat.gps.read_gll_non_blocking()

lat, dir_lat, lon, dir_lon = val
lat_rad = nmea_to_radians(lat, dir_lat)
lon_rad = nmea_to_radians(lon, dir_lon)

print("position actuelle :")
print("lat_rad =", lat_rad)
print("lon_rad =", lon_rad)