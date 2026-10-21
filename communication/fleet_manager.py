
import time

from communication.ddboats_client import send_position, get_boats


class FleetManager:
    def __init__(self, boat_id, server_ip, server_port=6800):
        self.boat_id = int(boat_id)
        self.server_ip = server_ip
        self.server_port = server_port

        self.last_send = 0.0
        self.send_interval = 1.0

    def update_own_position(self, latitude, longitude, cap):
        """
        Envoie périodiquement notre position au serveur central.
        latitude, longitude et cap sont exprimés en degrés.
        """
        if latitude is None or longitude is None or cap is None:
            return

        now = time.monotonic()

        if now - self.last_send < self.send_interval:
            return

        try:
            send_position(
                self.server_ip,
                self.server_port,
                self.boat_id,
                latitude,
                longitude,
                cap % 360)
            self.last_send = now

        except OSError as exc:
            print("Erreur UDP flotte :", exc)

    def get_other_boats(self):
        """
        Retourne les positions des autres bateaux
        au format attendu par l'IHM.
        """
        other_boats = []

        for boat_id, boat in get_boats().items():
            if int(boat_id) == self.boat_id:
                continue

            other_boats.append({
                "id": "DDBoat{:02d}".format(int(boat_id)),
                "latitude": boat["lat"],
                "longitude": boat["lon"],
                "heading_deg": boat["cap"],
                "age": boat["age"]
            })

        return other_boats
