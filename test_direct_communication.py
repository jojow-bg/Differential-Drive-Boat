
"""Test UDP direct sans GPS, IMU, Arduino ni moteurs."""

import argparse
import time

from communication.direct_fleet_manager import DirectFleetManager


parser = argparse.ArgumentParser(
    description="Test de communication UDP directe entre deux bateaux"
)
parser.add_argument("--id", type=int, required=True,
                    help="Identifiant unique du bateau")
parser.add_argument("--peer-ip", required=True,
                    help="Adresse IP de l'autre bateau")
parser.add_argument("--local-port", type=int, default=6801)
parser.add_argument("--peer-port", type=int, default=6801)
parser.add_argument("--lat", type=float, required=True,
                    help="Latitude fictive en degrés")
parser.add_argument("--lon", type=float, required=True,
                    help="Longitude fictive en degrés")
parser.add_argument("--cap", type=float, default=0.0,
                    help="Cap en degrés : 0=N, 90=E, 180=S, 270=W")

args = parser.parse_args()

fleet = DirectFleetManager(
    boat_id=args.id,
    peer_ip=args.peer_ip,
    peer_port=args.peer_port,
    local_port=args.local_port,
)

try:
    print("Test en cours. Aucun moteur n'est commandé.")
    print("Appuie sur Ctrl+C pour arrêter.")

    while True:
        # Envoie une position fictive à l'autre bateau.
        fleet.update_own_position(
            latitude=args.lat,
            longitude=args.lon,
            cap=args.cap,
        )

        # Affiche les positions reçues.
        others = fleet.get_other_boats()

        if not others:
            print("Aucun paquet reçu pour le moment.")
        else:
            for boat in others:
                print(
                    "Reçu {} : lat={}, "
                    "lon={}, cap={}°, "
                    "âge={}s".format(boat["id"], boat["latitude"], boat["longitude"], boat["heading_deg"], boat["age"])
                )

        time.sleep(1.0)

except KeyboardInterrupt:
    print("\nTest arrêté.")

finally:
    fleet.close()
