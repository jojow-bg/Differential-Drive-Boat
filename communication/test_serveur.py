
import time
from ddboats_client import send_position, get_boats

SERVER_IP = "172.20.26.111"  # À remplacer par l'IP du serveur
SERVER_PORT = 6800
BOAT_ID = 2

if __name__ == "__main__":
    print(f"Connexion UDP vers {SERVER_IP}:{SERVER_PORT}")

    send_position(
        SERVER_IP,
        SERVER_PORT,
        BOAT_ID,
        lat=48.199028,
        lon=-3.015556,
        cap=90
    )

    try:
        while True:
            boats = get_boats()

            if BOAT_ID in boats:
                print("Mon bateau :", boats[BOAT_ID])
            else:
                print("Mon bateau absent de la réponse ALL.")

            for boat_id, data in boats.items():
                if boat_id != BOAT_ID:
                    print(f"Autre bateau {boat_id} :", data)

            time.sleep(1)

    except KeyboardInterrupt:
        print("Test arrêté.")
