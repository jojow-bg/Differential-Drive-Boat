
"""Communication UDP directe entre deux bateaux, sans serveur central."""

import socket
import threading
import time


class DirectFleetManager:
    def __init__(self, boat_id, peer_ip, peer_port=6801, local_port=6801):
        self.boat_id = int(boat_id)
        self.peer_ip = peer_ip
        self.peer_port = int(peer_port)
        self.local_port = int(local_port)

        self.last_send = 0.0
        self.send_interval = 1.0

        self._lock = threading.Lock()
        self._other = None
        self._running = True

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", self.local_port))
        self._sock.settimeout(1.0)

        self._thread = threading.Thread(
            target=self._listen,
            daemon=True
        )
        self._thread.start()

        print(
            f"[UDP direct] Bateau {self.boat_id} : "
            f"écoute sur le port {self.local_port}, "
            f"pair={self.peer_ip}:{self.peer_port}"
        )

    def _listen(self):
        """Écoute les positions envoyées par l'autre bateau."""
        while self._running:
            try:
                data, address = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                parts = data.decode("utf-8").strip().split("|")

                if len(parts) != 5 or parts[0] != "POS":
                    continue

                boat_id = int(parts[1])
                latitude, longitude, cap = map(float, parts[2:5])

                if boat_id == self.boat_id:
                    continue

                with self._lock:
                    self._other = {
                        "id_num": boat_id,
                        "lat": latitude,
                        "lon": longitude,
                        "cap": cap % 360.0,
                        "received_at": time.monotonic(),
                        "address": address[0],
                    }

            except (ValueError, UnicodeDecodeError):
                print("[UDP direct] Paquet reçu mais format invalide.")

    def update_own_position(self, latitude, longitude, cap):
        """Envoie notre position au bateau pair, au plus une fois par seconde."""
        if latitude is None or longitude is None or cap is None:
            return

        now = time.monotonic()
        if now - self.last_send < self.send_interval:
            return

        message = "POS|{}|{:.8f}|{:.8f}|{:.1f}".format(
            self.boat_id,
            float(latitude),
            float(longitude),
            float(cap) % 360.0,
        )

        try:
            self._sock.sendto(
                message.encode("utf-8"),
                (self.peer_ip, self.peer_port),
            )
            self.last_send = now
        except OSError as exc:
            print(f"[UDP direct] Erreur d'envoi : {exc}")

    def get_other_boats(self):
        """Retourne le dernier message reçu au format attendu par ddboat.py."""
        with self._lock:
            boat = dict(self._other) if self._other else None

        if boat is None:
            return []

        age = max(0.0, time.monotonic() - boat["received_at"])

        return [{
            "id": f"DDBoat{boat['id_num']:02d}",
            "latitude": boat["lat"],
            "longitude": boat["lon"],
            "heading_deg": boat["cap"],
            "age": age,
        }]

    def close(self):
        """Arrête le thread d'écoute et ferme le socket UDP."""
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass
