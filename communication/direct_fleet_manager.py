
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
        self.send_interval = 0.25

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

        print("[UDP direct] Bateau {} : écoute sur le port {}, pair={}:{}".format(self.boat_id, self.local_port, self.peer_ip, self.peer_port))

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

                if len(parts) not in (5, 7) or parts[0] != "POS":
                    continue

                boat_id = int(parts[1])
                latitude, longitude, cap = map(float, parts[2:5])

                state = parts[5] if len(parts) == 7 else None
                ready = bool(int(parts[6])) if len(parts) == 7 else False

                if boat_id == self.boat_id:
                    continue

                with self._lock:
                    self._other = {
                        "id_num": boat_id,
                        "lat": latitude,
                        "lon": longitude,
                        "cap": cap % 360.0,
                        "state" : state,
                        "ready" : ready,
                        "received_at": time.monotonic(),
                        "address": address[0],
                    }

            except (ValueError, UnicodeDecodeError):
                print("[UDP direct] Paquet reçu mais format invalide.")

    def update_own_position(self, latitude, longitude, cap, state="APPROACH", ready=False):
        """Envoie notre position au bateau pair, au plus une fois par seconde."""
        if latitude is None or longitude is None or cap is None:
            return

        if state is None:
            state = "APPROACH"

        now = time.monotonic()
        terminal_state = state in ("ABORT", "DONE")
        if now - self.last_send < self.send_interval and not terminal_state:
            return

        message = "POS|{}|{:.8f}|{:.8f}|{:.1f}|{}|{}".format(
                self.boat_id,
                float(latitude),
                float(longitude),
                float(cap) % 360.0,
                state,
                int(ready),
            )

        try:
            self._sock.sendto(
                message.encode("utf-8"),
                (self.peer_ip, self.peer_port),
            )
            self.last_send = now
        except OSError as exc:
            print("[UDP direct] Erreur d'envoi : {}".format(exc))

    def get_other_boats(self):
        """Retourne le dernier message reçu au format attendu par ddboat.py."""
        with self._lock:
            boat = dict(self._other) if self._other else None

        if boat is None:
            return []

        age = max(0.0, time.monotonic() - boat["received_at"])

        return [{
            "id" : "DDBoat{:02d}".format(boat["id_num"]),
            "latitude": boat["lat"],
            "longitude": boat["lon"],
            "heading_deg": boat["cap"],
            "state": boat["state"],
            "ready": boat["ready"],
            "age": age,
        }]

    def close(self):
        """Arrête le thread d'écoute et ferme le socket UDP."""
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass
