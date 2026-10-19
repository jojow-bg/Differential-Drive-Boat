
import json
import socket


class TelemetryServer:
    """Envoie la télémétrie du DDBoat à un client TCP."""

    def __init__(self, host="0.0.0.0", port=5001):
        self.server = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        self.server.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        self.server.bind((host, port))
        self.server.listen(1)
        self.server.setblocking(False)

        self.client = None

        print("Serveur TCP en écoute sur le port {}".format(port))

    def send(self, data):
        """Envoie un dictionnaire Python converti en JSON."""

        # Accepte une nouvelle connexion si nécessaire
        if self.client is None:
            try:
                self.client, address = self.server.accept()
                self.client.settimeout(0.2)
                print("Client connecté : {}".format(address))

            except BlockingIOError:
                return

        try:
            message = json.dumps(data) + "\n"
            self.client.sendall(message.encode("utf-8"))

        except (OSError, TimeoutError):
            print("Client déconnecté.")

            try:
                self.client.close()
            except OSError:
                pass

            self.client = None

    def close(self):
        """Ferme proprement les sockets."""

        if self.client is not None:
            try:
                self.client.close()
            except OSError:
                pass

            self.client = None

        self.server.close()
        print("Serveur TCP arrêté.")
