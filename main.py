import os
import numpy as np

from controller.ddboat import ddboat
from controller.cooperative_mission import run_cooperative_mission
from drivers.arduino_driver_v2 import ArduinoIO
from drivers.gps_driver_v2 import GpsIO
from drivers.imu9_driver_v2 import Imu9IO
from communication.telemetry import TelemetryServer


def main():
    # --------------------------------------------------
    # Configuration de la mission coopérative
    # --------------------------------------------------

    # A = orbiteur ; B = bateau central
    role = os.getenv("DDBOAT_ROLE", "A").upper()

    if role not in ("A", "B"):
        raise ValueError("DDBOAT_ROLE doit valoir A ou B.")

    # Identifiants distincts entre les deux bateaux
    os.environ["DDBOAT_COMM_MODE"] = "direct"
    os.environ["DDBOAT_ID"] = os.getenv(
        "DDBOAT_ID", "2" if role == "A" else "5"
    )
    os.environ["DDBOAT_PEER_IP"] = os.getenv(
        "DDBOAT_PEER_IP", "172.20.25.205"
    )
    os.environ["DDBOAT_LOCAL_PORT"] = "6801"
    os.environ["DDBOAT_PEER_PORT"] = "6801"

    # Coordonnées GPS en degrés décimaux
    base_a = (48.198858, -3.014986)
    base_b = (48.198903, -3.014986)
    waypoint_goal = (48.198944, -3.015478)

    # Après le demi-tour, A est passé de l'autre côté de B : chacun rentre
    # à la base de son nouveau côté, sinon les deux bateaux se croisent.
    return_base = base_b if role == "A" else base_a
    boat_id = int(os.environ["DDBOAT_ID"])

    # --------------------------------------------------
    # Initialisation du matériel existant
    # --------------------------------------------------

    ard = None
    gps = None
    imu = None
    telemetry = None
    boat = None

    try:
        ard = ArduinoIO()
        gps = GpsIO()
        imu = Imu9IO()
        telemetry = TelemetryServer()

        boat = ddboat(
            ard,
            imu,
            gps,
            telemetry=telemetry,
        )

        print("===================================")
        print("Mission de capture coopérative")
        print("Rôle :", role)
        print("Identifiant :", boat_id)
        print("Base de retour :", return_base)
        print("Cible :", waypoint_goal)
        print("===================================")

        # La boucle de mission prend en charge la navigation,
        # la synchronisation avec le partenaire et l'arrêt sûr.
        run_cooperative_mission(
            boat=boat,
            boat_id=boat_id,
            is_orbiting_boat=(role == "A"),
            base_position=return_base,
            waypoint_goal=waypoint_goal,
            orbit_radius_m=5.0,
        )

    except KeyboardInterrupt:
        print("\nInterruption demandée par l'utilisateur.")

    except Exception as error:
        print("Erreur pendant la mission :", error)

    finally:
        # Arrêt moteur prioritaire avant de fermer le matériel.
        if boat is not None:
            try:
                boat.stop_motor()
            except Exception as error:
                print("Erreur lors de l'arrêt des moteurs :", error)

            # Ferme le socket et le thread UDP si disponibles.
            fleet = getattr(boat, "fleet", None)
            if fleet is not None and hasattr(fleet, "close"):
                try:
                    fleet.close()
                except Exception:
                    pass

        if gps is not None:
            try:
                gps.close()
            except Exception:
                pass

        if telemetry is not None:
            try:
                telemetry.close()
            except Exception:
                pass

        if ard is not None:
            try:
                if ard.arduino is not None:
                    ard.arduino.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()