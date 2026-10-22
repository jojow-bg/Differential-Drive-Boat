import math
import time
import csv
from pathlib import Path
import os
import sys

import numpy as np

# Ajoute la racine du projet au chemin de recherche
dossier_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_parent)

from controller.capture import CatchController


# Fréquence de la boucle de contrôle
CONTROL_PERIOD_S = 0.1

# Tolérance d'arrivée sur une cible GPS
WAYPOINT_TOLERANCE_M = 2.0

# Sécurité : une donnée partenaire plus vieille que cette durée
# provoque l'arrêt de la mission.
MAX_PARTNER_AGE_S = 5.0

# Un seul paquet UDP peut se perdre : l'état final est répété.
TERMINAL_STATE_REPEATS = 5

# Commandes moteur
PWM_MAX = 191
PWM_CRUISE = 100
PWM_SLOW = 65
SLOW_DOWN_DISTANCE_M = 5.0
TURN_IN_PLACE_GAIN = 80.0
# Plus serré que HEADING_TOLERANCE_DEG de capture.py (10°), sinon le bateau
# s'arrête pile à la limite et le bruit du cap annule la validation.
TURN_IN_PLACE_TOLERANCE_DEG = 5.0
# En dessous, les propulseurs ne font pas pivoter le bateau.
TURN_IN_PLACE_MIN_PWM = 50


def distance_m(lat1, lon1, lat2, lon2):
    """Distance approximative entre deux positions GPS en degrés."""
    radius = 6371000.0

    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2)
        * math.sin(dlon / 2.0) ** 2
    )

    return 2.0 * radius * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(lat1, lon1, lat2, lon2):
    """Cap compas (0° nord, 90° est) vers une cible GPS."""
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)

    y = math.sin(dlon) * math.cos(lat2)
    x = (
        math.cos(lat1) * math.sin(lat2)
        - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    )

    return math.degrees(math.atan2(y, x)) % 360.0


def wrap_angle(angle):
    """Ramène un angle en radians dans [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def execute_decision(boat, decision):
    """
    Exécute une décision de CatchController.
    Les caps de flotte et ddboat.heading sont des caps compas
    (0 = nord, 90° = est, sens horaire), en degrés et en radians.
    """
    if decision.get("stop_required"):
        boat.stop_motor()
        return

    if boat.lat_rad is None or boat.lon_rad is None or boat.heading is None:
        boat.stop_motor()
        return

    target = decision.get("target_gps")
    desired_heading = decision.get("desired_heading_deg")

    distance = None
    if target is not None:
        target_lat, target_lon = target
        current_lat = math.degrees(boat.lat_rad)
        current_lon = math.degrees(boat.lon_rad)
        distance = distance_m(current_lat, current_lon, target_lat, target_lon)

    on_target = distance is None or distance <= WAYPOINT_TOLERANCE_M

    # Sur le point, le cap demandé est tenu en tournant sur place :
    # avancer ferait dériver le bateau hors de sa position.
    if decision.get("hold_position") and desired_heading is not None and on_target:
        turn_in_place(boat, desired_heading)
        return

    # A doit suivre son point d'orbite en continu : ce point est toujours
    # plus proche que la tolérance d'arrivée.
    must_track = decision.get("track_target", False)

    if target is not None and (not on_target or must_track):
        desired_heading = bearing_deg(current_lat, current_lon, target_lat, target_lon)

    if desired_heading is None:
        boat.stop_motor()
        return

    error = wrap_angle(math.radians(desired_heading) - boat.heading)

    pwm = PWM_CRUISE
    if distance is not None and distance < SLOW_DOWN_DISTANCE_M:
        pwm = PWM_SLOW

    left, right = boat.compute_commands_diff(
        error,
        PWM_MOTOR=pwm,
        PWM_MAX=PWM_MAX,
    )
    boat.control_motor(left, right)


def turn_in_place(boat, desired_heading_deg):
    """Oriente le bateau vers un cap compas sans avancer."""
    error = wrap_angle(math.radians(desired_heading_deg) - boat.heading)

    if abs(error) < math.radians(TURN_IN_PLACE_TOLERANCE_DEG):
        boat.stop_motor()
        return

    turn = TURN_IN_PLACE_GAIN * error
    turn = math.copysign(max(abs(turn), TURN_IN_PLACE_MIN_PWM), turn)
    turn = int(max(-PWM_MAX, min(PWM_MAX, turn)))
    boat.control_motor(turn, -turn)


def publish_terminal_state(boat, state):
    """Annonce DONE ou ABORT au partenaire avant de quitter la boucle."""
    for _ in range(TERMINAL_STATE_REPEATS):
        try:
            boat.update_fleet(state=state, ready=False)
        except Exception as error:
            print("[MISSION] Impossible de publier l'état", state, ":", error)
        time.sleep(CONTROL_PERIOD_S)


def run_cooperative_mission(
    boat,
    boat_id,
    is_orbiting_boat,
    base_position,
    waypoint_goal,
    orbit_radius_m=5.0,
):
    """
    Exécute la capture coopérative jusqu'à DONE, ABORT
    ou une interruption.

    Les positions GPS et waypoint_goal sont en degrés décimaux.
    """
    controller = CatchController(
        boat_id=boat_id,
        is_orbiting_boat=is_orbiting_boat,
        base_position=base_position,
        orbit_radius_m=orbit_radius_m,
    )

    state = controller.APPROACH
    ready = False

    print("[MISSION] Démarrage du contrôleur coopératif.")

    try:
        while True:
            loop_start = time.monotonic()

            # Actualiser les capteurs
            boat.update_gps()
            boat.update_heading()

            # Sans GPS ou cap valides, arrêter par sécurité.
            if (
                boat.lat_rad is None
                or boat.lon_rad is None
                or boat.heading is None
            ):
                boat.stop_motor()
                print("[MISSION] Attente des mesures GPS/IMU.")
                time.sleep(CONTROL_PERIOD_S)
                continue

            # Publier position + état de mission et recevoir
            # les données du partenaire.
            #
            # Nécessite l'adaptation de DirectFleetManager et
            # de ddboat.update_fleet() décrite précédemment.
            other_boats = boat.update_fleet(
                state=state,
                ready=ready,
            )

            if not other_boats:
                boat.stop_motor()
                print("[MISSION] En attente du bateau partenaire.")
                time.sleep(CONTROL_PERIOD_S)
                continue

            other = other_boats[0]

            partner_done = other.get("state") == controller.DONE
            if not partner_done and other.get("age", float("inf")) > MAX_PARTNER_AGE_S:
                boat.stop_motor()
                controller.abort("Données du partenaire périmées")
                state = controller.ABORT
                ready = False
                # Publier explicitement ABORT pour que le partenaire
                # puisse lui aussi interrompre sa mission.
                publish_terminal_state(boat, state)
                print("[MISSION] ABORT : données du partenaire périmées.")
                break

            # Dictionnaires attendus par CatchController.
            own = {
                "latitude": math.degrees(boat.lat_rad),
                "longitude": math.degrees(boat.lon_rad),
                "heading_deg": boat.heading_to_compass(),
                "age": (
                    max(0.0, time.monotonic() - boat.gps_last_update)
                    if getattr(boat, "gps_last_update", None) is not None
                    else float("inf")
                ),
            }

            partner_state = other.get("state")
            partner_ready = other.get("ready", False)

            partner = {
                "latitude": other.get("latitude"),
                "longitude": other.get("longitude"),
                "heading_deg": other.get("heading_deg"),
                "age": other.get("age"),
                "state": partner_state,
                "ready": partner_ready,
            }

            # Sans état partenaire, ne pas lancer la capture.
            if partner_state is None:
                boat.stop_motor()
                print("[MISSION] Le partenaire n'envoie pas encore state.")
                time.sleep(CONTROL_PERIOD_S)
                continue

            # Calcul de la prochaine décision.
            decision = controller.update(
                own=own,
                other=partner,
                waypoint_goal=waypoint_goal,
            )

            state = decision["state"]
            ready = decision["ready"]

            print(
                "[MISSION] état={}, ready={}, distance={}".format(
                    state,
                    ready,
                    decision.get("distance_between_boats_m"),
                )
            )

            # Exécution de la décision moteur.
            execute_decision(boat, decision)

            if state in (controller.ABORT, controller.DONE):
                boat.stop_motor()
                publish_terminal_state(boat, state)

                print("[MISSION] Fin de mission :", state)

                if state == controller.ABORT:
                    print("[MISSION] Motif :", decision.get("reason"))

                break

            elapsed = time.monotonic() - loop_start
            time.sleep(max(0.0, CONTROL_PERIOD_S - elapsed))

    finally:
        # Arrêt moteur quelles que soient les circonstances.
        boat.stop_motor()
        print("[MISSION] Moteurs arrêtés.")