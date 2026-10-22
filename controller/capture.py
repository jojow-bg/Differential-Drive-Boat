import math
import time

EARTH_RADIUS = 6371000.0

SEPARATION_M = 5.0              # écart entre les deux bateaux pendant l'approche
MIN_SAFE_DISTANCE_M = 1.5       # distance minimale (APPROACH / ALIGN / CAPTURE)
RETURN_MIN_DISTANCE_M = 0.5     # seuil plus bas en RETURN (bateaux proches, corde détendue)
APPROACH_TOLERANCE_M = 5.0
HEADING_TOLERANCE_DEG = 10.0
MAX_GPS_AGE_S = 2.0             # 5 s = jusqu'à ~10 m d'erreur à 2 m/s
MAX_LOOP_GAP_S = 2.0            # trou maximal dans la boucle de contrôle
CONFIRM_TIME_S = 1.3           # une condition doit rester vraie ce temps avant validation
MIN_LATERAL_OFFSET_M = 0.5      # écart minimal à la ligne pour décider du côté
ORBIT_LOOKAHEAD_DEG = 10.0
APPROACH_STANDOFF_M = 1.0       # recul du centre de formation avant la bouée

# Durée maximale par état avant abandon (secondes).
STATE_TIMEOUT_S = {
    "APPROACH": 300.0,
    "ALIGN": 60.0,
    "CAPTURE": 120.0,
    "RETURN": 300.0,
}


def wrap_angle(angle):
    """Ramène un angle en radians dans [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def heading_deg_to_math(cap_deg):
    """Convertit le cap de ddboat.py en angle mathématique."""
    return math.radians(90.0 - cap_deg)


def math_to_heading_deg(angle):
    """Convertit un angle mathématique en cap de flotte dans [0, 360)."""
    return (90.0 - math.degrees(angle)) % 360.0


def gps_to_xy(latitude, longitude, reference_lat, reference_lon):
    """
    Approximation locale en mètres (valable sur de courtes distances).
    x = Est, y = Nord.
    """
    lat = math.radians(latitude)
    lon = math.radians(longitude)
    lat0 = math.radians(reference_lat)
    lon0 = math.radians(reference_lon)

    x = EARTH_RADIUS * (lon - lon0) * math.cos(lat0)
    y = EARTH_RADIUS * (lat - lat0)
    return x, y


def xy_to_gps(x, y, reference_lat, reference_lon):
    """Conversion inverse du repère local vers GPS."""
    lat0 = math.radians(reference_lat)
    latitude = reference_lat + math.degrees(y / EARTH_RADIUS)
    longitude = reference_lon + math.degrees(x / (EARTH_RADIUS * math.cos(lat0)))
    return latitude, longitude


def get_distance_xy(x1, y1, x2, y2):
    """Distance euclidienne en mètres."""
    return math.hypot(x2 - x1, y2 - y1)


def get_side_of_line(x, y, x0, y0, vx, vy, min_offset=0.0):
    """
    Côté d'un point par rapport à une ligne passant par (x0, y0)
    de direction UNITAIRE (vx, vy).

    +1 : côté gauche, -1 : côté droit,
     0 : sur la ligne (écart latéral <= min_offset).

    Le produit vectoriel d'un vecteur unitaire est directement
    l'écart latéral signé en mètres.
    """
    cross = vx * (y - y0) - vy * (x - x0)

    if abs(cross) <= min_offset:
        return 0
    return 1 if cross > 0 else -1


def get_rope_direction(x_a, y_a, x_b, y_b):
    """Direction de B vers A, en radians."""
    return math.atan2(y_a - y_b, x_a - x_b)


def get_desired_heading_b(x_a, y_a, x_b, y_b):
    """
    Cap souhaité de B : son arrière orienté vers A
    (point d'attache approximé au centre du bateau).
    """
    return wrap_angle(get_rope_direction(x_a, y_a, x_b, y_b) + math.pi)


def get_heading_error(desired_heading, current_heading):
    """Erreur angulaire signée, en radians."""
    return wrap_angle(desired_heading - current_heading)


def get_circle_waypoint(x_center, y_center, radius, theta):
    """Point cible sur un cercle dans le repère local."""
    return x_center + radius * math.cos(theta), y_center + radius * math.sin(theta)


def compute_geometry(pos_1_xy, pos_2_xy):
    """
    Calcule la ligne d'approche (milieu des deux bateaux -> bouée en (0, 0)).

    À appeler UNE SEULE FOIS (sur un bateau ou au sol) puis à partager aux
    deux contrôleurs via le paramètre `geometry`, pour que les deux bateaux
    utilisent exactement le même repère.
    """
    mx = (pos_1_xy[0] + pos_2_xy[0]) / 2.0
    my = (pos_1_xy[1] + pos_2_xy[1]) / 2.0

    norm = math.hypot(-mx, -my)
    if norm < 0.1:
        raise ValueError("Direction d'approche indéfinie : milieu initial trop proche de la bouée.")

    return {
        "line_origin": (mx, my),
        "line_direction": (-mx / norm, -my / norm),
    }


class CatchController:

    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    CAPTURE = "CAPTURE"
    RETURN = "RETURN"
    ABORT = "ABORT"
    DONE = "DONE"

    STATE_ORDER = {APPROACH: 0, ALIGN: 1, CAPTURE: 2, RETURN: 3, DONE: 4}

    def __init__(self, boat_id, is_orbiting_boat, base_position,
                 orbit_radius_m=SEPARATION_M, own_side=None,
                 sweep_forward=True, geometry=None):
        """
        boat_id : identifiant du bateau.
        is_orbiting_boat : True pour A (tourne), False pour B (centre).
        base_position : (latitude, longitude) de retour, DISTINCTE pour chaque bateau.
        orbit_radius_m : rayon de la trajectoire de A autour de B.
        own_side : optionnel, +1 (gauche) ou -1 (droite) attendu. Si fourni,
                   il est vérifié contre le côté réellement mesuré ; sinon il est déduit.
        sweep_forward : True = A balaie le demi-cercle côté bouée/avant,
                        False = côté arrière. Le sens de rotation en découle.
        geometry : géométrie partagée (voir compute_geometry), optionnelle.
        """
        if own_side not in (None, -1, 1):
            raise ValueError("own_side doit valoir None, +1 ou -1")

        if orbit_radius_m <= MIN_SAFE_DISTANCE_M:
            raise ValueError("Le rayon doit dépasser la distance minimale de sécurité")

        self.boat_id = int(boat_id)
        self.is_orbiting_boat = is_orbiting_boat
        self.base_position = base_position
        self.orbit_radius_m = orbit_radius_m
        self.own_side = own_side
        self.sweep_forward = sweep_forward
        self.geometry = geometry

        self.state = self.APPROACH

        self.reference_lat = None
        self.reference_lon = None
        self.approach_targets = None
        self.center_target = None
        self.orbit_direction = None
        self.orbit_start_angle = None
        self.base_xy = None

        self.previous_orbit_angle = None
        self.orbit_progress = 0.0

        self.last_update = None
        self.state_entered = None
        self.confirm_since = None

    # ------------------------------------------------------------------
    # Utilitaires internes
    # ------------------------------------------------------------------

    def _decision(self, target_gps=None, desired_heading_deg=None,
                  ready=False, stop_required=False, **extra):
        """
        Décision standard. `state` et `ready` sont à publier dans le
        FleetManager pour que l'autre bateau puisse se synchroniser.
        """
        decision = {
            "state": self.state,
            "ready": ready,
            "target_gps": target_gps,
            "desired_heading_deg": desired_heading_deg,
            "stop_required": stop_required,
        }
        decision.update(extra)
        return decision

    def abort(self, reason):
        """Interrompt la mission et mémorise la raison."""
        self.state = self.ABORT
        return self._decision(stop_required=True, reason=reason)

    def _set_state(self, new_state, now):
        self.state = new_state
        self.state_entered = now
        self.confirm_since = None

    def _confirmed(self, condition, now):
        """Vrai si la condition reste vraie pendant CONFIRM_TIME_S."""
        if not condition:
            self.confirm_since = None
            return False
        if self.confirm_since is None:
            self.confirm_since = now
        return now - self.confirm_since >= CONFIRM_TIME_S

    def _other_allows(self, other):
        """
        Barrière de synchronisation : on quitte l'état courant seulement si
        l'autre bateau a lui aussi terminé cet état (ready) ou l'a déjà quitté.
        """
        other_order = self.STATE_ORDER.get(other["state"])
        if other_order is None:
            return False

        own_order = self.STATE_ORDER[self.state]
        if other_order > own_order:
            return True
        return other_order == own_order and bool(other.get("ready"))

    def _initialize_geometry(self, own, other, waypoint_goal):
        """Repère local, côtés, cibles d'approche et sens d'orbite."""
        self.reference_lat, self.reference_lon = waypoint_goal

        own_xy = gps_to_xy(own["latitude"], own["longitude"],
                           self.reference_lat, self.reference_lon)
        other_xy = gps_to_xy(other["latitude"], other["longitude"],
                             self.reference_lat, self.reference_lon)

        if self.geometry is None:
            self.geometry = compute_geometry(own_xy, other_xy)

        x0, y0 = self.geometry["line_origin"]
        vx, vy = self.geometry["line_direction"]

        # Vérification des côtés avec get_side_of_line.
        measured_side = get_side_of_line(*own_xy, x0, y0, vx, vy, MIN_LATERAL_OFFSET_M)
        other_side = get_side_of_line(*other_xy, x0, y0, vx, vy, MIN_LATERAL_OFFSET_M)

        if measured_side == 0 or other_side == 0:
            raise ValueError("Un bateau est trop proche de la ligne d'approche pour définir son côté")

        if measured_side == other_side:
            raise ValueError("Les deux bateaux sont du même côté de la ligne d'approche")

        if self.own_side is not None and self.own_side != measured_side:
            raise ValueError("own_side ne correspond pas au côté réel du bateau")

        self.own_side = measured_side

        # Normale gauche : côté +1 = +n, côté -1 = -n (cohérent avec get_side_of_line).
        nx, ny = -vy, vx
        half = SEPARATION_M / 2.0

        # Centre de formation juste avant la bouée, du côté d'où arrivent les bateaux.
        cx = -vx * APPROACH_STANDOFF_M
        cy = -vy * APPROACH_STANDOFF_M

        self.approach_targets = {
            1: (cx + nx * half, cy + ny * half),
            -1: (cx - nx * half, cy - ny * half),
        }

        # Rôles : A orbite autour de B.
        side_a = self.own_side if self.is_orbiting_boat else -self.own_side
        self.center_target = self.approach_targets[-side_a]
        target_a = self.approach_targets[side_a]

        # Angle de départ de A autour de B, identique pour les deux bateaux.
        self.orbit_start_angle = math.atan2(target_a[1] - self.center_target[1],
                                            target_a[0] - self.center_target[0])

        # Sens de rotation déduit de la géométrie :
        # A part du côté side_a ; pour balayer l'avant (direction vx, vy),
        # il doit tourner dans le sens -side_a (horaire si A est à gauche).
        self.orbit_direction = -side_a if self.sweep_forward else side_a

        self.base_xy = gps_to_xy(self.base_position[0], self.base_position[1],
                                 self.reference_lat, self.reference_lon)

    # ------------------------------------------------------------------
    # Boucle principale
    # ------------------------------------------------------------------

    def update(self, own, other, waypoint_goal):
        """
        Calcule la prochaine décision ; n'actionne pas les moteurs.

        own   : {"latitude", "longitude", "heading_deg", "age"}
        other : même format + {"state", "ready"} publiés par l'autre bateau.
        waypoint_goal : (latitude, longitude) de la bouée.
        """
        now = time.monotonic()

        # --- Vérification des données ---
        required = ("latitude", "longitude", "heading_deg", "age")

        for data in (own, other):
            if any(data.get(key) is None for key in required):
                return self.abort("Données GPS ou cap manquants")

        if own["age"] > MAX_GPS_AGE_S:
            return self.abort("Données GPS trop anciennes")

        # Un partenaire en DONE a quitté sa boucle et n'émet plus :
        # ses données vieillissent normalement.
        partner_done = other.get("state") == self.DONE
        if not partner_done and other["age"] > MAX_GPS_AGE_S:
            return self.abort("Données de flotte trop anciennes")

        if other.get("state") is None:
            return self.abort("État de l'autre bateau manquant")

        if self.last_update is not None and now - self.last_update > MAX_LOOP_GAP_S:
            return self.abort("Interruption trop longue de la boucle de contrôle")
        self.last_update = now

        if self.state in (self.ABORT, self.DONE):
            return self._decision(stop_required=True)

        # Un abandon de l'autre bateau arrête aussi celui-ci.
        if other["state"] == self.ABORT:
            return self.abort("L'autre bateau a interrompu la mission")

        if self.state_entered is None:
            self.state_entered = now

        timeout = STATE_TIMEOUT_S.get(self.state)
        if timeout is not None and now - self.state_entered > timeout:
            return self.abort("Délai dépassé dans l'état {}".format(self.state))

        # --- Initialisation de la géométrie ---
        if self.reference_lat is None:
            try:
                self._initialize_geometry(own, other, waypoint_goal)
            except ValueError as exc:
                return self.abort(str(exc))

        own_xy = gps_to_xy(own["latitude"], own["longitude"],
                           self.reference_lat, self.reference_lon)
        other_xy = gps_to_xy(other["latitude"], other["longitude"],
                             self.reference_lat, self.reference_lon)

        distance = get_distance_xy(*own_xy, *other_xy)

        min_distance = RETURN_MIN_DISTANCE_M if self.state == self.RETURN else MIN_SAFE_DISTANCE_M
        if distance < min_distance:
            return self.abort("Distance entre les bateaux trop faible")

        # A = bateau qui orbite, B = bateau centre.
        a_xy, b_xy = (own_xy, other_xy) if self.is_orbiting_boat else (other_xy, own_xy)
        own_target_xy = self.approach_targets[self.own_side]

        # ----------------------------------------------------------
        # APPROACH : rejoindre sa cible de formation
        # ----------------------------------------------------------
        if self.state == self.APPROACH:
            target_gps = xy_to_gps(*own_target_xy, self.reference_lat, self.reference_lon)
            at_target = get_distance_xy(*own_xy, *own_target_xy) <= APPROACH_TOLERANCE_M
            ready = self._confirmed(at_target, now)

            print("[{}] APPROACH own_xy={}, target_xy={}, distance_cible={:.2f} m, at_target={}, ready={}, partner_state={}, partner_ready={}". format(self.boat_id, own_xy , own_target_xy, get_distance_xy(*own_xy, *own_target_xy), at_target, ready, other.get('state'),other.get('ready')))

            if ready and self._other_allows(other):
                self._set_state(self.ALIGN, now)
                ready = False

            return self._decision(target_gps=target_gps, ready=ready,
                                  distance_between_boats_m=distance)

        # ----------------------------------------------------------
        # ALIGN : maintien de position + orientation, avec barrière
        # ----------------------------------------------------------
        if self.state == self.ALIGN:
            if self.is_orbiting_boat:
                theta = math.atan2(a_xy[1] - b_xy[1], a_xy[0] - b_xy[0])
                desired_heading = wrap_angle(theta + self.orbit_direction * math.pi / 2.0)
            else:
                desired_heading = get_desired_heading_b(*a_xy, *b_xy)

            current_heading = heading_deg_to_math(own["heading_deg"])
            heading_error = get_heading_error(desired_heading, current_heading)

            aligned = abs(heading_error) <= math.radians(HEADING_TOLERANCE_DEG)
            in_place = get_distance_xy(*own_xy, *own_target_xy) <= APPROACH_TOLERANCE_M
            ready = self._confirmed(aligned and in_place, now)

            if ready and self._other_allows(other):
                self._set_state(self.CAPTURE, now)
                ready = False
                self.previous_orbit_angle = math.atan2(a_xy[1] - b_xy[1], a_xy[0] - b_xy[0])
                self.orbit_progress = 0.0

            # Le bateau tient sa position pendant l'alignement.
            hold_gps = xy_to_gps(*own_target_xy, self.reference_lat, self.reference_lon)

            return self._decision(target_gps=hold_gps,
                                  desired_heading_deg=math_to_heading_deg(desired_heading),
                                  ready=ready,
                                  heading_error_deg=math.degrees(heading_error),
                                  hold_position=True)

        # ----------------------------------------------------------
        # CAPTURE : demi-cercle coordonné
        # ----------------------------------------------------------
        if self.state == self.CAPTURE:
            theta = math.atan2(a_xy[1] - b_xy[1], a_xy[0] - b_xy[0])
            delta_theta = wrap_angle(theta - self.previous_orbit_angle)

            self.orbit_progress += self.orbit_direction * delta_theta
            self.previous_orbit_angle = theta

            ready = self.orbit_progress >= math.pi

            if ready and self._other_allows(other):
                self._set_state(self.RETURN, now)
                return self._decision(
                    target_gps=self.base_position,
                    orbit_progress_deg=math.degrees(self.orbit_progress),
                )

            if self.is_orbiting_boat:
                # Cible en avance sur le cercle, bornée à la fin du demi-cercle
                # pour que A s'arrête et attende B au lieu de continuer.
                remaining = max(math.pi - self.orbit_progress, 0.0)
                step = min(math.radians(ORBIT_LOOKAHEAD_DEG), remaining)
                target_theta = theta + self.orbit_direction * step

                # Le cercle est centré sur la position réelle de B.
                target_xy = get_circle_waypoint(b_xy[0], b_xy[1],
                                                self.orbit_radius_m, target_theta)
                target_gps = xy_to_gps(*target_xy, self.reference_lat, self.reference_lon)
                desired_heading = None
            else:
                # B tient sa position de centre (point partagé et déterministe),
                # arrière orienté vers A.
                target_gps = xy_to_gps(*self.center_target,
                                       self.reference_lat, self.reference_lon)
                desired_heading = math_to_heading_deg(get_desired_heading_b(*a_xy, *b_xy))

            return self._decision(
                target_gps=target_gps,
                desired_heading_deg=desired_heading,
                ready=ready,
                # B doit s'orienter sans que la cible GPS ne remplace
                # son cap par un cap de navigation vers le centre.
                hold_position=not self.is_orbiting_boat,
                track_target=self.is_orbiting_boat,
                orbit_progress_deg=math.degrees(self.orbit_progress),
                distance_between_boats_m=distance,
            )

        # ----------------------------------------------------------
        # RETURN : chaque bateau rejoint SA base (bases distinctes)
        # ----------------------------------------------------------
        if self.state == self.RETURN:
            distance_to_base = get_distance_xy(*own_xy, *self.base_xy)

            if distance_to_base <= APPROACH_TOLERANCE_M:
                self._set_state(self.DONE, now)
                return self._decision(stop_required=True)

            return self._decision(target_gps=self.base_position)

        return self.abort("État de mission inconnu")