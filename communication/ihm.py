"""
IHM DDBoat : tableau de bord de télémétrie en direct (lac de Guerlédan).

Affiche : carte du lac (zone de navigation), waypoints, trajectoire, bateau,
autres bateaux (canal à venir), boussole, propulseurs, indicateurs clés.
Lancer :  python3 ihm.py

Messages reçus (JSON, une ligne par message) : les mêmes qu'avant, plus
deux champs OPTIONNELS :
  "target_lat", "target_lon" : waypoint visé actuellement (degrés)
  "boats" : [{"id": "DDBoat03", "latitude": ..., "longitude": ...,
              "heading_deg": ...}, ...]   (autres bateaux)
"""

import json
import math
import socket
import time
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from matplotlib.colors import to_rgba
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, FancyBboxPatch, Patch, Polygon

# Rives du lac (optionnel) : fichier lake_contour.py à côté de ihm.py
try:
    from lake_contour import CONTOUR as _LAKE
except ImportError:
    _LAKE = []


# --------------------------------------------------
# CONFIGURATION
# --------------------------------------------------

SERVER_IP = "172.20.25.202"      # adresse IP du DDBoat
SERVER_PORT = 5002
MY_NAME = "DDBoat02"             # nom affiché pour ce bateau

# Origine du repère local du bateau (= "Base", comme dans ddboat.py)
ORIGIN_LAT, ORIGIN_LON = 48.198960, -3.015091

# Zone de navigation (le "polygon" : A, G, C, D, G', E, F), en degrés.
# Le G final en double a été retiré. Mettre la même liste dans
# ddboat.zone_limit pour que le bateau et l'IHM aient la même zone.
ZONE_POINTS = [
    ("A",  48.198960, -3.015091),
    ("G",  48.198944, -3.015478),
    ("C",  48.198124, -3.015355),
    ("D",  48.198259, -3.017114),
    ("G'", 48.199027, -3.015887),
    ("E",  48.199628, -3.016379),
    ("F",  48.199688, -3.015158),
]
ZONE_LIMIT = [(la, lo) for _n, la, lo in ZONE_POINTS]

# Ancienne zone (ancien ddboat.zone_limit), gardée pour comparaison
OLD_ZONE_LIMIT = [
    (48.198863, -3.015070),
    (48.197408, -3.015060),
    (48.1970689, -3.017328),
    (48.199072, -3.016258),
    (48.199707, -3.016404),
    (48.199814, -3.015137),
]

# Waypoints : (nom, latitude°, longitude°, tolérance m, cible de navigation ?)
#   cible=True  : waypoint que le bateau rallie (étoile + cercle)
#   cible=False : point de repère de la zone (losange)
WAYPOINTS = [
    ("WP1", 48.198863, -3.016258, 2.0, True),    
    ("WP2", 48.196500, -3.016900, 5.0, True),  
] + [(n, la, lo, 2.0, False) for n, la, lo in ZONE_POINTS]

PWM_MAX = 100
R_EARTH = 6371000.0
STALE_AFTER = 1.0                # s sans message => liaison perdue
RECONNECT_EVERY = 2.0            # s entre deux tentatives
OTHER_STALE = 5.0                # s : autre bateau grisé
OTHER_LOST = 30.0                # s : autre bateau retiré de la carte
MAX_FLEET_ROWS = 5

# Palette fintech
BG = "#080c16"
CARD = "#10172a"
INNER = "#0b1120"
BORDER = "#1f2a44"
GRID = "#1a2338"
TRACK = "#1b2540"
TEXT = "#f1f5f9"
MUTED = "#7c8aa5"
ACCENT = "#00d4aa"
VIOLET = "#8b5cf6"
AMBER = "#fbbf24"
RED = "#fb7185"
WATER = "#12446b"
SHORE = "#4da3dd"      # rives du lac
OLD = "#94a3b8"        # ancienne zone

FIG_W, FIG_H = 15, 8.6

plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Inter", "SF Pro Display", "Segoe UI",
                                   "Helvetica Neue", "Arial", "DejaVu Sans"]


# --------------------------------------------------
# LIAISON TCP (avec reconnexion automatique)
# --------------------------------------------------

class Link:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None
        self.buffer = b""
        self.last_try = 0.0
        self.last_msg = 0.0

    @property
    def connected(self):
        return self.sock is not None

    def _connect(self):
        self.last_try = time.monotonic()
        try:
            sock = socket.create_connection((self.host, self.port), timeout=0.4)
            sock.setblocking(False)
            self.sock = sock
            self.buffer = b""
        except OSError:
            self.sock = None

    def _drop(self):
        try:
            self.sock.close()
        except (OSError, AttributeError):
            pass
        self.sock = None

    def poll(self):
        """Retourne la liste de TOUS les messages reçus depuis le dernier appel."""
        if self.sock is None:
            if time.monotonic() - self.last_try > RECONNECT_EVERY:
                self._connect()
            return []

        while True:
            try:
                chunk = self.sock.recv(4096)
            except BlockingIOError:
                break
            except OSError:
                self._drop()
                return []
            if not chunk:
                self._drop()
                break
            self.buffer += chunk

        messages = []
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            if not line.strip():
                continue
            try:
                messages.append(json.loads(line.decode("utf-8")))
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue

        if messages:
            self.last_msg = time.monotonic()
        return messages

    def close(self):
        if self.sock is not None:
            self._drop()


# --------------------------------------------------
# OUTILS GÉOMÉTRIQUES
# --------------------------------------------------

def wrap_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def fmt(value, spec="{:.2f}", unit=""):
    if value is None:
        return "--"
    return spec.format(value) + unit


def to_local(lat_deg, lon_deg):
    """GPS (degrés) -> mètres autour de l'origine (x Est, y Nord)."""
    x = R_EARTH * math.cos(math.radians(ORIGIN_LAT)) \
        * math.radians(lon_deg - ORIGIN_LON)
    y = R_EARTH * math.radians(lat_deg - ORIGIN_LAT)
    return x, y


def msg_xy(msg):
    """Position locale d'un message : lat/lon en priorité, sinon x/y."""
    lat, lon = msg.get("latitude"), msg.get("longitude")
    if lat is not None and lon is not None:
        return to_local(lat, lon)
    x, y = msg.get("x"), msg.get("y")
    if x is not None and y is not None:
        return (x, y)
    return None


def point_in_poly(px, py, poly):
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > py) != (yj > py) and \
                px < (xj - xi) * (py - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


ZONE_XY = [to_local(la, lo) for la, lo in ZONE_LIMIT]
OLD_ZONE_XY = [to_local(la, lo) for la, lo in OLD_ZONE_LIMIT]
LAKE_XY = (np.array([to_local(p[1], p[0]) for p in _LAKE])
           if len(_LAKE) else np.empty((0, 2)))
WP_XY = [to_local(la, lo) for _n, la, lo, _t, _c in WAYPOINTS]
WP_INSIDE = [point_in_poly(x, y, ZONE_XY) for x, y in WP_XY]


# --------------------------------------------------
# OUTILS D'AFFICHAGE
# --------------------------------------------------

CARDS = []


def card(x, y, w, h, color=CARD):
    """Carte arrondie en coordonnées figure."""
    p = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0,rounding_size=0.012",
        mutation_aspect=FIG_W / FIG_H,
        transform=fig.transFigure,
        fc=color, ec=BORDER, lw=1, zorder=-5)
    fig.add_artist(p)
    CARDS.append(p)


def kpi_tile(x, y, w, h, label):
    card(x, y, w, h)
    fig.text(x + 0.012, y + h - 0.026, label.upper(), color=MUTED,
             fontsize=8.5, fontweight="bold", va="center")
    return fig.text(x + 0.012, y + 0.027, "--", color=TEXT, fontsize=17,
                    fontweight="bold", va="center")


def style_inner(ax):
    ax.set_facecolor(INNER)
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.tick_params(colors=MUTED, labelsize=8, length=0)


def maximize_window():
    """Ouvre la fenêtre agrandie (selon le backend graphique)."""
    try:
        window = fig.canvas.manager.window
    except Exception:
        return
    for action in (lambda: window.showMaximized(),            # Qt
                   lambda: window.state("zoomed"),            # Tk Windows
                   lambda: window.attributes("-zoomed", True)):  # Tk Linux
        try:
            action()
            return
        except Exception:
            continue


def on_resize(event):
    """Garde des coins arrondis réguliers quelle que soit la fenêtre."""
    w, h = fig.get_size_inches()
    if h > 0:
        for p in CARDS:
            p.set_mutation_aspect(w / h)


# --------------------------------------------------
# CONSTRUCTION DE L'INTERFACE
# --------------------------------------------------

plt.rcParams["toolbar"] = "None"
fig = plt.figure(figsize=(FIG_W, FIG_H), facecolor=BG)
try:
    fig.canvas.manager.set_window_title("DDBoat - Tableau de bord")
except Exception:
    pass
maximize_window()

# ---- En-tête ----
fig.text(0.030, 0.952, "◆", color=ACCENT, fontsize=20, va="center")
fig.text(0.050, 0.960, "DDBoat", color=TEXT, fontsize=20,
         fontweight="bold", va="center")
fig.text(0.050, 0.930, "Télémétrie en direct  ·  Lac de Guerlédan",
         color=MUTED, fontsize=10, va="center")
status = fig.text(0.970, 0.945, "", color=ACCENT, fontsize=11,
                  fontweight="bold", ha="right", va="center")

# ---- Indicateurs clés ----
kpi_labels = ["Distance", "Cap", "Erreur de cap", "Zone", "Waypoint",
              "Points GPS"]
gap = 0.012
tile_w = (0.94 - gap * (len(kpi_labels) - 1)) / len(kpi_labels)
tile_y, tile_h = 0.795, 0.10
kpis = [kpi_tile(0.03 + i * (tile_w + gap), tile_y, tile_w, tile_h, lab)
        for i, lab in enumerate(kpi_labels)]
K_DIST, K_CAP, K_ERR, K_ZONE, K_WP, K_PTS = kpis

# ---- Carte (colonne de gauche, large) ----
card(0.03, 0.065, 0.715, 0.705)
fig.text(0.045, 0.745, "Lac de Guerlédan", color=TEXT, fontsize=12,
         fontweight="bold", va="center")
fig.text(0.730, 0.745, "touches :  1 ancienne zone   2 rives     x Est / y Nord (m)",
         color=MUTED, fontsize=9,
         ha="right", va="center")

ax_map = fig.add_axes([0.040, 0.080, 0.695, 0.635])
style_inner(ax_map)
ax_map.grid(True, color=GRID, linewidth=0.8)
ax_map.set_axisbelow(True)

# Rives du lac (contour fermé => l'eau est colorée en plus)
lake_artists = []
if len(LAKE_XY):
    closed = (not np.isnan(LAKE_XY).any()
              and np.allclose(LAKE_XY[0], LAKE_XY[-1]))
    if closed:
        lake_artists.append(ax_map.add_patch(Polygon(
            LAKE_XY, closed=True, fc=to_rgba(WATER, 0.30), ec="none",
            zorder=0.5)))
    shore, = ax_map.plot(LAKE_XY[:, 0], LAKE_XY[:, 1], "-", color=SHORE,
                         lw=1.8, solid_joinstyle="round", zorder=1.5)
    lake_artists.append(shore)

# Ancienne zone (pointillés gris)
old_zone = ax_map.add_patch(Polygon(OLD_ZONE_XY, closed=True, fill=False,
                                    ec=OLD, ls=(0, (2, 3)), lw=1.5,
                                    zorder=1.2))
old_label = ax_map.annotate("ancienne zone", OLD_ZONE_XY[2], xytext=(8, -4),
                            textcoords="offset points", color=OLD,
                            fontsize=8.5, style="italic", zorder=6)

# Zone actuelle (le "polygon" A, G, C, D, G', E, F)
ax_map.add_patch(Polygon(ZONE_XY, closed=True, fc=to_rgba(WATER, 0.55),
                         ec=ACCENT, ls="--", lw=1.6, zorder=1.3))
ax_map.annotate("zone actuelle", ZONE_XY[3], xytext=(-6, -16),
                textcoords="offset points", color=ACCENT, fontsize=8.5,
                style="italic", ha="right", zorder=6)

# Base (origine du repère = point de retour)
ax_map.plot([0], [0], "o", mfc=INNER, mec=MUTED, mew=2, ms=9, zorder=4)
ax_map.annotate("Base", (0, 0), xytext=(9, -14), textcoords="offset points",
                color=MUTED, fontsize=9, zorder=6)

# Décalage des étiquettes (points, alignement) quand des points sont proches
LABEL_OFFSETS = {
    "WP1": (-12, -16, "right"),
    "G":   (9, -17, "left"),
    "G'":  (-9, 10, "right"),
}

# Waypoints
wp_artists = []
for (name, _la, _lo, tol, cible), (wx, wy), inside in zip(WAYPOINTS, WP_XY,
                                                        WP_INSIDE):
    if cible:
        col = AMBER if inside else RED
        circ = Circle((wx, wy), tol, fc=to_rgba(col, 0.12), ec=col, ls="--",
                      lw=1.2, zorder=3)
        ax_map.add_patch(circ)
        pt, = ax_map.plot([wx], [wy], marker="*", color=col, ms=14,
                          mec=BG, mew=0.8, ls="None", zorder=5)
        label = name if inside else name + "  (hors zone)"
        base_ms = 14
        size = 9
    else:
        col = AMBER
        circ = None
        pt, = ax_map.plot([wx], [wy], marker="D", color=col, ms=7,
                          mec=BG, mew=0.8, ls="None", zorder=5)
        label = name
        base_ms = 7
        size = 8.5
    dx, dy, ha = LABEL_OFFSETS.get(name, (9, 9, "left"))
    ax_map.annotate(label, (wx, wy), xytext=(dx, dy), ha=ha,
                    textcoords="offset points", color=TEXT, fontsize=size,
                    fontweight="bold", zorder=6,
                    bbox=dict(boxstyle="round,pad=0.22", fc=to_rgba(BG, 0.85),
                              ec=to_rgba(col, 0.6)))
    wp_artists.append((pt, circ, col, base_ms))

# Trajectoire : halo + trait dégradé
trail_glow = LineCollection([], linewidths=7, zorder=2)
trail = LineCollection([], linewidths=2.5, zorder=3)
ax_map.add_collection(trail_glow)
ax_map.add_collection(trail)
leg_line, = ax_map.plot([], [], ":", color=AMBER, lw=1.3, alpha=0.8, zorder=2)
boat_pt, = ax_map.plot([], [], marker=(3, 0, 0), color=ACCENT, ms=17,
                       mec=BG, mew=1, ls="None", zorder=7)

ax_map.legend(
    handles=[
        Line2D([], [], marker=(3, 0, 0), color=ACCENT, ls="None", ms=10,
               label="Bateau"),
        Line2D([], [], marker=(3, 0, 0), color=VIOLET, ls="None", ms=10,
               label="Autres bateaux"),
        Line2D([], [], marker="*", color=AMBER, ls="None", ms=11,
               label="Waypoint cible"),
        Line2D([], [], marker="D", color=AMBER, ls="None", ms=6,
               label="Point de la zone"),
        Patch(fc=to_rgba(WATER, 0.55), ec=ACCENT, ls="--",
              label="Zone actuelle"),
        Patch(fc="none", ec=OLD, ls=":", label="Ancienne zone"),
    ] + ([Line2D([], [], color=SHORE, lw=2, label="Rive du lac")]
         if len(LAKE_XY) else []),
    loc="upper left", facecolor=CARD, edgecolor=BORDER, labelcolor=TEXT,
    fontsize=9, framealpha=0.95)

# ---- Colonne de droite : boussole ----
RX, RW = 0.76, 0.21
card(RX, 0.470, RW, 0.300)
fig.text(RX + 0.012, 0.748, "Cap", color=TEXT, fontsize=11,
         fontweight="bold", va="center")
comp_value = fig.text(RX + RW - 0.012, 0.748, "--", color=ACCENT,
                      fontsize=11, fontweight="bold", ha="right",
                      va="center")

ax_comp = fig.add_axes([0.8105, 0.525, 0.1089, 0.19], projection="polar")
ax_comp.set_facecolor(INNER)
ax_comp.set_theta_zero_location("N")
ax_comp.set_theta_direction(-1)
ax_comp.set_ylim(0, 1)
ax_comp.set_yticks([])
ax_comp.set_xticks([math.radians(a) for a in range(0, 360, 90)])
ax_comp.set_xticklabels(["N", "E", "S", "O"], color=TEXT, fontsize=8,
                        fontweight="bold")
ax_comp.tick_params(axis="x", pad=1)
ax_comp.grid(True, color=GRID, linewidth=0.8)
ax_comp.spines["polar"].set_color(BORDER)
for a in range(0, 360, 15):
    r0 = 0.88 if a % 45 == 0 else 0.94
    ax_comp.plot([math.radians(a)] * 2, [r0, 1.0], color=MUTED,
                 lw=0.9, alpha=0.8)

bearing_arrow = ax_comp.annotate(
    "", xy=(0, 0.95), xytext=(0, 0),
    arrowprops=dict(arrowstyle="->", lw=1.8, color=AMBER, ls="--",
                    mutation_scale=13, shrinkA=0, shrinkB=0),
    visible=False)
needle = ax_comp.annotate(
    "", xy=(0, 0.80), xytext=(0, 0),
    arrowprops=dict(arrowstyle="-|>", lw=3, color=ACCENT,
                    mutation_scale=17, shrinkA=0, shrinkB=0))
needle_tail = ax_comp.annotate(
    "", xy=(math.pi, 0.3), xytext=(0, 0),
    arrowprops=dict(arrowstyle="-", lw=2, color=MUTED,
                    shrinkA=0, shrinkB=0))

fig.text(RX + 0.030, 0.488, "●", color=ACCENT, fontsize=8, va="center")
fig.text(RX + 0.040, 0.488, "Cap", color=MUTED, fontsize=8, va="center")
fig.text(RX + 0.090, 0.488, "●", color=AMBER, fontsize=8, va="center")
fig.text(RX + 0.100, 0.488, "Waypoint", color=MUTED, fontsize=8,
         va="center")

# ---- Propulseurs ----
card(RX, 0.300, RW, 0.155)
fig.text(RX + 0.012, 0.433, "Propulseurs", color=TEXT, fontsize=11,
         fontweight="bold", va="center")
fig.text(RX + RW - 0.012, 0.433, "PWM", color=MUTED, fontsize=8,
         ha="right", va="center")

ax_mot = fig.add_axes([RX + 0.050, 0.325, 0.150, 0.085])
style_inner(ax_mot)
ax_mot.set_facecolor(CARD)
ax_mot.set_xlim(0, PWM_MAX)
ax_mot.set_ylim(-0.6, 1.6)
ax_mot.set_yticks([1, 0])
ax_mot.set_yticklabels(["Gauche", "Droit"], color=TEXT, fontsize=9,
                       fontweight="bold")
ax_mot.set_xticks([0, 50, 100])
ax_mot.tick_params(axis="x", labelsize=7)
ax_mot.barh([1, 0], [PWM_MAX] * 2, height=0.5, color=TRACK, zorder=1)
bars = ax_mot.barh([1, 0], [0, 0], height=0.5, color=[ACCENT, ACCENT],
                   zorder=2)
mot_txt = [
    ax_mot.text(2, 1, "", color=TEXT, va="center", fontweight="bold",
                fontsize=9, zorder=3),
    ax_mot.text(2, 0, "", color=TEXT, va="center", fontweight="bold",
                fontsize=9, zorder=3),
]

# ---- Flotte (bateaux sur le canal) ----
card(RX, 0.065, RW, 0.220)
fig.text(RX + 0.012, 0.263, "Bateaux", color=TEXT, fontsize=11,
         fontweight="bold", va="center")
fleet_count = fig.text(RX + RW - 0.012, 0.263, "", color=MUTED, fontsize=8,
                       ha="right", va="center")
fleet_dots, fleet_txt = [], []
for i in range(MAX_FLEET_ROWS):
    ry = 0.228 - i * 0.032
    fleet_dots.append(fig.text(RX + 0.012, ry, "", fontsize=9, va="center"))
    fleet_txt.append(fig.text(RX + 0.024, ry, "", color=TEXT, fontsize=9,
                              va="center", family="monospace"))

# ---- Pied de page ----
footer_pos = fig.text(0.030, 0.030, "", color=MUTED, fontsize=9, va="center",
                      family="monospace")
fig.text(0.970, 0.030, "{}:{}".format(SERVER_IP, SERVER_PORT), color=MUTED,
         fontsize=9, ha="right", va="center", family="monospace")

fig.canvas.mpl_connect("resize_event", on_resize)


def on_key(event):
    """1 : ancienne zone    2 : rives du lac."""
    if event.key == "1":
        vis = not old_zone.get_visible()
        old_zone.set_visible(vis)
        old_label.set_visible(vis)
    elif event.key == "2":
        for art in lake_artists:
            art.set_visible(not art.get_visible())
    else:
        return
    fig.canvas.draw_idle()


fig.canvas.mpl_connect("key_press_event", on_key)


# --------------------------------------------------
# ÉTAT
# --------------------------------------------------

link = Link(SERVER_IP, SERVER_PORT)
latest = {}
xs, ys = [], []
others = {}            # id -> {x, y, heading, seen, ln, lbl}


def update_others(boats):
    """Met à jour les autres bateaux (canal inter-bateaux)."""
    now = time.monotonic()
    for b in boats:
        pos = msg_xy(b)
        if pos is None:
            continue
        bid = str(b.get("id") or b.get("name") or "?")
        o = others.get(bid)
        if o is None:
            ln, = ax_map.plot([], [], marker=(3, 0, 0), color=VIOLET, ms=14,
                              mec=BG, mew=1, ls="None", zorder=6)
            lbl = ax_map.annotate(bid, pos, xytext=(10, 10),
                                  textcoords="offset points", color=TEXT,
                                  fontsize=9, fontweight="bold", zorder=6)
            o = others[bid] = {"ln": ln, "lbl": lbl}
        o.update(x=pos[0], y=pos[1], heading=b.get("heading_deg"), seen=now)


TARGET_IDX = [i for i, w in enumerate(WAYPOINTS) if w[4]] or [0]


def active_waypoint(msg, pos):
    """Indice du waypoint visé : target_lat/lon si fournis, sinon d'après
    la distance reçue (parmi les waypoints cibles uniquement)."""
    tl, tn = msg.get("target_lat"), msg.get("target_lon")
    if tl is not None and tn is not None:
        tx, ty = to_local(tl, tn)
        return min(range(len(WP_XY)),
                   key=lambda i: math.hypot(WP_XY[i][0] - tx,
                                            WP_XY[i][1] - ty))
    d = msg.get("distance_m")
    if d is not None and pos is not None:
        return min(TARGET_IDX,
                   key=lambda i: abs(math.hypot(WP_XY[i][0] - pos[0],
                                                WP_XY[i][1] - pos[1]) - d))
    return TARGET_IDX[0]


def fit_view(pos):
    """Cadre la carte sur zone + waypoints + bateaux, en remplissant
    toute la largeur disponible (proportions réelles conservées)."""
    px = [p[0] for p in ZONE_XY] + [p[0] for p in WP_XY] + [0.0] + list(xs)
    py = [p[1] for p in ZONE_XY] + [p[1] for p in WP_XY] + [0.0] + list(ys)
    if pos is not None:
        px.append(pos[0])
        py.append(pos[1])
    for o in others.values():
        px.append(o["x"])
        py.append(o["y"])

    bb = ax_map.bbox
    ratio = bb.height / bb.width if bb.width > 0 else 0.5
    cx = (min(px) + max(px)) / 2
    cy = (min(py) + max(py)) / 2
    need_w = (max(px) - min(px)) * 1.12
    need_h = (max(py) - min(py)) * 1.12
    width = max(need_w, need_h / ratio, 20.0)
    height = width * ratio
    ax_map.set_xlim(cx - width / 2, cx + width / 2)
    ax_map.set_ylim(cy - height / 2, cy + height / 2)


def draw_trail():
    n = len(xs)
    if n < 2:
        trail.set_segments([])
        trail_glow.set_segments([])
        return
    pts = np.column_stack([xs, ys])
    segs = np.stack([pts[:-1], pts[1:]], axis=1)
    fade = np.linspace(0.15, 1.0, n - 1)
    trail.set_segments(segs)
    trail.set_color([to_rgba(ACCENT, a) for a in fade])
    trail_glow.set_segments(segs)
    trail_glow.set_color([to_rgba(ACCENT, a * 0.15) for a in fade])


def set_status(text, color):
    status.set_text(text)
    status.set_color(color)
    status.set_bbox(dict(boxstyle="round,pad=0.5,rounding_size=0.8",
                         fc=to_rgba(color, 0.12), ec=to_rgba(color, 0.6)))


def draw_others(now, pos):
    for bid, o in list(others.items()):
        age = now - o["seen"]
        if age > OTHER_LOST:
            o["ln"].remove()
            o["lbl"].remove()
            del others[bid]
            continue
        o["ln"].set_data([o["x"]], [o["y"]])
        o["ln"].set_color(VIOLET if age <= OTHER_STALE else MUTED)
        o["ln"].set_marker((3, 0, -(o["heading"] or 0)))
        o["lbl"].xy = (o["x"], o["y"])

    # Liste dans la carte "Bateaux"
    rows = [(ACCENT, "{:<9} {:>7}".format(MY_NAME[:9], "(moi)"))]
    for bid, o in sorted(others.items()):
        age = now - o["seen"]
        dist = (fmt(math.hypot(o["x"] - pos[0], o["y"] - pos[1]), "{:.0f}",
                    " m") if pos is not None else "--")
        rows.append((VIOLET if age <= OTHER_STALE else MUTED,
                     "{:<9} {:>7}".format(bid[:9], dist)))
    if not others:
        rows.append((MUTED, "En attente du canal..."))

    for i in range(MAX_FLEET_ROWS):
        if i < len(rows):
            fleet_dots[i].set_text("●")
            fleet_dots[i].set_color(rows[i][0])
            fleet_txt[i].set_text(rows[i][1])
            fleet_txt[i].set_color(MUTED if rows[i][0] == MUTED else TEXT)
        else:
            fleet_dots[i].set_text("")
            fleet_txt[i].set_text("")
    fleet_count.set_text("{} en ligne".format(1 + len(others)))


# --------------------------------------------------
# MISE À JOUR
# --------------------------------------------------

def update(_):
    global latest

    for msg in link.poll():
        latest = msg
        pos = msg_xy(msg)
        if msg.get("new_gps") and pos is not None:
            xs.append(pos[0])
            ys.append(pos[1])
        if msg.get("boats"):
            update_others(msg["boats"])

    pos = msg_xy(latest) if latest else None
    now = time.monotonic()

    # ---- Statut de liaison ----
    if not link.connected:
        set_status("●  HORS LIGNE - reconnexion...", RED)
    elif link.last_msg == 0 or now - link.last_msg > STALE_AFTER:
        set_status("●  CONNECTÉ - en attente de données", AMBER)
    elif pos is None:
        set_status("●  EN DIRECT - attente fix GPS", AMBER)
    else:
        set_status("●  EN DIRECT", ACCENT)

    draw_others(now, pos)
    fit_view(pos)

    if not latest:
        return

    heading = latest.get("heading_deg")
    left = latest.get("motor_left") or 0
    right = latest.get("motor_right") or 0
    distance = latest.get("distance_m")
    reached = bool(latest.get("target_reached"))
    active = active_waypoint(latest, pos)
    tx, ty = WP_XY[active]

    # ---- Carte ----
    draw_trail()
    for i, (pt, circ, col, base_ms) in enumerate(wp_artists):
        is_active = (i == active)
        pt.set_markersize(base_ms * 1.45 if is_active else base_ms)
        pt.set_color(ACCENT if (is_active and reached) else col)
        if circ is not None:
            circ.set_linewidth(2.2 if is_active else 1.0)

    if pos is not None:
        boat_pt.set_data([pos[0]], [pos[1]])
        boat_pt.set_marker((3, 0, -heading if heading is not None else 0))
        leg_line.set_data([pos[0], tx], [pos[1], ty])
    else:
        boat_pt.set_data([], [])
        leg_line.set_data([], [])

    # ---- Boussole ----
    bearing = None
    if pos is not None and not reached:
        bearing = math.atan2(tx - pos[0], ty - pos[1])

    if heading is not None:
        th = math.radians(heading)
        needle.xy = (th, 0.80)
        needle_tail.xy = (th + math.pi, 0.3)
        comp_value.set_text("{:05.1f}°".format(heading % 360))
    else:
        comp_value.set_text("--")

    if bearing is not None:
        bearing_arrow.xy = (bearing, 0.95)
        bearing_arrow.set_visible(True)
    else:
        bearing_arrow.set_visible(False)

    # ---- Propulseurs ----
    for bar, txt, value in zip(bars, mot_txt, (left, right)):
        w = max(0, min(value, PWM_MAX))
        bar.set_width(w)
        bar.set_color(RED if value > 0.9 * PWM_MAX
                      else AMBER if value > 0.7 * PWM_MAX else ACCENT)
        txt.set_text("{:.0f}".format(value))
        txt.set_x(min(w + 2, PWM_MAX - 14))

    # ---- Indicateurs clés ----
    err = None
    if bearing is not None and heading is not None:
        err = math.degrees(wrap_pi(bearing - math.radians(heading)))

    K_DIST.set_text(fmt(distance, "{:.1f}", " m"))
    K_CAP.set_text(fmt(heading % 360 if heading is not None else None,
                       "{:.0f}", "°"))
    if err is None:
        K_ERR.set_text("--")
        K_ERR.set_color(TEXT)
    else:
        K_ERR.set_text("{:+.1f}°".format(err))
        K_ERR.set_color(ACCENT if abs(err) < 15
                        else AMBER if abs(err) < 45 else RED)

    if pos is None:
        K_ZONE.set_text("--")
        K_ZONE.set_color(TEXT)
    elif point_in_poly(pos[0], pos[1], ZONE_XY):
        K_ZONE.set_text("DANS ZONE")
        K_ZONE.set_color(ACCENT)
    else:
        K_ZONE.set_text("HORS ZONE")
        K_ZONE.set_color(RED)

    K_WP.set_text("{} {}".format(WAYPOINTS[active][0],
                                 "atteint" if reached else "en cours"))
    K_WP.set_color(ACCENT if reached else TEXT)
    K_PTS.set_text(str(len(xs)))

    footer_pos.set_text("LAT {}   LON {}   X/Y {} / {} m".format(
        fmt(latest.get("latitude"), "{:.6f}", "°"),
        fmt(latest.get("longitude"), "{:.6f}", "°"),
        fmt(pos[0] if pos else None, "{:.1f}"),
        fmt(pos[1] if pos else None, "{:.1f}")))


# --------------------------------------------------
# LANCEMENT
# --------------------------------------------------

ani = FuncAnimation(fig, update, interval=100, blit=False,
                    cache_frame_data=False)

fig.canvas.mpl_connect("close_event", lambda _e: link.close())

if __name__ == "__main__":
    try:
        plt.show()
    finally:
        link.close()