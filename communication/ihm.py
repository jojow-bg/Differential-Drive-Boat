"""
IHM DDBoat : tableau de bord de télémétrie en direct (style fintech).

Affiche : indicateurs clés, carte avec trajectoire dégradée, boussole
(cap + direction du waypoint), puissance des propulseurs et état de la liaison.
Lancer :  python3 ihm.py
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
from matplotlib.patches import Circle, FancyBboxPatch


# --------------------------------------------------
# CONFIGURATION
# --------------------------------------------------

SERVER_IP = "172.20.25.202"      # adresse IP du DDBoat
SERVER_PORT = 5002

# Même waypoint que dans nav_to_coords_gps (radians)
TARGET_LAT = np.deg2rad(48.198863)
TARGET_LON = np.deg2rad(-3.016258)
WAYPOINT_TOLERANCE = 2.0         # mètres

PWM_MAX = 100
R_EARTH = 6371000.0
STALE_AFTER = 1.0                # s sans message => liaison perdue
RECONNECT_EVERY = 2.0            # s entre deux tentatives

# Palette fintech
BG = "#080c16"          # fond de la fenêtre
CARD = "#10172a"        # cartes
INNER = "#0b1120"       # zones de tracé
BORDER = "#1f2a44"      # contours
GRID = "#1a2338"
TRACK = "#1b2540"       # fond des jauges
TEXT = "#f1f5f9"
MUTED = "#7c8aa5"
ACCENT = "#00d4aa"      # turquoise (principal)
VIOLET = "#8b5cf6"      # secondaire
AMBER = "#fbbf24"
RED = "#fb7185"

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
# OUTILS
# --------------------------------------------------

def wrap_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def fmt(value, spec="{:.2f}", unit=""):
    if value is None:
        return "--"
    return spec.format(value) + unit


def card(x, y, w, h, color=CARD):
    """Carte arrondie en coordonnées figure."""
    fig.add_artist(FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0,rounding_size=0.012",
        mutation_aspect=FIG_W / FIG_H,
        transform=fig.transFigure,
        fc=color, ec=BORDER, lw=1, zorder=-5))


def kpi_tile(x, y, w, h, label):
    card(x, y, w, h)
    fig.text(x + 0.014, y + h - 0.026, label.upper(), color=MUTED,
             fontsize=8.5, fontweight="bold", va="center")
    return fig.text(x + 0.014, y + 0.027, "--", color=TEXT, fontsize=21,
                    fontweight="bold", va="center")


def style_inner(ax):
    ax.set_facecolor(INNER)
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.tick_params(colors=MUTED, labelsize=8, length=0)


# --------------------------------------------------
# CONSTRUCTION DE L'INTERFACE
# --------------------------------------------------

plt.rcParams["toolbar"] = "None"
fig = plt.figure(figsize=(FIG_W, FIG_H), facecolor=BG)
try:
    fig.canvas.manager.set_window_title("DDBoat - Tableau de bord")
except Exception:
    pass

# ---- En-tête ----
fig.text(0.030, 0.952, "◆", color=ACCENT, fontsize=20, va="center")
fig.text(0.050, 0.960, "DDBoat", color=TEXT, fontsize=20,
         fontweight="bold", va="center")
fig.text(0.050, 0.930, "Télémétrie en direct", color=MUTED,
         fontsize=10, va="center")
status = fig.text(0.970, 0.945, "", color=ACCENT, fontsize=11,
                  fontweight="bold", ha="right", va="center")

# ---- Indicateurs clés ----
n_tiles, gap = 5, 0.012
tile_w = (0.94 - gap * (n_tiles - 1)) / n_tiles
tile_y, tile_h = 0.795, 0.10
kpi_labels = ["Distance", "Cap", "Erreur de cap", "Points GPS", "Waypoint"]
kpis = [kpi_tile(0.03 + i * (tile_w + gap), tile_y, tile_w, tile_h, lab)
        for i, lab in enumerate(kpi_labels)]

# ---- Carte ----
card(0.03, 0.065, 0.585, 0.705)
fig.text(0.045, 0.745, "Carte locale", color=TEXT, fontsize=12,
         fontweight="bold", va="center")
fig.text(0.600, 0.745, "Est / Nord (m)", color=MUTED, fontsize=9,
         ha="right", va="center")

ax_map = fig.add_axes([0.050, 0.095, 0.545, 0.620])
style_inner(ax_map)
ax_map.grid(True, color=GRID, linewidth=0.8)
ax_map.set_axisbelow(True)
ax_map.set_aspect("equal", adjustable="datalim")
ax_map.set_xlim(-10, 10)
ax_map.set_ylim(-10, 10)

# Trajectoire : halo + trait dégradé (le plus récent est le plus vif)
trail_glow = LineCollection([], linewidths=7, zorder=2)
trail = LineCollection([], linewidths=2.5, zorder=3)
ax_map.add_collection(trail_glow)
ax_map.add_collection(trail)
gps_pts, = ax_map.plot([], [], "o", color=ACCENT, ms=3, alpha=0.6, zorder=3)

origin_pt, = ax_map.plot([0], [0], "o", mfc=INNER, mec=MUTED, mew=2, ms=9,
                         label="Départ", zorder=4)
target_circle = Circle((0, 0), WAYPOINT_TOLERANCE, fc=to_rgba(AMBER, 0.10),
                       ec=AMBER, ls="--", lw=1.2, visible=False, zorder=3)
ax_map.add_patch(target_circle)
leg_line, = ax_map.plot([], [], ":", color=AMBER, lw=1.3, alpha=0.8, zorder=1)
target_pt, = ax_map.plot([], [], "*", color=AMBER, ms=17, mec=BG, mew=0.8,
                         label="Waypoint", zorder=5)
boat_pt, = ax_map.plot([], [], marker=(3, 0, 0), color=ACCENT, ms=17,
                       mec=BG, mew=1, ls="None", label="Bateau", zorder=6)
ax_map.legend(loc="lower right", facecolor=CARD, edgecolor=BORDER,
              labelcolor=TEXT, fontsize=9, framealpha=0.95)

# ---- Boussole ----
card(0.635, 0.395, 0.335, 0.375)
fig.text(0.650, 0.745, "Cap", color=TEXT, fontsize=12,
         fontweight="bold", va="center")
comp_value = fig.text(0.955, 0.745, "--", color=ACCENT, fontsize=12,
                      fontweight="bold", ha="right", va="center")

ax_comp = fig.add_axes([0.7285, 0.468, 0.153, 0.260], projection="polar")
ax_comp.set_facecolor(INNER)
ax_comp.set_theta_zero_location("N")
ax_comp.set_theta_direction(-1)
ax_comp.set_ylim(0, 1)
ax_comp.set_yticks([])
ax_comp.set_xticks([math.radians(a) for a in range(0, 360, 45)])
ax_comp.set_xticklabels(["N", "NE", "E", "SE", "S", "SO", "O", "NO"],
                        color=TEXT, fontsize=9, fontweight="bold")
ax_comp.tick_params(axis="x", pad=2)
ax_comp.grid(True, color=GRID, linewidth=0.8)
ax_comp.spines["polar"].set_color(BORDER)
for a in range(0, 360, 10):
    r0 = 0.90 if a % 30 == 0 else 0.94
    ax_comp.plot([math.radians(a)] * 2, [r0, 1.0], color=MUTED,
                 lw=1.0, alpha=0.8)

bearing_arrow = ax_comp.annotate(
    "", xy=(0, 0.95), xytext=(0, 0),
    arrowprops=dict(arrowstyle="->", lw=2, color=AMBER, ls="--",
                    mutation_scale=16, shrinkA=0, shrinkB=0),
    visible=False)
needle = ax_comp.annotate(
    "", xy=(0, 0.82), xytext=(0, 0),
    arrowprops=dict(arrowstyle="-|>", lw=3.5, color=ACCENT,
                    mutation_scale=22, shrinkA=0, shrinkB=0))
needle_tail = ax_comp.annotate(
    "", xy=(math.pi, 0.35), xytext=(0, 0),
    arrowprops=dict(arrowstyle="-", lw=2.5, color=MUTED,
                    shrinkA=0, shrinkB=0))

fig.text(0.700, 0.410, "●", color=ACCENT, fontsize=9, va="center")
fig.text(0.711, 0.410, "Cap bateau", color=MUTED, fontsize=9, va="center")
fig.text(0.805, 0.410, "●", color=AMBER, fontsize=9, va="center")
fig.text(0.816, 0.410, "Direction waypoint", color=MUTED, fontsize=9,
         va="center")

# ---- Propulseurs ----
card(0.635, 0.065, 0.335, 0.31)
fig.text(0.650, 0.350, "Propulseurs", color=TEXT, fontsize=12,
         fontweight="bold", va="center")
fig.text(0.955, 0.350, "PWM 0 - {}".format(PWM_MAX), color=MUTED, fontsize=9,
         ha="right", va="center")

ax_mot = fig.add_axes([0.680, 0.095, 0.270, 0.215])
style_inner(ax_mot)
ax_mot.set_facecolor(CARD)
ax_mot.set_xlim(0, PWM_MAX)
ax_mot.set_ylim(-0.6, 1.6)
ax_mot.set_yticks([1, 0])
ax_mot.set_yticklabels(["Gauche", "Droit"], color=TEXT, fontsize=10.5,
                       fontweight="bold")
ax_mot.set_xticks([0, 25, 50, 75, 100])
ax_mot.barh([1, 0], [PWM_MAX] * 2, height=0.46, color=TRACK, zorder=1)
bars = ax_mot.barh([1, 0], [0, 0], height=0.46, color=[ACCENT, ACCENT],
                   zorder=2)
mot_txt = [
    ax_mot.text(2, 1, "", color=TEXT, va="center", fontweight="bold",
                fontsize=10, zorder=3),
    ax_mot.text(2, 0, "", color=TEXT, va="center", fontweight="bold",
                fontsize=10, zorder=3),
]

# ---- Pied de page ----
footer_pos = fig.text(0.030, 0.030, "", color=MUTED, fontsize=9, va="center",
                      family="monospace")
fig.text(0.970, 0.030, "{}:{}".format(SERVER_IP, SERVER_PORT), color=MUTED,
         fontsize=9, ha="right", va="center", family="monospace")


# --------------------------------------------------
# ÉTAT
# --------------------------------------------------

link = Link(SERVER_IP, SERVER_PORT)
latest = {}
xs, ys = [], []
target_xy = None        # position locale du waypoint (m)


def update_target_from(msg):
    """Déduit l'origine locale du bateau à partir d'un message, puis
    place le waypoint dans le même repère."""
    global target_xy
    lat, lon = msg.get("latitude"), msg.get("longitude")
    x, y = msg.get("x"), msg.get("y")
    if None in (lat, lon, x, y):
        return
    origin_lat = math.radians(lat) - y / R_EARTH
    origin_lon = math.radians(lon) - x / (R_EARTH * math.cos(origin_lat))
    tx = R_EARTH * math.cos(origin_lat) * (TARGET_LON - origin_lon)
    ty = R_EARTH * (TARGET_LAT - origin_lat)
    target_xy = (tx, ty)


def fit_view():
    """Recadre la carte sur trajectoire + bateau + waypoint."""
    px = list(xs) + [0.0]
    py = list(ys) + [0.0]
    if target_xy is not None:
        px.append(target_xy[0])
        py.append(target_xy[1])
    cx = (min(px) + max(px)) / 2
    cy = (min(py) + max(py)) / 2
    half = max(max(px) - min(px), max(py) - min(py)) / 2
    half = max(half * 1.25, 6.0)
    ax_map.set_xlim(cx - half, cx + half)
    ax_map.set_ylim(cy - half, cy + half)


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


# --------------------------------------------------
# MISE À JOUR
# --------------------------------------------------

def update(_):
    global latest

    for msg in link.poll():
        latest = msg
        x, y = msg.get("x"), msg.get("y")
        if msg.get("new_gps") and x is not None and y is not None:
            xs.append(x)
            ys.append(y)
        if target_xy is None:
            update_target_from(msg)

    # ---- Statut de liaison ----
    now = time.monotonic()
    if not link.connected:
        set_status("●  HORS LIGNE - reconnexion...", RED)
    elif link.last_msg == 0 or now - link.last_msg > STALE_AFTER:
        set_status("●  CONNECTÉ - en attente de données", AMBER)
    elif latest.get("x") is None:
        set_status("●  EN DIRECT - attente fix GPS", AMBER)
    else:
        set_status("●  EN DIRECT", ACCENT)

    if not latest:
        return

    x, y = latest.get("x"), latest.get("y")
    heading = latest.get("heading_deg")
    left = latest.get("motor_left") or 0
    right = latest.get("motor_right") or 0
    distance = latest.get("distance_m")
    reached = bool(latest.get("target_reached"))

    # ---- Carte ----
    draw_trail()
    gps_pts.set_data(xs, ys)

    if target_xy is not None:
        target_pt.set_data([target_xy[0]], [target_xy[1]])
        target_circle.center = target_xy
        target_circle.set_visible(True)

    if x is not None and y is not None:
        boat_pt.set_data([x], [y])
        angle = -heading if heading is not None else 0
        boat_pt.set_marker((3, 0, angle))
        if target_xy is not None:
            leg_line.set_data([x, target_xy[0]], [y, target_xy[1]])
    else:
        boat_pt.set_data([], [])
        leg_line.set_data([], [])
    fit_view()

    # ---- Boussole ----
    bearing = None
    if x is not None and target_xy is not None and not reached:
        bearing = math.atan2(target_xy[0] - x, target_xy[1] - y)

    if heading is not None:
        th = math.radians(heading)
        needle.xy = (th, 0.82)
        needle_tail.xy = (th + math.pi, 0.35)
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
        txt.set_x(min(w + 2, PWM_MAX - 10))

    # ---- Indicateurs clés ----
    err = None
    if bearing is not None and heading is not None:
        err = math.degrees(wrap_pi(bearing - math.radians(heading)))

    kpis[0].set_text(fmt(distance, "{:.1f}", " m"))
    kpis[1].set_text(fmt(heading % 360 if heading is not None else None,
                         "{:.0f}", "°"))
    if err is None:
        kpis[2].set_text("--")
        kpis[2].set_color(TEXT)
    else:
        kpis[2].set_text("{:+.1f}°".format(err))
        kpis[2].set_color(ACCENT if abs(err) < 15
                          else AMBER if abs(err) < 45 else RED)
    kpis[3].set_text(str(len(xs)))
    kpis[4].set_text("ATTEINT" if reached else "En cours")
    kpis[4].set_color(ACCENT if reached else TEXT)

    footer_pos.set_text("LAT {}   LON {}   X/Y {} / {} m".format(
        fmt(latest.get("latitude"), "{:.6f}", "°"),
        fmt(latest.get("longitude"), "{:.6f}", "°"),
        fmt(x, "{:.1f}"), fmt(y, "{:.1f}")))


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