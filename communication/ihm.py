"""
IHM DDBoat : tableau de bord de télémétrie en direct.

Affiche : carte avec trajectoire, boussole (cap + direction du waypoint),
puissance des propulseurs, état de la liaison et valeurs numériques.
Lancer :  python3 ihm.py
"""

import json
import math
import socket
import time
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle


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

# Palette
BG = "#0f172a"
PANEL = "#1e293b"
GRID = "#334155"
TEXT = "#e2e8f0"
MUTED = "#94a3b8"
BLUE = "#38bdf8"
GREEN = "#22c55e"
ORANGE = "#f59e0b"
RED = "#ef4444"


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


def style_axes(ax):
    ax.set_facecolor(PANEL)
    for spine in ax.spines.values():
        spine.set_color(GRID)
    ax.tick_params(colors=MUTED)


# --------------------------------------------------
# CONSTRUCTION DE L'INTERFACE
# --------------------------------------------------

plt.rcParams["toolbar"] = "None"
fig = plt.figure(figsize=(14, 8), facecolor=BG)
try:
    fig.canvas.manager.set_window_title("DDBoat - Tableau de bord")
except Exception:
    pass

gs = GridSpec(
    3, 2, figure=fig,
    width_ratios=[2.3, 1],
    height_ratios=[1.25, 0.6, 1.0],
    left=0.05, right=0.97, top=0.85, bottom=0.07,
    wspace=0.15, hspace=0.35,
)

# ---- En-tête ----
fig.text(0.05, 0.955, "DDBoat", color=TEXT, fontsize=22,
         fontweight="bold", va="center")
fig.text(0.05, 0.915, "Télémétrie en direct", color=MUTED,
         fontsize=11, va="center")
status = fig.text(0.97, 0.945, "", color=ORANGE, fontsize=14,
                  fontweight="bold", ha="right", va="center")

# ---- Carte ----
ax_map = fig.add_subplot(gs[:, 0])
style_axes(ax_map)
ax_map.set_title("Carte locale", color=TEXT, loc="left", fontsize=13)
ax_map.set_xlabel("x (m)  -  Est", color=MUTED)
ax_map.set_ylabel("y (m)  -  Nord", color=MUTED)
ax_map.grid(True, color=GRID, linewidth=0.6, alpha=0.7)
ax_map.set_aspect("equal", adjustable="datalim")
ax_map.set_xlim(-10, 10)
ax_map.set_ylim(-10, 10)

traj_line, = ax_map.plot([], [], "-", color=BLUE, lw=2, alpha=0.9,
                         label="Trajectoire", zorder=2)
gps_pts, = ax_map.plot([], [], ".", color=BLUE, ms=4, alpha=0.5, zorder=2)
origin_pt, = ax_map.plot([0], [0], "s", color=MUTED, ms=8,
                         label="Départ", zorder=3)
target_pt, = ax_map.plot([], [], "*", color=ORANGE, ms=18,
                         label="Waypoint", zorder=4)
target_circle = Circle((0, 0), WAYPOINT_TOLERANCE, fill=False,
                       ec=ORANGE, ls="--", lw=1.2, visible=False, zorder=3)
ax_map.add_patch(target_circle)
leg_line, = ax_map.plot([], [], ":", color=ORANGE, lw=1.2, zorder=1)
boat_pt, = ax_map.plot([], [], marker=(3, 0, 0), color=RED, ms=18,
                       ls="None", label="Bateau", zorder=5)
leg = ax_map.legend(loc="lower right", facecolor=BG, edgecolor=GRID,
                    labelcolor=TEXT, fontsize=9)

# ---- Boussole ----
ax_comp = fig.add_subplot(gs[0, 1], projection="polar")
ax_comp.set_facecolor(PANEL)
ax_comp.set_theta_zero_location("N")
ax_comp.set_theta_direction(-1)
ax_comp.set_ylim(0, 1)
ax_comp.set_yticks([])
ax_comp.set_xticks([math.radians(a) for a in range(0, 360, 45)])
ax_comp.set_xticklabels(["N", "NE", "E", "SE", "S", "SO", "O", "NO"],
                        color=TEXT, fontsize=11, fontweight="bold")
ax_comp.grid(True, color=GRID, linewidth=0.8)
ax_comp.spines["polar"].set_color(GRID)
ax_comp.set_title("Cap", color=TEXT, fontsize=13, loc="left", pad=18)

needle = ax_comp.annotate(
    "", xy=(0, 0.88), xytext=(0, 0),
    arrowprops=dict(arrowstyle="-|>", lw=3.5, color=RED,
                    mutation_scale=22, shrinkA=0, shrinkB=0),
)
needle_tail = ax_comp.annotate(
    "", xy=(math.pi, 0.4), xytext=(0, 0),
    arrowprops=dict(arrowstyle="-", lw=2.5, color=MUTED,
                    shrinkA=0, shrinkB=0),
)
bearing_arrow = ax_comp.annotate(
    "", xy=(0, 0.95), xytext=(0, 0),
    arrowprops=dict(arrowstyle="->", lw=2, color=ORANGE, ls="--",
                    mutation_scale=16, shrinkA=0, shrinkB=0),
    visible=False,
)

# ---- Propulseurs ----
ax_mot = fig.add_subplot(gs[1, 1])
style_axes(ax_mot)
ax_mot.set_title("Propulseurs", color=TEXT, fontsize=13, loc="left")
ax_mot.set_xlim(0, PWM_MAX)
ax_mot.set_ylim(-0.6, 1.6)
ax_mot.set_yticks([1, 0])
ax_mot.set_yticklabels(["Gauche", "Droit"], color=TEXT, fontsize=11)
ax_mot.set_xticks([0, 25, 50, 75, 100])
ax_mot.grid(True, axis="x", color=GRID, linewidth=0.6)
ax_mot.set_axisbelow(True)
bars = ax_mot.barh([1, 0], [0, 0], height=0.6, color=[GREEN, GREEN])
mot_txt = [
    ax_mot.text(2, 1, "", color=BG, va="center", fontweight="bold"),
    ax_mot.text(2, 0, "", color=BG, va="center", fontweight="bold"),
]

# ---- Valeurs numériques ----
ax_info = fig.add_subplot(gs[2, 1])
style_axes(ax_info)
ax_info.set_xticks([])
ax_info.set_yticks([])
ax_info.set_title("Mesures", color=TEXT, fontsize=13, loc="left")
info = ax_info.text(0.06, 0.92, "", color=TEXT, family="monospace",
                    fontsize=10.5, va="top", linespacing=1.55,
                    transform=ax_info.transAxes)


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
        status.set_text("●  HORS LIGNE - reconnexion...")
        status.set_color(RED)
    elif link.last_msg == 0 or now - link.last_msg > STALE_AFTER:
        status.set_text("●  CONNECTÉ - en attente de données")
        status.set_color(ORANGE)
    elif latest.get("x") is None:
        status.set_text("●  EN DIRECT - attente fix GPS")
        status.set_color(ORANGE)
    else:
        status.set_text("●  EN DIRECT")
        status.set_color(GREEN)

    if not latest:
        return

    x, y = latest.get("x"), latest.get("y")
    heading = latest.get("heading_deg")
    left = latest.get("motor_left") or 0
    right = latest.get("motor_right") or 0
    distance = latest.get("distance_m")
    reached = bool(latest.get("target_reached"))

    # ---- Carte ----
    traj_line.set_data(xs, ys)
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
        needle.xy = (th, 0.88)
        needle_tail.xy = (th + math.pi, 0.4)
        ax_comp.set_title("Cap   {:05.1f}°".format(heading % 360), color=TEXT, fontsize=13, loc="left", pad=18, fontweight="bold")
    else:
        ax_comp.set_title("Cap   --", color=TEXT, fontsize=13, loc="left", pad=18)

    if bearing is not None:
        bearing_arrow.xy = (bearing, 0.95)
        bearing_arrow.set_visible(True)
    else:
        bearing_arrow.set_visible(False)

    # ---- Propulseurs ----
    for bar, txt, value in zip(bars, mot_txt, (left, right)):
        bar.set_width(max(0, value))
        bar.set_color(GREEN if value < 0.7 * PWM_MAX else ORANGE)
        txt.set_text("{:.0f}".format(value))

    # ---- Valeurs numériques ----
    if bearing is not None and heading is not None:
        err = math.degrees(wrap_pi(bearing - math.radians(heading)))
        err_txt = "{:+.1f}°".format(err)
    else:
        err_txt = "--"

    lines = [
        "Latitude   {}".format(fmt(latest.get("latitude"), "{:.6f}", "°")),
        "Longitude  {}".format(fmt(latest.get("longitude"), "{:.6f}", "°")),
        "X / Y      {} / {} m".format(fmt(x, "{:.1f}"), fmt(y, "{:.1f}")),
        "Cap        {}".format(fmt(heading, "{:.1f}", "°")),
        "Erreur cap {}".format(err_txt),
        "Distance   {}".format(fmt(distance, "{:.1f}", " m")),
        "Points GPS {}".format(len(xs)),
        "Waypoint   {}".format("ATTEINT" if reached else "en cours"),
    ]
    info.set_text("\n".join(lines))
    info.set_color(GREEN if reached else TEXT)


# --------------------------------------------------
# LANCEMENT
# --------------------------------------------------

ani = FuncAnimation(fig, update, interval=100, blit=False,
                    cache_frame_data=False)

fig.canvas.mpl_connect("close_event", lambda _e: link.close())

try:
    plt.show()
finally:
    link.close()