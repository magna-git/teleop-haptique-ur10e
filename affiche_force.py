"""
affiche_force.py : montre en direct la force mesurée sur la pince du UR10e.
À lancer dans un 2e terminal, APRÈS teleop_6axes_v2.py (qui remet le capteur à zéro).
Lecture seule : ce script n'envoie rien au robot.

  - jauge : se remplit et change de couleur (vert, orange, rouge)
  - courbe : force des 10 dernières secondes
Fermer la fenêtre pour quitter.
"""
from collections import deque
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import rtde_receive

ROBOT_IP = "169.254.140.20"  # vrai UR10e  |  URSim : "172.17.0.2"
F_MAX = 80.0                 # N : même limite que MAX_ROBOT_FORCE (arrêt du robot)
F_NOISE = 1.5                # N : en dessous, on affiche 0 (même zone morte que le code)
HISTORY = 10.0               # s : durée de la courbe
REFRESH = 50                 # ms : rafraîchissement (20 images par seconde)

robot = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
t0 = time.monotonic()
times, values = deque(), deque()

fig, (ax_bar, ax_curve) = plt.subplots(2, 1, figsize=(9, 6),
                                       gridspec_kw={"height_ratios": [1, 3]})
fig.suptitle("Force sur la pince", fontsize=18)

# Jauge
ax_bar.set_xlim(0, F_MAX)
ax_bar.set_yticks([])
ax_bar.set_xlabel("Force (N)")
bar = ax_bar.barh([0], [0], color="green")[0]
label = ax_bar.text(F_MAX * 0.98, 0, "0 N", ha="right", va="center", fontsize=22, weight="bold")

# Courbe
ax_curve.set_xlim(-HISTORY, 0)
ax_curve.set_ylim(0, F_MAX)
ax_curve.set_xlabel("Temps (s)")
ax_curve.set_ylabel("Force (N)")
ax_curve.axhspan(F_MAX * 0.5, F_MAX * 0.8, color="orange", alpha=0.1)
ax_curve.axhspan(F_MAX * 0.8, F_MAX, color="red", alpha=0.1)
ax_curve.grid(alpha=0.3)
(line,) = ax_curve.plot([], [], lw=2)


def color(f):
    """Vert jusqu'à 50 % de la limite, orange jusqu'à 80 %, rouge au-delà."""
    if f < 0.5 * F_MAX:
        return "green"
    return "orange" if f < 0.8 * F_MAX else "red"


def update(_):
    now = time.monotonic() - t0
    fx, fy, fz = robot.getActualTCPForce()[:3]           # forces (N), couples ignorés
    f = float(np.linalg.norm([fx, fy, fz]))              # intensité totale de la force
    f = 0.0 if f < F_NOISE else f
    times.append(now)
    values.append(f)
    while times and now - times[0] > HISTORY:            # on garde les 10 dernières secondes
        times.popleft()
        values.popleft()

    bar.set_width(min(f, F_MAX))
    bar.set_color(color(f))
    label.set_text(f"{f:4.1f} N")
    line.set_data([t - now for t in times], list(values))
    line.set_color(color(f))
    return bar, label, line


anim = FuncAnimation(fig, update, interval=REFRESH, blit=False, cache_frame_data=False)
try:
    plt.tight_layout()
    plt.show()
finally:
    robot.disconnect()