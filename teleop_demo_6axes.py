import csv
import ctypes
import curses
import json
import math
import os
import socket
import time

import numpy as np
from scipy.spatial.transform import Rotation
import rtde_control
import rtde_receive


# --- Source et robot ---------------------------------------------------
INPUT = "omega"              # "omega" (labo) ou "clavier" (maison + URSim)
ROBOT_IP = "169.254.140.20"  # vrai UR10e  |  URSim : "172.17.0.2"
LIB_PATH = "/home/user/sdk-3.17.7/lib/release/lin-x86_64-gcc/libdrd.so.3.17.7"

# R traduit un vecteur des axes de l'omega vers les axes du robot.
# Choisir UNE des 4 matrices selon la position de l'opérateur (vue de dessus).
# Ce sont toutes des rotations autour de z : déterminant = +1 (jamais de miroir).
#   0°  (même sens)   : [[ 1, 0, 0], [ 0, 1, 0], [0, 0, 1]]
#   90°               : [[ 0,-1, 0], [ 1, 0, 0], [0, 0, 1]]
#   180° (face à face): [[-1, 0, 0], [ 0,-1, 0], [0, 0, 1]]
#   -90°              : [[ 0, 1, 0], [-1, 0, 0], [0, 0, 1]]
R = np.array([[-1.0,  0.0, 0.0],     # x robot = -x omega
              [ 0.0, -1.0, 0.0],     # y robot = -y omega
              [ 0.0,  0.0, 1.0]])    # z robot = +z omega (ne change jamais)
assert abs(np.linalg.det(R) - 1.0) < 1e-6, "R doit être une rotation (det = +1)"

# --- Mode test ---------------------------------------------------------
# True : seule la torsion du stylo agit (position et inclinaison figées).
#   Test : espace, puis vriller le stylo -> seul le poignet 3 doit tourner.
# False : fonctionnement normal.
MODE_TEST_PIVOT = False

# --- Position ----------------------------------------------------------
SCALE = 16.0                 # NON UTILISÉ : remplacé par les MODES (plus bas)
# Boîte autour du point de départ, en mètres, axes de la base [x, y, z]
# Elle ne sert plus qu'aux obstacles du labo : singularités et table ont leurs propres limites.
BOX_MIN = np.array([-1.000, -1.000, -0.300])   # z (3e valeur) : descente max sous le départ
BOX_MAX = np.array([+1.000, +1.000, +0.800])   # z (3e valeur) : montée max au-dessus du départ
TABLE_Z = None               # m : hauteur de la table dans le repère Base (MESURÉE à la tablette)
                             #   None = pas de limite table (seule BOX_MIN compte)
TABLE_CLEARANCE = 0.030      # m : la pince s'arrête 3 cm au-dessus de la table
# Zone UR10e sans singularité : la pince s'arrête au bord, sans arrêt du programme
WORK_MAX_REACH = 1.10        # m : distance maxi épaule -> pointe (UR10e : 1,30 m à la bride) À MESURER
WORK_MIN_RADIUS = 0.30       # m : distance mini à l'axe de la base (colonne centrale)
SHOULDER_Z = 0.181           # m : hauteur de l'épaule du UR10e (plan UR : 180,7 mm)
SAFETY_MARGIN = 0.005        # au-delà de la boîte + 5 mm : arrêt
SPEED_START = 0.200          # NON UTILISÉ : remplacé par les MODES (plus bas)
SPEED_STEP = 0.020           # + / - : pas de 20 mm/s
SPEED_MIN = 0.010            # 10 mm/s
SPEED_MAX = 0.600            # vitesse la plus haute avec + : 600 mm/s (doit suivre SCALE)
ACCEL_TIME = 0.2             # NON UTILISÉ : remplacé par les MODES (plus bas)
TRACK_MIN = 0.050            # écart consigne / réel toléré : au moins 50 mm
TRACK_LAG = 1.0              # ... ou vitesse x 1 s si c'est plus grand

# --- Inclinaison -------------------------------------------------------
ROT_SCALE = 1.0                               # 1° de stylo -> 1° de pince   #   important 
MAX_ANGLE = math.radians(60)                 # inclinaison max (paliers : 45 -> 60 -> 90)
SAFETY_ANGLE = MAX_ANGLE + math.radians(5)   # au-delà : arrêt (toujours MAX_ANGLE + 5°)
ROT_SPEED_PER_SPEED = math.radians(10) / 0.015   # 10 °/s quand vitesse = 15 mm/s
ROT_SPEED_MAX = math.radians(45)             # plafond : 45 °/s
ROT_TRACK = math.radians(20)                 # écart d'orientation toléré
TILT_DEADZONE = math.radians(5)              # penché < 5° ignoré (petits penchés involontaires)

# --- Pivot (torsion du stylo -> rotation de la bride) -----------------
HANDLE_AXIS = np.array([1.0, 0.0, 0.0])  # axe du stylo = x de l'omega (mesuré : calib_axe_poignee.py)
TWIST_SIGN = +1                          # +1 ou -1 : sens du pivot
TWIST_SCALE = 1.0                        # 1° de vrille -> 1° de pivot
TWIST_DEADZONE = math.radians(2)         # vrille < 2° ignorée (gestes parasites)
SPIN_SPEED = math.radians(30)            # vitesse max du pivot : 30 °/s
SPIN_MAX = math.radians(170)             # butée : +/- 170°

# --- Retour d'effort ---------------------------------------------------
K_FB = 0.3                   # 10 N sur la pince -> 3 N dans la main (paliers : 0.2 -> 0.3 -> 0.4)
FORCE_SIGN = +1              # à régler avec test_capteur.py
F_DEADZONE = 1.5             # N : en dessous, c'est du bruit
F_ALPHA = 0.1                # filtre passe-bas (petit = très lissé)
F_MAX_HAND = 6.0             # N : force max dans la main (omega : 12 N max)
F_RAMP_TIME = 1.0            # s : montée progressive du gain
MAX_ROBOT_FORCE = 80.0       # N : au-delà, arrêt
PULSE_FORCE = 2.0            # N : vibration « objet saisi »
PULSE_TIME = 0.15            # s : durée de la vibration
PULSE_HZ = 25                # Hz : fréquence de la vibration

# --- Pince Robotiq -----------------------------------------------------
GRIPPER_PORT = 63352         # port de l'URCap Robotiq
GRIPPER_SPEED = 30           # 0-255 : vitesse des doigts
GRIPPER_FORCE = 30           # 0-255 : force de serrage
GRIPPER_POLL = 0.25          # s : lecture de l'état pendant que les doigts bougent

# --- Boucle et sécurité du robot --------------------------------------
DT = 0.01                    # période de la boucle : 10 ms (100 Hz)
LOOKAHEAD = 0.05             # lissage de servoL (s) : 0,1 = doux mais en retard, 0,05 = plus réactif
GAIN = 600                   # raideur de servoL (validé au labo)
SING_MARGIN = math.radians(15)   # distance mini aux singularités (coude, poignet 2)
MIN_RADIUS = 0.20            # m : distance mini de la pointe à l'axe de la base
JOINT_SPEED_SOFT = 1.0       # rad/s : au-delà, le robot ralentit tout seul
MAX_JOINT_SPEED = 2.0        # rad/s : au-delà, arrêt
MAX_LOOP_DELAY = 0.05        # s : boucle plus lente que ça -> arrêt

# --- Clavier (main virtuelle) -----------------------------------------
KEY_STEP = 0.002             # 2 mm par appui
KEY_ROT = math.radians(2)    # 2° par appui
HAND_LIMIT = 0.08            # m : course max de la main virtuelle

# --- Modes de démo (touches 1, 2, 3 pendant la démo) ------------------
# scale : 1 cm de main -> scale cm de pince | speed : vitesse au choix du mode (m/s)
# speed_max : plafond de la touche + dans ce mode (<= SPEED_MAX) | accel_time : vivacité (s)
MODES = {
    ord('1'): dict(name="PRÉCISION", scale=4.0, speed=0.060, speed_max=0.150, accel_time=0.4),
    ord('2'): dict(name="RAPIDE", scale=7.0, speed=0.300, speed_max=0.500, accel_time=0.3),
    ord('3'): dict(name="GRANDE AMPLITUDE", scale=14.0, speed=0.400, speed_max=0.600, accel_time=0.3),
}
START_MODE = ord('1')        # mode au lancement : précision (le plus prudent)

# --- Positions de départ ----------------------------------------------
HOLD_HANDLE = True           # poignée omega tenue au centre tant que le suivi est OFF
# Position de départ du bras, en degrés [base, épaule, coude, poignet 1, poignet 2, poignet 3]
# None = le bras reste où il est. Exemple classique UR : [0, -90, 90, -90, -90, 0]
Q_HOME_DEG = [-5.5, -90.6, -90.6, -265.2, -91.7, -8.5]   # relevé au labo (lire_position.py)
HOME_SPEED = 0.3             # rad/s : vitesse du trajet vers la position de départ (lente)
HOME_ACCEL = 0.3             # rad/s² : accélération de ce trajet
HANDLE_SPEED = 0.05          # m/s : vitesse de la poignée quand elle revient au centre (douce)
HANDLE_ACCEL = 0.2           # m/s² : accélération de ce retour

# --- Enregistrement ----------------------------------------------------
LOG_DIR = "logs"


# =====================================================================
# 2. LA MAIN : OMEGA.6 OU CLAVIER
# Les deux classes ont les mêmes méthodes (position, orientation, button,
# key, enable_force, set_force, close) : le programme principal ne sait
# pas laquelle il utilise.
# =====================================================================
class OmegaHand:
    """Vraie poignée omega.6, lue avec le SDK Force Dimension (librairie C, via ctypes)."""

    def __init__(self):
        fd = self.fd = ctypes.CDLL(LIB_PATH)
        fd.drdOpen.restype = ctypes.c_int
        fd.drdIsInitialized.argtypes = [ctypes.c_int]
        fd.drdIsInitialized.restype = ctypes.c_bool
        fd.drdClose.argtypes = [ctypes.c_int]
        fd.dhdGetPosition.argtypes = [ctypes.POINTER(ctypes.c_double)] * 3 + [ctypes.c_int]
        fd.dhdGetPosition.restype = ctypes.c_int
        self.Mat = (ctypes.c_double * 3) * 3
        fd.dhdGetOrientationFrame.argtypes = [ctypes.POINTER(self.Mat), ctypes.c_int]
        fd.dhdGetOrientationFrame.restype = ctypes.c_int
        fd.dhdGetButton.argtypes = [ctypes.c_int, ctypes.c_int]
        fd.dhdGetButton.restype = ctypes.c_int
        fd.dhdSetForce.argtypes = [ctypes.c_double] * 3 + [ctypes.c_int]
        fd.dhdSetForce.restype = ctypes.c_int
        fd.dhdEnableForce.argtypes = [ctypes.c_ubyte, ctypes.c_int]
        fd.dhdEnableForce.restype = ctypes.c_int
        self.device = fd.drdOpen()
        if self.device < 0:
            raise RuntimeError("Omega.6 non détecté (HapticDesk est-il fermé ?)")
        if not fd.drdIsInitialized(self.device):
            raise RuntimeError("Omega.6 non calibré")
        fd.drdStart.argtypes = [ctypes.c_int]
        fd.drdStart.restype = ctypes.c_int
        fd.drdMoveToPos.argtypes = [ctypes.c_double] * 3 + [ctypes.c_bool, ctypes.c_int]
        fd.drdMoveToPos.restype = ctypes.c_int
        fd.drdStop.argtypes = [ctypes.c_bool, ctypes.c_int]
        fd.drdStop.restype = ctypes.c_int
        fd.drdSetPosMoveParam.argtypes = [ctypes.c_double] * 3 + [ctypes.c_int]
        fd.drdSetPosMoveParam.restype = ctypes.c_int
        self.holding = False                   # poignée tenue au centre par le contrôleur ?

    def position(self):
        """Position de la poignée (m), axes de l'omega."""
        x, y, z = ctypes.c_double(), ctypes.c_double(), ctypes.c_double()
        if self.fd.dhdGetPosition(ctypes.byref(x), ctypes.byref(y), ctypes.byref(z),
                                  self.device) < 0:
            raise RuntimeError("Erreur lecture position omega.6")
        p = np.array([x.value, y.value, z.value])
        if not np.all(np.isfinite(p)):
            raise RuntimeError("Position omega.6 invalide")
        return p

    def orientation(self):
        """Orientation du stylo : matrice 3x3 du SDK, rangée dans un objet Rotation."""
        m = self.Mat()
        if self.fd.dhdGetOrientationFrame(ctypes.byref(m), self.device) < 0:
            raise RuntimeError("Erreur lecture orientation omega.6")
        mat = np.array([[m[i][j] for j in range(3)] for i in range(3)])
        if not np.all(np.isfinite(mat)):
            raise RuntimeError("Orientation omega.6 invalide")
        return Rotation.from_matrix(mat)

    def button(self):
        v = self.fd.dhdGetButton(0, self.device)
        if v < 0:
            raise RuntimeError("Erreur lecture bouton")
        return v == 1

    def key(self, k):
        pass                                   # l'omega n'utilise pas le clavier

    def hold_center(self):
        """Le contrôleur de l'omega ramène la poignée au centre (0, 0, 0) et l'y maintient,
        comme HapticDesk. Pendant ce temps, set_force n'envoie rien."""
        if not self.holding:
            if self.fd.drdStart(self.device) < 0:
                raise RuntimeError("Omega.6 : démarrage du contrôleur impossible")
            # retour doux : accélération, vitesse max, jerk (variation d'accélération)
            self.fd.drdSetPosMoveParam(HANDLE_ACCEL, HANDLE_SPEED, 10 * HANDLE_ACCEL, self.device)
            self.fd.drdMoveToPos(0.0, 0.0, 0.0, False, self.device)   # False : ne bloque pas la boucle
            self.holding = True

    def release(self):
        """Rend la poignée libre (forces actives) pour que l'utilisateur la déplace."""
        if self.holding:
            self.fd.drdStop(True, self.device)                       # True : garde les forces actives
            self.holding = False

    def enable_force(self, on):
        self.fd.dhdEnableForce(1 if on else 0, self.device)

    def set_force(self, f):
        """Force (N) dans les axes de l'omega ; la gravité est compensée par le SDK."""
        if self.holding:
            return                             # le contrôleur tient la poignée : on n'interfère pas
        if self.fd.dhdSetForce(float(f[0]), float(f[1]), float(f[2]), self.device) < 0:
            raise RuntimeError("Erreur envoi de force à l'omega.6")

    def close(self):
        if self.device >= 0:
            try:
                self.release()
                self.fd.dhdSetForce(0.0, 0.0, 0.0, self.device)   # plus aucune force
                self.fd.dhdEnableForce(0, self.device)
            finally:
                self.fd.drdClose(self.device)


class KeyboardHand:
    """Main virtuelle au clavier, pour tester à la maison avec URSim.
    La vrille du stylo virtuel est autour de HANDLE_AXIS (x) : touches i / k."""
    MOVES = {ord('z'): (0, -1), ord('s'): (0, +1), ord('q'): (1, +1),
             ord('d'): (1, -1), ord('r'): (2, +1), ord('f'): (2, -1)}
    TURNS = {ord('i'): (0, +1), ord('k'): (0, -1), ord('j'): (1, +1),
             ord('l'): (1, -1), ord('u'): (2, +1), ord('o'): (2, -1)}

    def __init__(self):
        self.p = np.zeros(3)
        self.rot = Rotation.identity()
        self.pressed = False

    def position(self):
        return self.p.copy()

    def orientation(self):
        return self.rot

    def button(self):
        return self.pressed

    def key(self, k):
        if k == ord('b'):
            self.pressed = not self.pressed
        elif k in self.MOVES:
            a, s = self.MOVES[k]
            self.p[a] = float(np.clip(self.p[a] + s * KEY_STEP, -HAND_LIMIT, HAND_LIMIT))
        elif k in self.TURNS:
            a, s = self.TURNS[k]
            axis = np.zeros(3)
            axis[a] = s * KEY_ROT
            self.rot = self.rot * Rotation.from_rotvec(axis)   # à droite : axes du stylo virtuel

    def enable_force(self, on):
        pass

    def set_force(self, f):
        pass

    def hold_center(self):
        pass

    def release(self):
        pass

    def close(self):
        pass


# =====================================================================
# 3. LA PINCE ROBOTIQ
# Ordres texte envoyés à l'URCap Robotiq (socket TCP, port 63352).
# La pince doit être ACTIVÉE avant (robotiq_clavier.py) : état « STA 3 ».
# =====================================================================
class Gripper:

    def __init__(self, ip):
        self.sock = socket.create_connection((ip, GRIPPER_PORT), timeout=1.0)
        if self.ask("GET STA") != "STA 3":
            raise RuntimeError("Pince non activée : lance d'abord robotiq_clavier.py")
        self.set(f"SET SPE {GRIPPER_SPEED}")
        self.set(f"SET FOR {GRIPPER_FORCE}")
        self.set("SET GTO 1")                  # la pince suivra chaque nouvelle position
        self.closed = False
        self.obj = 3

    def ask(self, text):
        self.sock.sendall((text + "\n").encode("ascii"))
        return self.sock.recv(1024).decode("ascii").strip()

    def set(self, text):
        if self.ask(text).lower() != "ack":
            raise RuntimeError(f"Pince : commande non confirmée ({text})")

    def grip(self, closed):
        self.closed = closed
        self.set("SET POS 255" if closed else "SET POS 0")   # 255 = fermée, 0 = ouverte
        self.set("SET GTO 1")                                 # « vas-y »

    def toggle(self):
        self.grip(not self.closed)
        self.obj = 0                           # doigts en mouvement

    def object_state(self):
        """OBJ 0 : en mouvement | 1 : objet en ouvrant | 2 : OBJET SAISI
        | 3 : position atteinte, pas d'objet."""
        r = self.ask("GET OBJ")
        self.obj = int(r.split()[-1]) if r.startswith("OBJ") else self.obj
        return self.obj

    def close(self):
        self.sock.close()                      # la pince garde sa position


# =====================================================================
# 4. SÉCURITÉ
# Chaque fonction vérifie un risque et lève une exception s'il est présent.
# Toute exception mène à l'arrêt propre (bloc finally de la section 8).
# =====================================================================
JOINT_NAMES = ["base", "épaule", "coude", "poignet 1", "poignet 2", "poignet 3"]


def valid_pose(pose):
    """Une pose valide = 6 nombres finis."""
    return len(pose) == 6 and all(math.isfinite(v) for v in pose)


def check_robot(robot):
    if not robot.isConnected():
        raise RuntimeError("Connexion robot perdue")
    if robot.isEmergencyStopped():
        raise RuntimeError("Arrêt d'urgence détecté")
    if robot.isProtectiveStopped():
        raise RuntimeError("Arrêt de protection détecté")


def check_singularities(q, tcp):
    """Coude ou poignet 2 près de 0°/180°, ou pointe près de l'axe de la base -> arrêt."""
    if abs(math.sin(q[2])) < math.sin(SING_MARGIN):
        raise RuntimeError(f"Coude proche d'une singularité ({math.degrees(q[2]):.0f}°)")
    if abs(math.sin(q[4])) < math.sin(SING_MARGIN):
        raise RuntimeError(f"Poignet 2 proche d'une singularité ({math.degrees(q[4]):.0f}°)")
    if math.hypot(tcp[0], tcp[1]) < MIN_RADIUS:
        raise RuntimeError("Pointe trop proche de l'axe de la base (singularité d'épaule)")


def check_joint_speeds(qd):
    i = max(range(6), key=lambda j: abs(qd[j]))          # l'articulation la plus rapide
    if abs(qd[i]) > MAX_JOINT_SPEED:
        raise RuntimeError(f"Articulation trop rapide : {JOINT_NAMES[i]} "
                           f"({abs(qd[i]):.2f} rad/s) : singularité probable")


def slow_factor(qd):
    """Ralentissement automatique : 1.0 si tout va bien, moins si une articulation
    dépasse JOINT_SPEED_SOFT (ex. 1,5 rad/s -> facteur 0,67)."""
    fastest = max(abs(v) for v in qd)
    return min(1.0, JOINT_SPEED_SOFT / fastest) if fastest > 1e-6 else 1.0


def final_check(target, start_pose, start_rot):
    """Dernière barrière avant l'envoi : boîte + marge, inclinaison < SAFETY_ANGLE."""
    if not valid_pose(target):
        raise RuntimeError("Cible invalide : rien n'est envoyé")
    for i in range(3):
        if (target[i] < start_box_min[i] - SAFETY_MARGIN
                or target[i] > start_box_max[i] + SAFETY_MARGIN):
            raise RuntimeError(f"Cible hors limites sur {'xyz'[i]} : rien n'est envoyé")
    angle = (start_rot.inv() * Rotation.from_rotvec(target[3:])).magnitude()
    if angle > SAFETY_ANGLE:
        raise RuntimeError("Rotation hors limites : rien n'est envoyé")


def check_tracking(actual_pose, target_pos, target_rot, track_limit):
    """Le robot suit-il la consigne ? Sinon il est bloqué ou a heurté quelque chose."""
    error = np.linalg.norm(np.array(actual_pose[:3]) - target_pos)
    if error > track_limit:
        raise RuntimeError(f"Écart de suivi trop grand : {error * 1000:.1f} mm")
    rot_error = (Rotation.from_rotvec(actual_pose[3:]).inv() * target_rot).magnitude()
    if rot_error > ROT_TRACK:
        raise RuntimeError(f"Écart d'orientation trop grand : {math.degrees(rot_error):.1f}°")
    return error, rot_error


def limit_rotation(rot, max_angle):
    """Ramène une rotation à max_angle au plus (même axe)."""
    rv = rot.as_rotvec()
    a = np.linalg.norm(rv)
    return Rotation.from_rotvec(rv / a * max_angle) if a > max_angle else rot


def continuous_rotvec(rv, prev):
    """Près de 180°, une orientation a deux vecteurs de rotation possibles.
    On garde le plus proche du précédent, sinon servoL fait tourner les poignets brutalement."""
    a = np.linalg.norm(rv)
    if a > 1e-9:
        alt = rv * (1.0 - 2.0 * math.pi / a)
        if np.linalg.norm(alt - prev) < np.linalg.norm(rv - prev):
            return alt
    return rv


# =====================================================================
# 5. CALCULS DU MOUVEMENT
# Aucune de ces fonctions ne parle au robot ni à l'omega : elles calculent.
# =====================================================================

# --- 5a. Position ------------------------------------------------------
def compute_position(cmd, vel, goal, speed, accel, slow):
    """Avance la consigne cmd vers goal, sans dépasser speed ni accel.
    Profil trapézoïdal : vitesse max, freinage v = sqrt(2·a·d), jamais au-delà de la cible."""
    err = goal - cmd
    dist = np.linalg.norm(err)
    v_des = (err / dist * min(speed, math.sqrt(2 * accel * dist), dist / DT)
             if dist > 1e-6 else np.zeros(3))
    dv = v_des - vel
    n = np.linalg.norm(dv)
    if n > accel * DT:                         # accélération bornée
        dv *= accel * DT / n
    vel = (vel + dv) * slow                    # ralentissement automatique
    cmd = np.clip(cmd + vel * DT, start_box_min, start_box_max)
    return cmd, vel


def clip_workspace(p):
    """Ramène une position dans la zone sans singularité du UR10e, au lieu d'arrêter le robot :
    sphère max autour de l'épaule (coude) et colonne interdite autour de la base (épaule)."""
    p = p.copy()
    shoulder = np.array([0.0, 0.0, SHOULDER_Z])
    v = p - shoulder
    d = np.linalg.norm(v)
    if d > WORK_MAX_REACH:                       # bras presque tendu : singularité du coude
        p = shoulder + v * WORK_MAX_REACH / d
    r = math.hypot(p[0], p[1])
    if r < WORK_MIN_RADIUS:                      # trop près de l'axe : singularité d'épaule
        p[:2] = p[:2] * WORK_MIN_RADIUS / r if r > 1e-6 else [WORK_MIN_RADIUS, 0.0]
    return p


# --- 5b. Séparer le geste du stylo : pencher (swing) + vriller (twist) --
def split_twist(rot, axis):
    """Décomposition swing-twist avec un quaternion : rot = swing x twist.
    twist = rotation autour de axis seulement, swing = le reste (pencher).
    Renvoie (swing, angle du twist en rad)."""
    x, y, z, w = rot.as_quat()                 # scipy : [x, y, z, w]
    proj = float(np.dot([x, y, z], axis))      # part de la rotation autour de axis
    angle = 2.0 * math.atan2(proj, w)
    angle = (angle + math.pi) % (2 * math.pi) - math.pi   # entre -180° et +180°
    twist = Rotation.from_rotvec(np.asarray(axis) * angle)
    swing = rot * twist.inv()                  # on retire la vrille, à droite (côté stylo)
    return swing, angle


def split_hand_gesture(r_hand, hand_rot_ref):
    """Geste de la main depuis l'appui sur espace, coupé en deux :
    - le pencher, ré-exprimé dans les axes fixes de l'omega ;
    - l'angle de vrille autour de l'axe du stylo."""
    d_local = hand_rot_ref.inv() * r_hand                    # geste vu depuis le stylo
    swing, twist_angle = split_twist(d_local, HANDLE_AXIS)
    d_tilt = hand_rot_ref * swing * hand_rot_ref.inv()       # pencher, axes fixes de l'omega
    return d_tilt, twist_angle


# --- 5c. Inclinaison ---------------------------------------------------
def compute_tilt(cmd_rot, cmd_rot_ref, d_tilt, rot_speed):
    """La pince se penche comme le stylo (sans la vrille), limitée en angle et en vitesse."""
    a = d_tilt.magnitude()                                         # zone morte du penché
    if a < TILT_DEADZONE:
        d_tilt = Rotation.identity()
    else:
        d_tilt = Rotation.from_rotvec(d_tilt.as_rotvec() * (a - TILT_DEADZONE) / a)
    d_robot = Rotation.from_matrix(R @ d_tilt.as_matrix() @ R.T)   # axes omega -> axes robot
    d_robot = Rotation.from_rotvec(ROT_SCALE * d_robot.as_rotvec())
    goal_rot = limit_rotation(d_robot * cmd_rot_ref, MAX_ANGLE)    # à gauche : axes de la base
    step = goal_rot * cmd_rot.inv()                                # reste à tourner
    a = step.magnitude()
    if a > rot_speed * DT:                                         # vitesse angulaire limitée
        step = Rotation.from_rotvec(step.as_rotvec() / a * rot_speed * DT)
    return limit_rotation(step * cmd_rot, MAX_ANGLE)


# --- 5d. Pivot ---------------------------------------------------------
def pivot_goal(spin_ref, twist_angle):
    """Angle de pivot voulu : pivot au moment de l'espace + vrille du stylo.
    Zone morte : une vrille < TWIST_DEADZONE est ignorée, sans saut au-delà."""
    if abs(twist_angle) < TWIST_DEADZONE:
        twist_used = 0.0
    else:
        twist_used = math.copysign(abs(twist_angle) - TWIST_DEADZONE, twist_angle)
    goal = spin_ref + TWIST_SIGN * TWIST_SCALE * twist_used
    return max(-SPIN_MAX, min(SPIN_MAX, goal))               # butée +/- SPIN_MAX


def compute_pivot(spin, goal_spin, slow):
    """Avance le pivot vers goal_spin, au plus de SPIN_SPEED (réduite par slow)."""
    max_step = SPIN_SPEED * slow * DT
    return spin + max(-max_step, min(max_step, goal_spin - spin))


# --- 5e. Construire la cible : méthode « bride » ------------------------
def build_target(cmd, tcp_rot, spin, tcp_p, tcp_r):
    """Pose du TCP avec pivot autour de l'axe du poignet 3 (axe z de la bride).
    T_tcp = T_bride x Rz(spin) x T_offset. Rien ne bouge : c'est un calcul.
      1. remonter de la pointe (TCP) à la bride ;
      2. tourner la bride de spin autour de SON axe z (à droite) ;
      3. redescendre de la bride à la pointe."""
    flange_rot = tcp_rot * tcp_r.inv()                       # 1. orientation de la bride
    flange_pos = cmd - flange_rot.apply(tcp_p)               #    position de la bride
    spun_rot = flange_rot * Rotation.from_rotvec([0.0, 0.0, spin])   # 2. pivot du poignet 3
    target_rot = spun_rot * tcp_r                            # 3. orientation du TCP
    target_pos = flange_pos + spun_rot.apply(tcp_p)          #    position du TCP
    return target_pos, target_rot


# =====================================================================
# 6. RETOUR D'EFFORT
# f_main = FORCE_SIGN x gain x R^T x f_robot  (filtrée, zone morte, saturée)
# =====================================================================
def force_feedback(f_robot, f_filt, fb_gain, enabled):
    """Renvoie (force pour la main, force filtrée, gain appliqué)."""
    if np.linalg.norm(f_robot) > MAX_ROBOT_FORCE:
        raise RuntimeError(f"Force sur la pince trop grande : {np.linalg.norm(f_robot):.0f} N")
    f_filt = F_ALPHA * f_robot + (1 - F_ALPHA) * f_filt          # 1. filtre passe-bas
    n = np.linalg.norm(f_filt)
    f_used = np.zeros(3) if n < F_DEADZONE else f_filt * (n - F_DEADZONE) / n   # 2. zone morte
    target_gain = K_FB if enabled else 0.0
    step_g = K_FB * DT / F_RAMP_TIME                              # 3. rampe de gain
    fb_gain += max(-step_g, min(step_g, target_gain - fb_gain))
    f_hand = FORCE_SIGN * fb_gain * (R.T @ f_used)                # 4. axes robot -> axes omega
    n = np.linalg.norm(f_hand)
    if n > F_MAX_HAND:                                            # 5. saturation
        f_hand = f_hand * F_MAX_HAND / n
    return f_hand, f_filt, fb_gain


def grasp_pulse(now, pulse_start):
    """Vibration « objet saisi » : signal carré sur z, à PULSE_HZ."""
    side = 1.0 if int((now - pulse_start) * PULSE_HZ * 2) % 2 == 0 else -1.0
    return np.array([0.0, 0.0, side * PULSE_FORCE])


# =====================================================================
# 7. AFFICHAGE (terminal, avec curses)
# =====================================================================
def draw(screen, lines):
    screen.erase()
    height, width = screen.getmaxyx()
    for i, line in enumerate(lines[:height]):
        screen.addstr(i, 0, line[:width - 1])
    screen.refresh()


def screen_lines(s):
    """Texte affiché ; s est un dictionnaire avec les valeurs du cycle."""
    dc, dr = (s["cmd"] - s["start"]) * 1000, (s["actual"] - s["start"]) * 1000
    g = s["gripper"]
    pince = "—" if g is None else ("FERMÉE" if g.closed else "ouverte")
    if g is not None:
        pince += {0: " (en mouvement)", 2: " - OBJET SAISI", 1: " - objet"}.get(g.obj, "")
    return [
        f">>> MODE {s['mode']} (touches 1 / 2 / 3) | échelle ×{s['scale']:.0f} <<<",
        f"TÉLÉOP 6 AXES | {INPUT} | {ROBOT_IP} | log : {s['log_name']}"
        + ("  | MODE TEST PIVOT" if MODE_TEST_PIVOT else ""),
        f"Vitesse : {s['speed']*1000:3.0f} mm/s, {math.degrees(s['rot_speed']):3.0f} °/s (+/-)"
        f" | Inclinaison : {'ON ' if s['rot_on'] else 'OFF'} (t)",
        f"Suivi : {'ON  (suit)' if s['active'] else 'OFF (tient)'} (espace)"
        f" | Retour d'effort : {'ON ' if s['fb_on'] else 'OFF'} (g), gain {s['fb_gain']:.2f}",
        f"Force pince (N) : {s['f_filt'][0]:+5.1f} {s['f_filt'][1]:+5.1f} {s['f_filt'][2]:+5.1f}"
        f" | main (N) : {s['f_hand'][0]:+4.1f} {s['f_hand'][1]:+4.1f} {s['f_hand'][2]:+4.1f}",
        "",
        f"Cible (mm) : x {dc[0]:+6.1f}  y {dc[1]:+6.1f}  z {dc[2]:+6.1f}",
        f"Réel  (mm) : x {dr[0]:+6.1f}  y {dr[1]:+6.1f}  z {dr[2]:+6.1f}",
        f"Écart : {s['error']*1000:4.1f} mm (max {s['track_limit']*1000:.0f})",
        f"Inclinaison : cible {s['ang_cmd']:4.1f}°  écart {math.degrees(s['rot_error']):4.1f}°"
        f" (max {math.degrees(MAX_ANGLE):.0f}°)",
        f"Vrille stylo : {math.degrees(s['twist']):+6.1f}° | Pivot pince : {math.degrees(s['spin']):+6.1f}°"
        f" (max ±{math.degrees(SPIN_MAX):.0f}°)",
        f"Pince : {pince} (bouton omega : 1 appui = fermer / ouvrir)",
        "",
        s["note"],
        "1 précision | 2 rapide | 3 grande amplitude | espace : suivi | bouton : pince",
        "t : inclinaison | g : retour d'effort | + - : vitesse | p : pince | x : quitter",
    ]


# =====================================================================
# 8. PROGRAMME PRINCIPAL (boucle 100 Hz)
# =====================================================================
start_box_min = start_box_max = None          # boîte absolue, fixée au démarrage


def main(screen):
    global start_box_min, start_box_max
    screen.nodelay(True)                       # lecture du clavier non bloquante
    hand = robot = control = log_file = gripper = None
    message = "Arrêt demandé (x)."
    try:
        # --- 8a. Connexions et vérifications de départ -------------------
        hand = OmegaHand() if INPUT == "omega" else KeyboardHand()
        robot = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        check_robot(robot)
        if Q_HOME_DEG is not None:             # trajet lent vers la position de départ choisie
            q_home = [math.radians(a) for a in Q_HOME_DEG]
            check_singularities(q_home, [1.0, 0.0, 0.0])    # angles sûrs (le rayon est vérifié après)
            home = rtde_control.RTDEControlInterface(ROBOT_IP)
            home.moveJ(q_home, HOME_SPEED, HOME_ACCEL)      # bloque jusqu'à l'arrivée
            home.stopScript()
            home.disconnect()
        start_pose = robot.getActualTCPPose()
        if not valid_pose(start_pose):
            raise RuntimeError("Pose TCP invalide")
        check_singularities(robot.getActualQ(), start_pose)
        q5_start = math.degrees(robot.getActualQ()[5])           # poignet 3 au départ
        if abs(q5_start) + math.degrees(SPIN_MAX) > 355:
            raise RuntimeError(f"Poignet 3 trop près de sa butée ({q5_start:.0f}°) : "
                               "ramène-le vers 0° avant de lancer")
        if hand.button():
            raise RuntimeError("Relâche le bouton avant le démarrage")
        gripper = Gripper(ROBOT_IP) if INPUT == "omega" else None
        control = rtde_control.RTDEControlInterface(ROBOT_IP, 1.0 / DT)   # boucle à 100 Hz
        control.zeroFtSensor()                 # tare : robot immobile, pince sans contact
        try:
            off = control.getTCPOffset()       # TCP vu depuis la bride [x, y, z, rx, ry, rz]
        except Exception:
            raise RuntimeError("Lecture du TCP impossible (getTCPOffset)")
        if not valid_pose(off):
            raise RuntimeError("TCP invalide")
        tcp_p = np.array(off[:3])              # position du TCP dans le repère de la bride
        tcp_r = Rotation.from_rotvec(off[3:])  # orientation du TCP par rapport à la bride
        hand.enable_force(True)
        if HOLD_HANDLE:
            hand.hold_center()                 # la poignée commence au centre

        # --- 8b. Log ------------------------------------------------------
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(os.path.join(LOG_DIR, "boite.json"), "w") as f:   # pour visu_ur10e.py
            json.dump({"centre": list(start_pose[:3]),
                       "min": list(BOX_MIN), "max": list(BOX_MAX)}, f)
        log_name = os.path.join(LOG_DIR, time.strftime("essai_%Y%m%d_%H%M%S.csv"))
        log_file = open(log_name, "w", newline="")
        log = csv.writer(log_file)
        log.writerow(["t", "bouton", "suivi", "inclinaison", "vitesse_max",
                      "main_x", "main_y", "main_z",
                      "cible_x", "cible_y", "cible_z", "reel_x", "reel_y", "reel_z",
                      "angle_cible_deg", "ecart_mm", "ecart_rot_deg", "vrille_deg", "pivot_deg",
                      *[f"q{i}_deg" for i in range(6)], *[f"qd{i}" for i in range(6)]])

        # --- 8c. État de départ -------------------------------------------
        start = np.array(start_pose[:3])
        start_rot = Rotation.from_rotvec(start_pose[3:])
        start_box_min, start_box_max = start + BOX_MIN, start + BOX_MAX
        if TABLE_Z is not None:                       # limite basse fixe : la table
            start_box_min[2] = TABLE_Z + TABLE_CLEARANCE
            if start[2] < start_box_min[2]:
                raise RuntimeError("Pince déjà trop près de la table au démarrage")
        if np.linalg.norm(clip_workspace(start) - start) > 1e-6:   # départ hors zone UR10e ?
            raise RuntimeError("Position de départ hors de la zone de travail "
                               "(trop loin ou trop près de l'axe) : change Q_HOME_DEG "
                               "ou WORK_MAX_REACH / WORK_MIN_RADIUS")
        cmd, vel = start.copy(), np.zeros(3)   # consigne de position et sa vitesse
        cmd_rot = Rotation.identity()          # inclinaison commandée, relative au départ
        spin = 0.0                             # pivot commandé (rad)
        twist = 0.0                            # vrille actuelle du stylo (rad)
        prev_rv = np.array(start_pose[3:])     # dernier vecteur de rotation envoyé
        hand_ref = robot_ref = None            # photos prises à l'appui sur espace
        hand_rot_ref = cmd_rot_ref = None
        spin_ref = 0.0
        follow = active = False                # suivi demandé / suivi en cours
        rot_on = fb_on = True
        f_filt, f_hand, fb_gain = np.zeros(3), np.zeros(3), 0.0
        was_pressed = False
        last_poll = 0.0
        pulse_start = pulse_until = -1.0
        note = "MODE TEST PIVOT : seule la vrille du stylo agit" if MODE_TEST_PIVOT else ""
        mode = MODES[START_MODE]               # mode de démo actif
        scale, speed = mode["scale"], mode["speed"]
        speed_max, accel_time = mode["speed_max"], mode["accel_time"]
        rebase = False                         # nouvelle photo de la main demandée (changement de mode)
        t0 = previous = last_draw = time.monotonic()

        while True:
            # --- 8d. Rythme 100 Hz et surveillance du retard --------------
            t_cycle = control.initPeriod()
            now = time.monotonic()
            if now - previous > MAX_LOOP_DELAY:
                raise RuntimeError("Retard excessif de la boucle")
            previous = now

            # --- 8e. Clavier ----------------------------------------------
            k = screen.getch()
            while k != -1:
                if k in MODES:                            # changement de mode de démo
                    mode = MODES[k]
                    scale, speed = mode["scale"], mode["speed"]
                    speed_max, accel_time = mode["speed_max"], mode["accel_time"]
                    rebase = True                         # pas de saut : on repart de la main actuelle
                    note = f"Mode {mode['name']} : échelle ×{scale:.0f}, {speed*1000:.0f} mm/s"
                elif k in (ord('+'), ord('=')):
                    speed = min(speed_max, SPEED_MAX, speed + SPEED_STEP)
                elif k == ord('-'):
                    speed = max(SPEED_MIN, speed - SPEED_STEP)
                elif k == ord(' '):
                    follow = not follow
                    note = "Suivi activé" if follow else "Suivi arrêté (espace)"
                elif k == ord('p') and gripper is not None:
                    gripper.toggle()
                    previous = time.monotonic()   # l'échange réseau ne compte pas comme retard
                elif k == ord('g'):
                    fb_on = not fb_on
                elif k == ord('t'):
                    rot_on = not rot_on
                    hand_rot_ref = None           # nouvelle référence de rotation
                elif k == ord('x'):
                    return message
                else:
                    hand.key(k)
                k = screen.getch()
            accel = speed / accel_time
            track_limit = max(TRACK_MIN, TRACK_LAG * speed)

            # --- 8f. Lectures et surveillance -----------------------------
            check_robot(robot)
            if not control.isProgramRunning():
                raise RuntimeError("Programme de contrôle arrêté côté robot")
            p_hand = hand.position()
            r_hand = hand.orientation()
            pressed = hand.button()
            actual_pose = robot.getActualTCPPose()
            actual = np.array(actual_pose[:3])
            q_now = robot.getActualQ()
            qd_now = robot.getActualQd()
            check_singularities(q_now, actual_pose)
            check_joint_speeds(qd_now)
            slow = slow_factor(qd_now)
            if slow < 1.0:
                note = f"Ralenti : {JOINT_NAMES[int(np.argmax(np.abs(qd_now)))]}"
            rot_speed = min(speed * ROT_SPEED_PER_SPEED, ROT_SPEED_MAX) * slow

            # --- 8g. Retour d'effort --------------------------------------
            f_robot = np.array(robot.getActualTCPForce()[:3])        # N, axes de la base
            f_hand, f_filt, fb_gain = force_feedback(f_robot, f_filt, fb_gain, fb_on and active)
            if now < pulse_until:
                f_hand = f_hand + grasp_pulse(now, pulse_start)
            hand.set_force(f_hand)

            # --- 8h. Bouton -> pince (seulement au moment de l'appui) -----
            if pressed and not was_pressed and gripper is not None:
                gripper.toggle()
                previous = time.monotonic()
            was_pressed = pressed
            if gripper is not None and gripper.obj == 0 and now - last_poll > GRIPPER_POLL:
                if gripper.object_state() == 2:  # objet saisi -> vibration
                    pulse_start, pulse_until = now, now + PULSE_TIME
                    note = "Objet saisi"
                last_poll = previous = time.monotonic()

            # --- 8i. Embrayage : photos de départ à chaque appui sur espace
            if follow and not active:
                rebase = False
                hand.release()                     # la poignée devient libre
                previous = time.monotonic()
                hand_ref, robot_ref, vel, active = p_hand, cmd.copy(), np.zeros(3), True
                hand_rot_ref = None
            elif rebase and active:                # changement de mode pendant le suivi
                hand_ref, robot_ref = p_hand, cmd.copy()   # nouvelle photo, la vitesse continue
                rebase = False
            elif not follow and active:
                active, vel = False, np.zeros(3)
                if HOLD_HANDLE:
                    hand.hold_center()             # suivi OFF : la poignée revient au centre
                    previous = time.monotonic()

            # --- 8j. Calcul du mouvement ----------------------------------
            if active:
                if hand_rot_ref is None:           # photos de l'orientation
                    hand_rot_ref, cmd_rot_ref, spin_ref = r_hand, cmd_rot, spin
                d_tilt, twist = split_hand_gesture(r_hand, hand_rot_ref)

                if not MODE_TEST_PIVOT:            # en mode test : position et inclinaison figées
                    goal = clip_workspace(np.clip(robot_ref + scale * R @ (p_hand - hand_ref),
                                                  start_box_min, start_box_max))
                    cmd, vel = compute_position(cmd, vel, goal, speed, accel, slow)
                    cmd = clip_workspace(cmd)
                    if rot_on:
                        cmd_rot = compute_tilt(cmd_rot, cmd_rot_ref, d_tilt, rot_speed)

                spin = compute_pivot(spin, pivot_goal(spin_ref, twist), slow)

            # --- 8k. Construire, vérifier, envoyer ------------------------
            tcp_rot = cmd_rot * start_rot                              # TCP incliné, sans pivot
            final_check(list(cmd) + list(tcp_rot.as_rotvec()), start_pose, start_rot)
            target_pos, target_rot = build_target(cmd, tcp_rot, spin, tcp_p, tcp_r)
            rv = continuous_rotvec(target_rot.as_rotvec(), prev_rv)    # pas de saut près de 180°
            prev_rv = rv
            target = list(target_pos) + list(rv)
            if not valid_pose(target):
                raise RuntimeError("Cible invalide : rien n'est envoyé")
            error, rot_error = check_tracking(actual_pose, target_pos, target_rot, track_limit)
            control.servoL(target, 0.0, 0.0, DT, LOOKAHEAD, GAIN)     # le SEUL envoi au robot

            # --- 8l. Enregistrement ---------------------------------------
            ang_cmd = math.degrees(cmd_rot.magnitude())
            log.writerow([f"{now - t0:.3f}", int(pressed), int(active), int(rot_on), f"{speed:.3f}",
                          *[f"{v:.5f}" for v in p_hand], *[f"{v:.5f}" for v in cmd],
                          *[f"{v:.5f}" for v in actual], f"{ang_cmd:.2f}",
                          f"{error * 1000:.2f}", f"{math.degrees(rot_error):.2f}",
                          f"{math.degrees(twist):.1f}", f"{math.degrees(spin):.1f}",
                          *[f"{math.degrees(v):.1f}" for v in q_now], *[f"{v:.3f}" for v in qd_now]])

            # --- 8m. Affichage (10 Hz) ------------------------------------
            if now - last_draw >= 0.1:
                draw(screen, screen_lines(dict(
                    cmd=cmd, start=start, actual=actual, gripper=gripper, log_name=log_name,
                    speed=speed, rot_speed=rot_speed, rot_on=rot_on, active=active,
                    fb_on=fb_on, fb_gain=fb_gain, f_filt=f_filt, f_hand=f_hand,
                    error=error, track_limit=track_limit, ang_cmd=ang_cmd,
                    rot_error=rot_error, twist=twist, spin=spin, note=note,
                    mode=mode["name"], scale=scale)))
                last_draw = now

            control.waitPeriod(t_cycle)

    except KeyboardInterrupt:
        message = "Arrêt demandé (Ctrl+C)."
    except Exception as e:
        message = f"ARRÊT DE SÉCURITÉ : {e}"
    finally:
        # --- 8n. Arrêt propre, quelle que soit la raison ------------------
        if control is not None:
            try:
                control.servoStop()
                control.stopScript()
            except Exception:
                pass
        if robot is not None:
            robot.disconnect()
        if hand is not None:
            hand.close()
        if gripper is not None:
            gripper.close()
        if log_file is not None:
            log_file.close()
    return message


if __name__ == "__main__":
    print(curses.wrapper(main))
    print("Connexions fermées.")