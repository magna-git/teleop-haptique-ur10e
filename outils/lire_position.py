"""lire_position.py : affiche les angles actuels du bras, à copier dans Q_HOME_DEG.
Amener d'abord le bras à la main (tablette) dans la position de départ voulue."""
import math
import rtde_receive

robot = rtde_receive.RTDEReceiveInterface("169.254.140.20")
q = [round(math.degrees(a), 1) for a in robot.getActualQ()]
print("Q_HOME_DEG =", q)
robot.disconnect()