"""
calib_axe_poignee.py : mesurer l'axe EXACT de vrille du stylo (robot non utilisé).
Mode d'emploi :
  1. stylo droit et immobile, Entrée ;
  2. vriller le stylo dans un sens puis dans l'autre, plusieurs fois, en roulant
     le stylo entre le pouce et l'index (coude posé), pendant ~15 s ;
  3. Ctrl+C : le script affiche la ligne HANDLE_AXIS à copier dans teleop_6axes.py.
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import time
import numpy as np
from teleop_demo_6axes import OmegaHand

hand = OmegaHand()
samples = []
try:
    input("Stylo droit et immobile, puis Entrée : ")
    ref = hand.orientation()
    print("Vrille dans un sens puis dans l'autre... (Ctrl+C pour finir)")
    while True:
        rv = (ref.inv() * hand.orientation()).as_rotvec()   # geste vu depuis le stylo
        a = np.linalg.norm(rv)
        if a > np.radians(15):                              # vrille assez grande
            u = rv / a
            if samples and u @ samples[0] < 0:              # même sens pour tous
                u = -u
            samples.append(u)
            print(f"angle {np.degrees(a):5.1f}°  axe {np.round(u, 2)}")
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    hand.close()
if len(samples) >= 10:
    axis = np.mean(samples, axis=0)
    axis /= np.linalg.norm(axis)
    spread = np.degrees(np.mean([np.arccos(np.clip(u @ axis, -1, 1)) for u in samples]))
    print(f"\n{len(samples)} mesures, dispersion moyenne {spread:.1f}°")
    print(f"HANDLE_AXIS = np.array([{axis[0]:.3f}, {axis[1]:.3f}, {axis[2]:.3f}])")
else:
    print("\nPas assez de mesures : vrille plus longtemps (plus de 15°).")