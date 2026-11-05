"""
Test du capteur d'effort du UR10e (le robot ne bouge pas).
1. Remise à zéro du capteur (robot immobile, pince sans contact)
2. Affichage de la force 5 fois par seconde pendant 20 s
   -> appuie doucement sur la pince VERS LE HAUT et note le signe de Fz
Exige le mode À distance (pour zeroFtSensor).
"""
import time
import numpy as np
import rtde_control, rtde_receive

IP = "169.254.140.20"
r = rtde_receive.RTDEReceiveInterface(IP)
c = rtde_control.RTDEControlInterface(IP)
c.zeroFtSensor()                      # remise à zéro : rien ne doit toucher la pince
time.sleep(0.5)
samples = []
t0 = time.time()
while time.time() - t0 < 20:
    f = r.getActualTCPForce()         # [Fx, Fy, Fz, Tx, Ty, Tz] en N et N.m
    samples.append(f[:3])
    print(f"Fx {f[0]:+6.1f}  Fy {f[1]:+6.1f}  Fz {f[2]:+6.1f} N")
    time.sleep(0.2)
s = np.array(samples[:10])            # les 2 premières secondes, sans contact
print("Bruit (écart-type sur 2 s, sans contact) :", np.round(s.std(axis=0), 2), "N")
c.stopScript()