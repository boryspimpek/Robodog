import math
import time
from matplotlib import pyplot as plt
import numpy as np

"""
To jest kod realizujący trot gait dla quadruped robot dog. 
Robot ma 4 nogi po dwa serwa na każdą nogę. 
Konstrukcja nogi wygląda jak 2-DOF planar parallel manipulator. 
Serwa dla kazdej nogi zamontowane są na tym samym poziomie obok siebie,
odleglośc miedzy osiami serw wynosi d mm.
Każda noga ma dwa ogniwa o długościach r1 i r2 mm.
Nogi FL i BR są w tej samej fazie, zaczynają ruch po łuku,
nogi FR i BL są w tej samej fazie zaczynają ruch po lini.
"""

# Konfiguracja robota
r1 = 40.0  # długość pierwszego ogniwa
r2 = 50.0  # długość drugiego ogniwa  
d = 30.0   # odległość między serwami na podstawie

# ID serw
FL_ids = [1, 2]  # Front Left
FR_ids = [3, 4]  # Front Right  
BL_ids = [5, 6]  # Back Left
BR_ids = [7, 8]  # Back Right

def rad_to_servo(rad):
    """Zamienia kąt w radianach na wartość serwa (0-4095)"""
    center = 2048
    scale = 2048 / 3.1415926535
    return 4095 - int(round(rad * scale + center))

def SyncMoveTo(servo_dict, max_speed, acc, wait):
    """Symulacja funkcji ruchu serw - w prawdziwym robocie tu będzie komunikacja z serwami"""
    print(f"Ruch serw: {servo_dict}")
    # W prawdziwym kodzie tutaj będzie:
    # real_SyncMoveTo(servo_dict, max_speed, acc, wait)

def solve_ik(x_target, y_target, d=d, r1=r1, r2=r2):
    L1 = math.sqrt((x_target - d/2)**2 + (y_target)**2)
    phi2 = math.atan2(y_target, x_target - d/2)
    alpha2 = math.acos((r1**2 + L1**2 - r2**2) / (2 * r1 * L1))
    theta2 = phi2 + alpha2  # elbow_out=True

    L2 = math.sqrt((x_target - (-d/2))**2 + (y_target)**2)
    phi1 = math.atan2(y_target, x_target - (-d/2))
    alpha1 = math.acos((r1**2 + L2**2 - r2**2) / (2 * r1 * L2))
    theta1 = phi1 - alpha1 # elbow_out=False

    print(f"L1 = {L1:.2f}")    
    print(f"phi2 deg: {math.degrees(phi2):.2f}")
    print(f"alpha2 deg: {math.degrees(alpha2):.2f}")
    print(f"THETA2 = {math.degrees(theta2):.2f}")
    print()
    print(f"L2 = {L2:.2f}")    
    print(f"phi2 deg: {math.degrees(phi1):.2f}")
    print(f"alpha1 deg: {math.degrees(alpha1):.2f}")
    print(f"THETA1 = {math.degrees(theta1):.2f}")
    print()

    return theta1, theta2

def generateSwing():
    points = []
    
    # FAZA 1: Przenoszenie (łuk w górę)
    for angle in np.linspace(math.pi, 0, 50):
        x = -20 * math.cos(angle)  
        y = -60 + 20 * math.sin(angle)
        points.append((x, y))
    
    # FAZA 2: Podpora (powrót po linii)
    for t in np.linspace(0, 1, 50):
        x = -20 + 40 * t   
        y = -60
        points.append((x, y))
    
    return points

def generateStance():
    points = []
    
    # FAZA 1: Podpora (ruch poziomo)
    for t in np.linspace(0, 1, 50):
        x = -20 + 40 * t  
        y = -60
        points.append((x, y))
    
    # FAZA 2: Przenoszenie (powrót po łuku)  
    for angle in np.linspace(0, math.pi, 50):  
        x = 20 * math.cos(angle)  
        y = -60 + 20 * math.sin(angle)
        points.append((x, y))
    
    return points

def main():
    # Przygotowanie trajektorii
    traj_FL = generateSwing()
    traj_FR = generateStance() 
    traj_BL = generateStance()
    traj_BR = generateSwing()
    
    for i in range(100):
        # Pobierz aktualne pozycje dla wszystkich nóg
        fl_x, fl_y = traj_FL[i]
        fr_x, fr_y = traj_FR[i]
        bl_x, bl_y = traj_BL[i]
        br_x, br_y = traj_BR[i]

        # Oblicz kąty dla każdej nogi
        t1_fl, t2_fl = solve_ik(fl_x, fl_y)
        t1_fr, t2_fr = solve_ik(fr_x, fr_y)
        t1_bl, t2_bl = solve_ik(bl_x, bl_y)
        t1_br, t2_br = solve_ik(br_x, br_y)
        
        # Zamień kąty na wartości serw
        servo_positions = {
            FL_ids[0]: rad_to_servo(t1_fl), FL_ids[1]: rad_to_servo(t2_fl),  # FL
            FR_ids[0]: rad_to_servo(t1_fr), FR_ids[1]: rad_to_servo(t2_fr),  # FR  
            BL_ids[0]: rad_to_servo(t1_bl), BL_ids[1]: rad_to_servo(t2_bl),  # BL
            BR_ids[0]: rad_to_servo(t1_br), BR_ids[1]: rad_to_servo(t2_br),  # BR
        }

        # Wyślij komendę do serw
        SyncMoveTo(servo_dict=servo_positions, max_speed=800, acc=30, wait=True)
        time.sleep(0.02)

    