import math
import time
from matplotlib import pyplot as plt
import numpy as np
from st3215 import ST3215

servo = ST3215('COM3')


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

t_points = 10

def rad_to_servo(rad):
    """Zamienia kąt w radianach na wartość serwa (0-4095)"""
    center = 2048
    scale = 2048 / 3.1415926535
    return 4095 - int(round(rad * scale + center))

def solve_ik(x_target, y_target, d=d, r1=r1, r2=r2):
    L1 = math.sqrt((x_target - d/2)**2 + (y_target)**2)
    phi2 = math.atan2(y_target, x_target - d/2)
    alpha2 = math.acos((r1**2 + L1**2 - r2**2) / (2 * r1 * L1))
    theta2 = phi2 + alpha2  # elbow_out=True

    L2 = math.sqrt((x_target - (-d/2))**2 + (y_target)**2)
    phi1 = math.atan2(y_target, x_target - (-d/2))
    alpha1 = math.acos((r1**2 + L2**2 - r2**2) / (2 * r1 * L2))
    theta1 = phi1 - alpha1 # elbow_out=False

    # print(f"L2 = {L2:.2f}")    
    # print(f"phi2 deg: {math.degrees(phi2):.2f}")
    # print(f"alpha2 deg: {math.degrees(alpha2):.2f}")
    # print(f"THETA2 = {math.degrees(theta2):.2f}")
    # print()
    # print(f"L1 = {L1:.2f}")    
    # print(f"phi1 deg: {math.degrees(phi1):.2f}")
    # print(f"alpha1 deg: {math.degrees(alpha1):.2f}")
    # print(f"THETA1 = {math.degrees(theta1):.2f}")
    # print()

    return theta1, theta2

def generateSwing():
    points = []
    
    # FAZA 1: Przenoszenie (łuk w górę)
    for angle in np.linspace(math.pi, 0, t_points):
        x = -20 * math.cos(angle)  
        y = -60 + 20 * math.sin(angle)
        points.append((x, y))
        # print(f"F1 Punkt: x={x:.2f}, y={y:.2f}")

    # FAZA 2: Podpora (powrót po linii)
    for t in np.linspace(0, 1, t_points):
        x = -20 + 40 * t   
        y = -60
        points.append((x, y))
        # print(f"F2 Punkt: x={x:.2f}, y={y:.2f}")
    
    return points

def generateStance():
    points = []
    
    # FAZA 1: Podpora (ruch poziomo)
    for t in np.linspace(0, 1, t_points):
        x = -20 + 40 * t  
        y = -60
        points.append((x, y))
    
    # FAZA 2: Przenoszenie (powrót po łuku)  
    for angle in np.linspace(0, math.pi, t_points):  
        x = 20 * math.cos(angle)  
        y = -60 + 20 * math.sin(angle)
        points.append((x, y))
    
    return points

def home():
    x = 0
    y = -60
    move_to_point(x, y)

def move_to_point(x, y):
    theta1, theta2 = solve_ik(x, y)
    print(f"THETA1 = {math.degrees(theta1):.2f}")
    print(f"THETA2 = {math.degrees(theta2):.2f}")
    servo_positions = {
        FL_ids[0]: rad_to_servo(theta1), FL_ids[1]: rad_to_servo(theta2),  # FL
    }
    print("Servo positions:", servo_positions)

def main():
    traj_FL = generateSwing()
    
    for i in range(2 * t_points):
        x, y = traj_FL[i]
        t1_fl, t2_fl = solve_ik(x, y)
        
        target_positions = {
            FL_ids[0]: rad_to_servo(t1_fl), 
            FL_ids[1]: rad_to_servo(t2_fl)
        }
        






































# servo_positions = {1: 3757, 2: 2903}  
# servo.SyncMoveTo(servo_positions, max_speed=800, acc=30, wait=False)

# servo_positions = {1: 3239, 2: 2385}  
# servo.SyncMoveTo(servo_positions, max_speed=800, acc=30, wait=False)


# for i in range (20):
#     main()   

# home()

# move_to_point(0, -60)

# for i in range (5):

#     move_to_point(-20, -60)
#     move_to_point(20, -60)
