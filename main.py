""" To jest kod realizujący trot gait dla quadruped robot dog. 
Robot ma 4 nogi po dwa serwa na każdą nogę. 
Konstrukcja nogi wygląda jak 2-DOF planar parallel manipulator. 
Serwa dla kazdej nogi zamontowane są na tym samym poziomie obok siebie, 
odleglośc miedzy osiami serw wynosi d mm. Każda noga ma dwa ogniwa o długościach r1 i r2 mm. 
Nogi FL i BR są w tej samej fazie, zaczynają ruch po łuku, 
nogi FR i BL są w tej samej fazie zaczynają ruch po lini. """

import math
import time
import numpy as np
from st3215 import ST3215

servo = ST3215('COM3')

# Konfiguracja
## LF [1,2], FR [3,4], BL [5,6], BR [7,8]

sts_id = [1, 2, 3, 4, 5, 6, 7, 8,]
acc = 250
speed = 2400
r1 = 40.0
r2 = 50.0  
d = 30.0
t_points = 10
swing_width = 20
swing_height = 10
base_y = -80
x_offset = 0

# Inicjalizacja serw
for id in sts_id:
    servo.SetMode(id, 0)
    servo.SetAcceleration(id, acc)
    servo.SetSpeed(id, speed)

def solve_ik(x_target, y_target, d=d, r1=r1, r2=r2):
    L1 = math.sqrt((x_target - d/2)**2 + y_target**2)
    phi2 = math.atan2(y_target, x_target - d/2)
    alpha2 = math.acos((r1**2 + L1**2 - r2**2) / (2 * r1 * L1))
    theta2 = phi2 + alpha2

    L2 = math.sqrt((x_target + d/2)**2 + y_target**2)
    phi1 = math.atan2(y_target, x_target + d/2)
    alpha1 = math.acos((r1**2 + L2**2 - r2**2) / (2 * r1 * L2))
    theta1 = phi1 - alpha1

    return theta1, theta2

def trot_gait(leg_pair):
    # Trajectory for FL 
    if leg_pair == "FL":         
        points = []
        half_width = swing_width / 2
        for i in range (20):
            for angle in np.linspace(math.pi, 0, t_points, endpoint=False):
                x = -half_width * math.cos(angle) + x_offset
                y = base_y + swing_height * math.sin(angle)
                points.append((x, y))

            for t in np.linspace(0, 1, t_points, endpoint=False):  
                x = -half_width + swing_width * t + x_offset
                y = base_y
                points.append((x, y))

        return points

    # Trajectory for  BL leg
    if leg_pair == "BL":        
        points = []
        half_width = swing_width / 2
        for i in range (20):
            for t in np.linspace(0, 1, t_points, endpoint=False):
                x = -half_width + swing_width * t + x_offset
                y = base_y
                points.append((x, y))

            for angle in np.linspace(0, math.pi, t_points, endpoint=False):
                x = half_width * math.cos(angle) + x_offset
                y = base_y + swing_height * np.sin(angle)
                points.append((x, y))

        return points

first_trajectory = trot_gait("FL")
second_trajectory = trot_gait("BL")

for i in range(len(first_trajectory)):
    # LF (1,2) - First Trajectory
    x, y = first_trajectory[i]
    th1, th2 = solve_ik(x, y)
    servo.WritePosition(1, servo.rad_to_servo(th1))
    servo.WritePosition(2, servo.rad_to_servo(th2))

    # # RB (7,8) - First Trajectory
    # th1, th2 = solve_ik(x, y)
    # servo.WritePosition(8, servo.rad_to_servo(th1))
    # servo.WritePosition(7, servo.rad_to_servo(th2))

    # RF (3,4) - Second Trajectory
    x2, y2 = second_trajectory[i]
    th1, th2 = solve_ik(x2, y2)
    servo.WritePosition(4, servo.rad_to_servo(th1))
    servo.WritePosition(3, servo.rad_to_servo(th2))

    # # LB (5,6) - Second Trajectory
    # th1, th2 = solve_ik(x2, y2)
    # servo.WritePosition(5, servo.rad_to_servo(th1))
    # servo.WritePosition(6, servo.rad_to_servo(th2))

    time.sleep(0.06)
