""" To jest kod realizujący trot gait dla quadruped robot dog. 
Robot ma 4 nogi po dwa serwa na każdą nogę. 
Konstrukcja nogi wygląda jak 2-DOF planar parallel manipulator. 
Serwa dla kazdej nogi zamontowane są na tym samym poziomie obok siebie, 
odleglośc miedzy osiami serw wynosi d mm. Każda noga ma dwa ogniwa o długościach r1 i r2 mm. 
Nogi FL i BR są w tej samej fazie, zaczynają ruch po łuku, 
nogi FR i BL są w tej samej fazie zaczynają ruch po lini. """

import math
import time
import keyboard
import numpy as np
from itertools import cycle
from st3215 import ST3215

servo = ST3215('COM3')

# Servo settings
## LF [1,2], FR [3,4], BL [5,6], BR [7,8]
sts_id = [1, 2, 3, 4, 5, 6, 7, 8,]
acc = 250
speed = 2400

# Leg settings
r1 = 40.0
r2 = 50.0  
d = 30.0

# Trot gait settings
t_points = 10
swing_width = 25
swing_height = 20
base_y = -60
x_offset = 0

#####################################

for id in sts_id:
    servo.SetMode(id, 0)
    servo.SetAcceleration(id, acc)
    servo.SetSpeed(id, speed)

def solve_ik(x_target, y_target):
    L1 = math.sqrt((x_target - d/2)**2 + y_target**2)
    phi2 = math.atan2(y_target, x_target - d/2)
    alpha2 = math.acos((r1**2 + L1**2 - r2**2) / (2 * r1 * L1))
    theta2 = phi2 + alpha2

    L2 = math.sqrt((x_target + d/2)**2 + y_target**2)
    phi1 = math.atan2(y_target, x_target + d/2)
    alpha1 = math.acos((r1**2 + L2**2 - r2**2) / (2 * r1 * L2))
    theta1 = phi1 - alpha1

    return theta1, theta2

def trot_gait_cycle(leg_pair):
    points = []
    half_width = swing_width / 2

    if leg_pair == "FL":
        # faza łuku
        for angle in np.linspace(math.pi, 0, t_points, endpoint=False):
            x = -half_width * math.cos(angle) + x_offset
            y = base_y + swing_height * math.sin(angle)
            points.append((x, y))
        # faza liniowa
        for t in np.linspace(0, 1, t_points, endpoint=False):
            x = -half_width + swing_width * t + x_offset
            y = base_y
            points.append((x, y))

    if leg_pair == "BL":
        # faza liniowa
        for t in np.linspace(0, 1, t_points, endpoint=False):
            x = -half_width + swing_width * t + x_offset
            y = base_y
            points.append((x, y))
        # faza łuku
        for angle in np.linspace(0, math.pi, t_points, endpoint=False):
            x = half_width * math.cos(angle) + x_offset
            y = base_y + swing_height * math.sin(angle)
            points.append((x, y))
    print("points")            
    print("\n".join(f"{i:03d}: ({x:.2f}, {y:.2f})" for i, (x, y) in enumerate(points)))
    return cycle(points)  # ZWRACAMY CYKLICZNĄ TRAJEKTORIĘ!

def stand_still():
    th1, th2 = solve_ik(0, base_y)
    servo.WritePosition(1, servo.rad_to_servo(th1))
    servo.WritePosition(2, servo.rad_to_servo(th2))
    servo.WritePosition(4, servo.rad_to_servo(th1))
    servo.WritePosition(3, servo.rad_to_servo(th2))

def start_trot():
    traj_FL = trot_gait_cycle("FL")
    traj_BL = trot_gait_cycle("BL")

    while True:
        if keyboard.is_pressed("w"):
            x, y = next(traj_FL)
            th1, th2 = solve_ik(x, y)
            servo.WritePosition(1, servo.rad_to_servo(th1))
            servo.WritePosition(2, servo.rad_to_servo(th2))

            x2, y2 = next(traj_BL)
            th1, th2 = solve_ik(x2, y2)
            servo.WritePosition(4, servo.rad_to_servo(th1))
            servo.WritePosition(3, servo.rad_to_servo(th2))
        
        else:
            stand_still()

        time.sleep(0.06)

def generate_cycle_servo_positions(leg_pair):
    traj = trot_gait_cycle(leg_pair)
    positions = []

    for _ in range(2 * t_points):  # pełen cykl (łuk + linia)
        x, y = next(traj)
        th1, th2 = solve_ik(x, y)
        s1 = servo.rad_to_servo(th1)
        s2 = servo.rad_to_servo(th2)
        positions.append((s1, s2))

    return positions

cycle_FL = generate_cycle_servo_positions("FL")
cycle_BL = generate_cycle_servo_positions("BL")

print("Pozycje serw (FL):")
for p in cycle_FL:
    print(p)

