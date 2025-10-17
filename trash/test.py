import math
import time
from matplotlib import pyplot as plt
import numpy as np
from st3215 import ST3215

servo = ST3215('COM3')

# Konfiguracja
sts_id = [1, 2]
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

def rad_to_servo(rad):
    center = 2048
    scale = 2048 / 3.1415926535
    return 4095 - int(round(rad * scale + center))

def plot(trajectory):
    points = trajectory()
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]

    plt.plot(x_coords, y_coords, 'b.-')
    plt.plot(x_coords[0], y_coords[0], 'ro', markersize=8)  # czerwone kółko na pierwszym punkcie
    plt.show()

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

def first_leg_pair(swing_width=swing_width, swing_height=swing_height, base_y=base_y, t_points=t_points, x_offset=x_offset):
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

def second_leg_pair(swing_width=swing_width, swing_height=swing_height, base_y=base_y, t_points=t_points, x_offset=x_offset):
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

first_trajectory = first_leg_pair()
second_trajectory = second_leg_pair()

for i, point in enumerate(first_trajectory):
    x, y = point
    
    theta1, theta2 = solve_ik(x, y)
    pos1 = rad_to_servo(theta1)
    pos2 = rad_to_servo(theta2)
    
    servo.WritePosition(sts_id[0], pos1)
    servo.WritePosition(sts_id[1], pos2)
    
    time.sleep(0.06)

