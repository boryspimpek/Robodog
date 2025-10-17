import math
import time
from matplotlib import pyplot as plt
import numpy as np
from st3215 import ST3215

servo = ST3215('COM3')

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

    return theta1, theta2

def move_leg(leg_ids, x, y, speed=2400, acc=50):
    theta1, theta2 = solve_ik(x, y)
    pos1 = rad_to_servo(theta1)
    pos2 = rad_to_servo(theta2)
    
    # print(f"Moving leg {leg_ids} to: x={x:.2f}, y={y:.2f}")
    # print(f"Angles: theta1={math.degrees(theta1):.1f}°, theta2={math.degrees(theta2):.1f}°")
    # print(f"Servo positions: {pos1}, {pos2}")
    
    # Ustawienie trybu pozycyjnego i parametrów ruchu dla każdego serwa
    for servo_id in leg_ids:
        servo.SetMode(servo_id, 0)  # tryb pozycyjny
        servo.SetAcceleration(servo_id, acc)
        servo.SetSpeed(servo_id, speed)
    
    time.sleep(0.01)
    
    result1 = servo.WritePosition(leg_ids[0], pos1)
    result2 = servo.WritePosition(leg_ids[1], pos2)
    
    return result1 and result2

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
        # print(f"F1 Punkt: x={x:.2f}, y={y:.2f}")

    # FAZA 2: Przenoszenie (powrót po łuku)  
    for angle in np.linspace(0, math.pi, t_points):  
        x = 20 * math.cos(angle)  
        y = -60 + 20 * math.sin(angle)
        points.append((x, y))
        # print(f"F2 Punkt: x={x:.2f}, y={y:.2f}")

    return points

def plot(trajectory):
    points = trajectory()
    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]

    plt.plot(x_coords, y_coords, 'b.-')
    plt.plot(x_coords[0], y_coords[0], 'ro', markersize=8)  # czerwone kółko na pierwszym punkcie
    plt.show()

def start_trot_gait(leg_ids, trajectory_points, speed=2400, acc=30, delay=0.1):
    print(f"Liczba punktów trajektorii: {len(trajectory_points)}")
    print("Wszystkie punkty trajektorii:")
    for i, point in enumerate(trajectory_points):
        x, y = point
        print(f"  {i}: x={x:.2f}, y={y:.2f}")
    
    print("\nWykonywanie ruchu:")
    for i, point in enumerate(trajectory_points):
        x, y = point
        print(f"Point {i+1}/{len(trajectory_points)}: x={x:.2f}, y={y:.2f}")
        
        success = move_leg(leg_ids, x, y, speed, acc)
        
        if not success:
            print(f"Błąd podczas ruchu do punktu {i+1}")
            break
            
        time.sleep(delay)

def main():
    swing_trajectory = generateSwing()
    stance_trajectory = generateStance()
    
    print("=== SWING TRAJECTORY ===")
    for i in range (5):
        start_trot_gait(FL_ids, swing_trajectory, speed=2400, acc=250, delay=0.1)
        time.sleep(0.5)
    
    time.sleep(2)
    print("\n=== STANCE TRAJECTORY ===")
    for i in range (5):
        start_trot_gait(FL_ids, stance_trajectory, speed=2400, acc=250, delay=0.1)        
        time.sleep(0.5)

if __name__ == "__main__":
    main()