import math
from matplotlib import pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import numpy as np
# from st3215 import ST3215

"""
To jest kod realizujący trot gait dla quadruped robot dog. 
Robot ma 4 nogi po dwa serwa na każdą nogę. 
Konstrukcja nogi wygląda jak 2-DOF planar parallel manipulator. 
"""

r1 = 40.0  # długość pierwszego ogniwa
r2 = 50.0  # długość drugiego ogniwa
d = 30.0   # odległość między serwami na podstawie
    
FL_ids = [1, 2]  # IDs for Front Left leg servos
FR_ids = [3, 4]  # IDs for Front Right leg servos   
BL_ids = [5, 6]  # IDs for Back Left leg servos
BR_ids = [7, 8]  # IDs for Back Right leg servos

def rad_to_servo(rad):
    center = 2048
    scale = 2048 / 3.1415926535
    return 4095 - int(round(rad * scale + center))

def compute_theta(x_a, y_a, x_p, y_p, r1, r2, mirror=False):
    L = math.sqrt((x_p - x_a)**2 + (y_p - y_a)**2)
    phi = math.atan2(y_p - y_a, x_p - x_a)
    alpha = math.acos((r1**2 + L**2 - r2**2) / (2 * r1 * L))
    return phi + alpha if mirror else phi - alpha

def compute_servo_positions_single_leg(x_target, y_target, is_mirror=False):
    if is_mirror:
        # Dla prawej strony - lustrzane odbicie w osi X
        theta1 = compute_theta(d/2, 0, x_target, y_target, r1, r2, mirror=True)
        theta2 = compute_theta(-d/2, 0, x_target, y_target, r1, r2, mirror=True)
    else:
        # Dla lewej strony - standardowe obliczenia
        theta1 = compute_theta(-d/2, 0, x_target, y_target, r1, r2)
        theta2 = compute_theta(d/2, 0, x_target, y_target, r1, r2)
    
    return theta1, theta2

def compute_all_servo_positions(FL_pos, FR_pos, BL_pos, BR_pos):
    legs = [(FL_pos, FL_ids, False), (FR_pos, FR_ids, True), (BL_pos, BL_ids, False), (BR_pos, BR_ids, True),]

    result = {}
    for pos, ids, mirror in legs:
        theta1, theta2 = compute_servo_positions_single_leg(*pos, is_mirror=mirror)
        result[ids[0]] = rad_to_servo(theta1)
        result[ids[1]] = rad_to_servo(theta2)
    return result

def execute_trajectory(traj_FL, traj_FR, traj_BL, traj_BR):
    if len(traj_FL) != len(traj_FR) != len(traj_BL) != len(traj_BR):
        raise ValueError("All trajectories must have the same length")
        
    all_positions = []
    for i in range(len(traj_FL)):
        positions = compute_all_servo_positions(traj_FL[i], traj_FR[i], traj_BL[i], traj_BR[i])
        all_positions.append(positions)
    
    for positions in all_positions:
        SyncMoveTo(servo_dict=positions, max_speed=800, acc=30, wait=True)

def generate_trajectory_FL():
    points = []
    
    # FAZA 1: Przenoszenie (łuk) - od pozycji startowej do końcowej
    for angle in np.linspace(math.pi, 0, 50):
        x = -20 * math.cos(angle)  
        y = -60 + 20 * math.sin(angle)
        points.append((x, y))
    
    # FAZA 2: Podpora (linia) - powrót do pozycji startowej
    for t in np.linspace(0, 1, 50):
        x = -20 + 40 * t   
        y = -60
        points.append((x, y))
    
    return points

def generate_trajectory_BL():
    points = []
    
    # FAZA 1: Podpora (linia) - od pozycji startowej do końcowej
    for t in np.linspace(0, 1, 50):
        x = -20 + 40 * t  
        y = -60
        points.append((x, y))
    
    # FAZA 2: Przenoszenie (łuk) - powrót do pozycji startowej  
    for angle in np.linspace(0, math.pi, 50):  
        x = 20 * math.cos(angle)  
        y = -60 + 20 * math.sin(angle)
        points.append((x, y))
    
    return points

def generate_trajectory_FR():
    points = []
    
    # FAZA 1: Podpora (linia)
    for t in np.linspace(0, 1, 50):
        x = -20 + 40 * t  # Lustrzane odbicie  
        y = -60
        points.append((x, y))
    
    # FAZA 2: Przenoszenie (łuk)
    for angle in np.linspace(math.pi, 0, 50):
        x = -20 * math.cos(angle)  # Lustrzane odbicie w osi X
        y = -60 + 20 * math.sin(angle)
        points.append((x, y))
    
    return points

def generate_trajectory_BR():
    points = []
    
    # FAZA 1: Przenoszenie (łuk) 
    for angle in np.linspace(0, math.pi, 50):  
        x = 20 * math.cos(angle)  # Lustrzane odbicie
        y = -60 + 20 * math.sin(angle)
        points.append((x, y))
    
    # FAZA 2: Podpora (linia) 
    for t in np.linspace(0, 1, 50):
        x = -20 + 40 * t  # Lustrzane odbicie
        y = -60
        points.append((x, y))
    
    return points

def visualize_trajectories_with_direction():
    """Wizualizacja trajektorii z oznaczeniem kierunku i faz"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.suptitle('Trajektorie nóg robota - Trot Gait z kierunkiem', fontsize=16)
    
    # Generuj trajektorie
    traj_FL = generate_trajectory_FL()
    traj_FR = generate_trajectory_FR()
    traj_BL = generate_trajectory_BL()
    traj_BR = generate_trajectory_BR()
    
    # Konwertuj na numpy arrays
    traj_FL = np.array(traj_FL)
    traj_FR = np.array(traj_FR)
    traj_BL = np.array(traj_BL)
    traj_BR = np.array(traj_BR)
    
    # Stwórz kolory wskazujące progresję w czasie
    def add_direction_plot(ax, trajectory, title, phase_split=50):
        # Podziel na fazy
        phase1 = trajectory[:phase_split]  # Faza 1
        phase2 = trajectory[phase_split:]  # Faza 2
        
        # Rysuj fazy różnymi kolorami
        ax.plot(phase1[:, 0], phase1[:, 1], 'b-', linewidth=3, alpha=0.7)
        ax.plot(phase2[:, 0], phase2[:, 1], 'r-', linewidth=3, alpha=0.7)
        
        # Dodaj strzałki kierunku co 10 punktów
        for i in range(0, len(trajectory)-1, 10):
            dx = trajectory[i+1, 0] - trajectory[i, 0]
            dy = trajectory[i+1, 1] - trajectory[i, 1]
            ax.arrow(trajectory[i, 0], trajectory[i, 1], dx*0.8, dy*0.8, 
                    head_width=2, head_length=2, fc='black', ec='black', alpha=0.5)
        
        # Punkty startowe i końcowe
        ax.scatter(trajectory[0, 0], trajectory[0, 1], color='green', s=150, 
                  label='Start', zorder=5, marker='o')
        ax.scatter(trajectory[-1, 0], trajectory[-1, 1], color='red', s=150, 
                  label='Koniec', zorder=5, marker='s')
        
        ax.set_title(title)
        ax.set_xlabel('X [mm]')
        ax.set_ylabel('Y [mm]')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.axis('equal')
    
    # Rysuj wszystkie nogi
    add_direction_plot(axes[0, 0], traj_FL, 'FL (Lewa przód)')
    add_direction_plot(axes[0, 1], traj_FR, 'FR (Prawa przód)')  
    add_direction_plot(axes[1, 0], traj_BL, 'BL (Lewa tył)')
    add_direction_plot(axes[1, 1], traj_BR, 'BR (Prawa tył)')
    
    plt.tight_layout()
    plt.show()

# Zmodyfikowany główny kod do testowania
if __name__ == "__main__":
    visualize_trajectories_with_direction()
    traj_FL = generate_trajectory_FL()
    traj_BL = generate_trajectory_BL()
    traj_FR = generate_trajectory_FR()  
    traj_BR = generate_trajectory_BR()  
    execute_trajectory(traj_FL, traj_FR, traj_BL, traj_BR)