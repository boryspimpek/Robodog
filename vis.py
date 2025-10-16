import math
import time
from matplotlib import pyplot as plt
import numpy as np

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
    
def visualize_single_leg_trajectory(trajectory_type="swing"):
    if trajectory_type == "swing":
        trajectory = generateSwing()
        title = "Trajektoria SWING (przenoszenie)"
    else:
        trajectory = generateStance()
        title = "Trajektoria STANCE (podpora)"
    
    x_a1, y_a1 = -d/2, 0  # Serwo 1
    x_a2, y_a2 = d/2, 0   # Serwo 2
    color1, color2 = 'red', 'blue'
    
    plt.figure(figsize=(13, 8))
    
    # Rysowanie całej trajektorii
    traj_x = [p[0] for p in trajectory]
    traj_y = [p[1] for p in trajectory]
    plt.plot(traj_x, traj_y, 'g-', alpha=0.3, linewidth=3, label='Trajektoria końcówki')
    
    # Kluczowe pozycje do pokazania kątów
    key_positions = [0, len(trajectory)//4, len(trajectory)//2, 3*len(trajectory)//4]
    key_colors = ['red', 'blue', 'green', 'orange']
    key_labels = ['START', '25%', '50%', '75%']
    
    valid_points = 0
    
    for i in range(len(trajectory)):
        x_p, y_p = trajectory[i]
        
        try:
            # Obliczenie kątów używając Twojej funkcji
            theta1, theta2 = solve_ik(x_p, y_p)
            
            # Położenie przegubów pośrednich
            x_b1 = x_a1 + r1 * math.cos(theta1)
            y_b1 = y_a1 + r1 * math.sin(theta1)
            x_b2 = x_a2 + r1 * math.cos(theta2)  
            y_b2 = y_a2 + r1 * math.sin(theta2)
            
            # Rysowanie tylko dla kluczowych pozycji z kątami
            if i in key_positions:
                idx = key_positions.index(i)
                color = key_colors[idx]
                label = key_labels[idx]
                
                # Rysowanie konfiguracji robota
                plt.plot([x_a1, x_b1, x_p], [y_a1, y_b1, y_p], 
                        color=color, marker='o', alpha=0.8, linewidth=2.5,
                        label=f'{label} - konfiguracja')
                plt.plot([x_a2, x_b2, x_p], [y_a2, y_b2, y_p], 
                        color=color, marker='o', alpha=0.8, linewidth=2.5)
                
                # Zaznaczenie końcówki
                plt.scatter(x_p, y_p, color=color, s=100, alpha=1.0, zorder=5)
                
                # Etykieta theta1 na ramieniu r1
                mid_x1 = (x_a1 + x_b1) / 2
                mid_y1 = (y_a1 + y_b1) / 2
                plt.text(mid_x1, mid_y1, f'θ₁={math.degrees(theta1):.0f}°, {rad_to_servo(theta1)}', 
                        fontsize=9, ha='center', va='center', 
                        fontweight='bold',  
                        color='white',      
                        bbox=dict(boxstyle="round,pad=0.3", 
                                facecolor='black', 
                                alpha=0.9,
                                edgecolor='black',  
                                linewidth=1.5))     
                
                # Etykieta theta2 na ramieniu r2  
                mid_x2 = (x_a2 + x_b2) / 2
                mid_y2 = (y_a2 + y_b2) / 2
                plt.text(mid_x2, mid_y2, f'θ₂={math.degrees(theta2):.0f}°, {rad_to_servo(theta2)}',
                        fontsize=9, ha='center', va='center', 
                        fontweight='bold',  
                        color='white',      
                        bbox=dict(boxstyle="round,pad=0.3", 
                                facecolor='black', 
                                alpha=0.9,
                                edgecolor='black',  
                                linewidth=1.5))     
            
            valid_points += 1
            
        except (ValueError, ZeroDivisionError) as e:
            # Pomijamy punkty poza zasięgiem
            continue
    
    # Podstawa robota (miejsce montażu serw)
    plt.plot([x_a1, x_a2], [y_a1, y_a2], 'k-', linewidth=6,)
    plt.scatter([x_a1, x_a2], [y_a1, y_a2], color='black', s=120, zorder=5)
    
    # Oznaczenie serw
    plt.text(x_a1, y_a1 - 8, 'Serwo 1', ha='center', va='top', fontsize=10,
            bbox=dict(boxstyle="round,pad=0.3", facecolor='red', alpha=0.7))
    plt.text(x_a2, y_a2 - 8, 'Serwo 2', ha='center', va='top', fontsize=10,
            bbox=dict(boxstyle="round,pad=0.3", facecolor='blue', alpha=0.7))
    
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=10, loc='upper right')
    plt.title(f'{title}\n'
              f'Pokazano kąty dla kluczowych pozycji trajektorii', fontsize=14, pad=20)
    plt.xlabel('X [mm]', fontsize=12)
    plt.ylabel('Y [mm]', fontsize=12)
    
    # Dodanie informacji o parametrach
    textstr = f'Parametry mechaniczne:\n• r₁ = {r1} mm\n• r₂ = {r2} mm\n• d = {d} mm\n• Wysokość kroku = 20 mm\n• Długość kroku = 40 mm'
    props = dict(boxstyle='round', facecolor='lightblue', alpha=0.8)
    plt.text(0.02, 0.98, textstr, transform=plt.gca().transAxes, fontsize=10,
             verticalalignment='top', bbox=props)
    
    # Dodanie opisu kątów
    angle_info = 'θ₁ - kąt serwa 1\nθ₂ - kąt serwa 2'
    plt.text(0.98, 0.02, angle_info, transform=plt.gca().transAxes, fontsize=9,
             verticalalignment='bottom', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
    
    plt.tight_layout()
    plt.show()

def visualize_single_point(x_target, y_target):
    x_a1, y_a1 = -d/2, 0  # Serwo 1
    x_a2, y_a2 = d/2, 0   # Serwo 2
    
    plt.figure(figsize=(10, 8))
    
    try:
        # Obliczenie kątów używając funkcji IK
        theta1, theta2 = solve_ik(x_target, y_target)
        
        # Położenie przegubów pośrednich
        x_b1 = x_a1 + r1 * math.cos(theta1)
        y_b1 = y_a1 + r1 * math.sin(theta1)
        x_b2 = x_a2 + r1 * math.cos(theta2)  
        y_b2 = y_a2 + r1 * math.sin(theta2)
        
        # Rysowanie konfiguracji robota
        # Pierwsze ramię (czerwone)
        plt.plot([x_a1, x_b1, x_target], [y_a1, y_b1, y_target], 
                color='red', marker='o', linewidth=3, alpha=0.8,
                label=f'Ramię 1 - θ₁={math.degrees(theta1):.1f}°')
        
        # Drugie ramię (niebieskie)
        plt.plot([x_a2, x_b2, x_target], [y_a2, y_b2, y_target], 
                color='blue', marker='o', linewidth=3, alpha=0.8,
                label=f'Ramię 2 - θ₂={math.degrees(theta2):.1f}°')
        
        # Zaznaczenie końcówki
        plt.scatter(x_target, y_target, color='green', s=150, zorder=5, 
                   label=f'Końcówka ({x_target}, {y_target})')
        
        # Podstawa robota (miejsce montażu serw)
        plt.plot([x_a1, x_a2], [y_a1, y_a2], 'k-', linewidth=8, label='Podstawa')
        plt.scatter([x_a1, x_a2], [y_a1, y_a2], color='black', s=150, zorder=5)
        
        # Oznaczenie serw
        plt.text(x_a1, y_a1 - 8, 'Serwo 1', ha='center', va='top', fontsize=11,
                bbox=dict(boxstyle="round,pad=0.3", facecolor='red', alpha=0.7))
        plt.text(x_a2, y_a2 - 8, 'Serwo 2', ha='center', va='top', fontsize=11,
                bbox=dict(boxstyle="round,pad=0.3", facecolor='blue', alpha=0.7))
        
        # Etykiety kątów na ramionach
        mid_x1 = (x_a1 + x_b1) / 2
        mid_y1 = (y_a1 + y_b1) / 2
        plt.text(mid_x1, mid_y1, f'θ₁={-math.degrees(theta1):.0f}°, {rad_to_servo(theta1)}', 
                fontsize=10, ha='center', va='center', 
                fontweight='bold',  
                color='white',      
                bbox=dict(boxstyle="round,pad=0.3", 
                        facecolor='black', 
                        alpha=0.9,
                        edgecolor='black',  
                        linewidth=1.5))     
        
        mid_x2 = (x_a2 + x_b2) / 2
        mid_y2 = (y_a2 + y_b2) / 2
        plt.text(mid_x2, mid_y2, f'θ₂={-math.degrees(theta2):.0f}°, {rad_to_servo(theta2)}',
                fontsize=10, ha='center', va='center', 
                fontweight='bold',  
                color='white',      
                bbox=dict(boxstyle="round,pad=0.3", 
                        facecolor='black', 
                        alpha=0.9,
                        edgecolor='black',  
                        linewidth=1.5))     
        
        plt.axis('equal')
        plt.grid(True, alpha=0.3)
        plt.legend(fontsize=11, loc='upper right')
        plt.title(f'Konfiguracja nogi robota dla punktu ({x_target}, {y_target})', 
                 fontsize=14, pad=20)
        plt.xlabel('X [mm]', fontsize=12)
        plt.ylabel('Y [mm]', fontsize=12)
        
        # Dodanie informacji o parametrach
        textstr = f'Parametry mechaniczne:\n• r₁ = {r1} mm\n• r₂ = {r2} mm\n• d = {d} mm\n\nKąty:\n• θ₁ = {math.degrees(theta1):.1f}°\n• θ₂ = {math.degrees(theta2):.1f}°'
        props = dict(boxstyle='round', facecolor='lightblue', alpha=0.8)
        plt.text(0.02, 0.98, textstr, transform=plt.gca().transAxes, fontsize=10,
                 verticalalignment='top', bbox=props)
        
        plt.tight_layout()
        plt.show()
        
    except (ValueError, ZeroDivisionError) as e:
        print(f"Błąd: Punkt ({x_target}, {y_target}) jest poza zasięgiem robota!")
        # Rysowanie tylko podstawy dla nieosiągalnego punktu
        plt.plot([x_a1, x_a2], [y_a1, y_a2], 'k-', linewidth=8)
        plt.scatter([x_a1, x_a2], [y_a1, y_a2], color='black', s=150, zorder=5)
        plt.scatter(x_target, y_target, color='red', s=150, zorder=5, 
                   label=f'Punkt poza zasięgiem ({x_target}, {y_target})')
        plt.axis('equal')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.title(f'PUNKT POZA ZASIĘGIEM: ({x_target}, {y_target})', color='red')
        plt.xlabel('X [mm]')
        plt.ylabel('Y [mm]')
        plt.show()


# solve_ik(20, -60)
visualize_single_leg_trajectory("swing")
visualize_single_leg_trajectory("stance")
visualize_single_point(20, -60)
# visualize_single_point(-65, -30)

    