import math
import matplotlib.pyplot as plt

DegToRad = 2 * math.pi / 360

def ik_omni4wheel_arm(vx,vy,omega):
    ans = []
    ans.append(( vx - vy - omega)) #FR
    ans.append((-vx - vy - omega)) #BR
    ans.append(( vx + vy - omega)) #FL
    ans.append((-vx + vy - omega)) #BL
    return ans

def fk_omni(FR, BR, FL, BL):
    # Constants for the geometry of the robot (assuming a square robot layout)
    l = 1  # distance from center to wheel (in meters, adjust as needed)
    # Convert wheel velocities to linear and angular velocities
    vx = (FR - BR + FL - BL) / 4
    vy = (-FR - BR + FL + BL) / 4
    omega = (-FR + BR - FL + BL) / (4 * l)
    return vx, vy, omega

def visualize_fk(FR, BR, FL, BL):
    # Get the FK results
    vx, vy, omega = fk_omni(FR, BR, FL, BL)
    v_total = math.sqrt(vx**2 + vy**2)
    
    # Wheel positions and speeds
    wheel_positions = {
        "FR": (1, 1,  -45 , FR),  # Front-Right
        "BR": (1, -1, 225 , BR), # Back-Right
        "FL": (-1, 1, 45 , FL), # Front-Left
        "BL": (-1, -1,135, BL) # Back-Left
    }
    
    # Create the plot
    fig, ax = plt.subplots(figsize=(6, 6))
    
    # Plot the omni-wheel speeds as arrows
    for label, (x, y,angle, speed) in wheel_positions.items():
        length_x = speed*math.cos(angle*DegToRad)
        length_y = speed*math.sin(angle*DegToRad)
        # print(f"{label} : ({x} , {y})   ({length_x} , {length_y})")
        ax.quiver(x, y, length_x, length_y, angles='xy', scale_units='xy', scale=1, color='blue', label=f'{label}: {speed:.2f}')
        ax.text(x * 1.2, y * 1.2, f"{speed:.2f}", fontsize=10, color='blue', ha='center')
    
    # Plot the center velocity vectors
    ax.quiver(0, 0, vx, 0, angles='xy', scale_units='xy', scale=1, color='red', label=f'vx: {vx:.2f}')
    ax.quiver(0, 0, 0, vy, angles='xy', scale_units='xy', scale=1, color='green', label=f'vy: {vy:.2f}')
    ax.quiver(0, 0, vx, vy, angles='xy', scale_units='xy', scale=1, color='purple', label=f'v_total: {v_total:.2f}')
    
    # Annotate angular velocity
    ax.text(0, -1.5, f"ω: {omega:.2f} rad/s", fontsize=12, color='orange', ha='center')
    
    # Configure the plot
    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)
    ax.axhline(0, color='black', linewidth=0.5)
    ax.axvline(0, color='black', linewidth=0.5)
    ax.grid(color='gray', linestyle='--', linewidth=0.5)
    ax.set_aspect('equal', adjustable='box')
    
    plt.title("Omnidirectional Robot Kinematics Visualization")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.legend()
    plt.show()

# Example usage
# FR, BR, FL, BL = (1,-1,-1,1) #FORWARD
# FR, BR, FL, BL = (-1,-1,-1,-1) # RIGHT
# FR, BR, FL, BL = (0,-1,-1,0) # RIGHT - FORWAR
# FR, BR, FL, BL = (-1, 1, -1, 1) # Rotatation CW

# FR, BR, FL, BL = (0,0,0,1)
# visualize_fk(FR, BR, FL, BL)

# print(f"1,0,0 : {ik_omni4wheel_arm(1,0,0)}")
# print(f"0,1,0 : {ik_omni4wheel_arm(0,1,0)}")
# print(f"1,1,0 : {ik_omni4wheel_arm(1,1,0)}")
# print(f"0,0,1 : {ik_omni4wheel_arm(0,0,1)}")

# ans = ik_omni4wheel_arm(-1 ,0 , 0)
# ans = ik_omni4wheel_arm(0 ,1 , 0)
ans = ik_omni4wheel_arm(0 ,0 , 1)
# ans = ik_omni4wheel_arm(1 ,1 , 1)

print(ans)
visualize_fk(*ans)
