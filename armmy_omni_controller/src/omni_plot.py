import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

# Constants
DegToRad = math.pi / 180.0
wheel_radius = 0.0175
wheel_offset = 0.038375

def fk_omni(FR, BR, FL, BL):
    l = wheel_offset  # Distance from the center to wheel
    vx = (FR - BR + FL - BL) / 4
    vy = (-FR - BR + FL + BL) / 4
    omega = (-FR - BR - FL - BL) / (4 * l)
    return vx, vy, omega

class OmniWheelSubscriber(Node):
    def __init__(self):
        super().__init__('omni_wheel_visualization')
        self.FR = 0.0
        self.BR = 0.0
        self.FL = 0.0
        self.BL = 0.0

        self.subscription = self.create_subscription(
            Float64MultiArray,
            'forward_velocity_controller/commands',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.FR = msg.data[0] * wheel_radius
        self.BR = msg.data[1] * wheel_radius
        self.FL = msg.data[2] * wheel_radius
        self.BL = msg.data[3] * wheel_radius

def setup_plot(ax):
    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)
    ax.axhline(0, color='black', linewidth=0.5)
    ax.axvline(0, color='black', linewidth=0.5)
    ax.grid(color='gray', linestyle='--', linewidth=0.5)
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Omnidirectional Robot Kinematics Visualization")
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")

    wheel_positions = {
        "FR": (1, 1, -45),
        "BR": (1, -1, 225),
        "FL": (-1, 1, 45),
        "BL": (-1, -1, 135)
    }

    quivers = []
    texts = []
    for _ in wheel_positions:
        quiver = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='blue')
        quivers.append(quiver)
        text = ax.text(0, 0, "", fontsize=10, color='blue', ha='center')
        texts.append(text)

    vx_arrow = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='red')
    vy_arrow = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='green')
    total_velocity_arrow = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='purple')
    omega_text = ax.text(0, -5.0, "", fontsize=12, color='orange', ha='center')
    
    vel_texts = []
    for _ in range(3):
        text = ax.text(0, 0, "", fontsize=10, color='green', ha='center')
        vel_texts.append(text)

    return quivers, texts, vx_arrow, vy_arrow, total_velocity_arrow, omega_text, wheel_positions , vel_texts

def update_plot(frame, node, quivers, texts, vx_arrow, vy_arrow, total_velocity_arrow, omega_text, wheel_positions, vel_texts):
    vx, vy, omega = fk_omni(node.FR, node.BR, node.FL, node.BL)
    for i, (label, (x, y, angle)) in enumerate(wheel_positions.items()):
        speed = [node.FR, node.BR, node.FL, node.BL][i]
        length_x = speed * math.cos(angle * DegToRad)
        length_y = speed * math.sin(angle * DegToRad)
        quivers[i].set_UVC(length_x, length_y)
        quivers[i].set_offsets([x, y])
        texts[i].set_position((x * 1.2, y * 1.2))
        texts[i].set_text(f"{speed:.2f}")

    vx_arrow.set_UVC(vx, 0)
    vy_arrow.set_UVC(0, vy)
    total_velocity_arrow.set_UVC(vx, vy)

    vel_texts[0].set_text(f"vx: {vx:.2f}")
    vel_texts[0].set_position((-3, -5.0)) 
    vel_texts[1].set_text(f"vy: {vy:.2f}")
    vel_texts[1].set_position((-1, -5.0)) 
    vel_texts[2].set_text(f"v_total: {math.sqrt(vx**2 + vy**2):.2f}")
    vel_texts[2].set_position((1, -5.0)) 
    omega_text.set_text(f"ω: {omega:.2f} rad/s")
    omega_text.set_position((3, -5.0)) 

def main():
    rclpy.init()
    node = OmniWheelSubscriber()

    # Start the ROS 2 node in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # Set up the plot
    fig, ax = plt.subplots(figsize=(6, 6))
    quivers, texts, vx_arrow, vy_arrow, total_velocity_arrow, omega_text, wheel_positions, vel_texts = setup_plot(ax)

    # Real-time visualization with FuncAnimation
    ani = FuncAnimation(fig, update_plot, fargs=(
        node, quivers, texts, vx_arrow, vy_arrow, total_velocity_arrow, omega_text, wheel_positions , vel_texts
    ), interval=100)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
