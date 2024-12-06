import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # Assuming wheel speed is of type Float32
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants
DegToRad = math.pi / 180.0

# Visualization function
def visualize_fk(FR, BR, FL, BL):
    # Get the FK results (Assuming you have a function to compute vx, vy, omega)
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
    for label, (x, y, angle, speed) in wheel_positions.items():
        length_x = speed * math.cos(angle * DegToRad)
        length_y = speed * math.sin(angle * DegToRad)
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

# ROS 2 Node to subscribe to wheel speed data
class OmniWheelSubscriber(Node):
    def __init__(self):
        super().__init__('omni_wheel_visualization')
        
        # Initialize wheel speed data
        self.FR = 0.0
        self.BR = 0.0
        self.FL = 0.0
        self.BL = 0.0
        
        # Subscriber to /omni_controller topic (example topic)
        self.subscription = self.create_subscription(
            WheelSpeeds,  # Replace with your message type
            '/omni_controller',
            self.listener_callback,
            10
        )
        
        # Set up real-time visualization with FuncAnimation
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.ani = FuncAnimation(self.fig, self.update_plot, blit=False, interval=100)
        plt.show()

    def listener_callback(self, msg):
        # Update wheel speed data from the message
        self.FR = msg.FR
        self.BR = msg.BR
        self.FL = msg.FL
        self.BL = msg.BL
        
    def update_plot(self, frame):
        # Call the visualization function with updated wheel speeds
        visualize_fk(self.FR, self.BR, self.FL, self.BL)

def main():
    rclpy.init(args=None)
    
    omni_wheel_subscriber = OmniWheelSubscriber()
    
    # Run the ROS 2 node
    rclpy.spin(omni_wheel_subscriber)
    
    omni_wheel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
