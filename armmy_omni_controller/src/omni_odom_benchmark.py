import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import math

class OdomGroundTruthPlotter(Node):
    def __init__(self):
        super().__init__('odom_ground_truth_plotter')

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/gazebo_ground_truth/odom', self.ground_truth_callback, 10)

        # Data storage: separate lists for x and y coordinates
        self.odom_x = []
        self.odom_y = []
        self.odom_heading = 0.00
        self.odom_vx = 0.00
        self.odom_vy = 0.00
        self.odom_ang_vel = 0.00

        self.ground_truth_x = []
        self.ground_truth_y = []
        self.ground_truth_heading = 0.00
        self.gt_vx = 0.00
        self.gt_vy = 0.00
        self.gt_ang_vel = 0.00

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.odom_x.append(position.x)
        self.odom_y.append(position.y)

        # Convert quaternion to yaw
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_heading = yaw  # Store the yaw

        self.odom_vx = msg.twist.twist.linear.x
        self.odom_vy = msg.twist.twist.linear.y
        self.odom_ang_vel = msg.twist.twist.angular.z

    def ground_truth_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.ground_truth_x.append(position.x)
        self.ground_truth_y.append(position.y)

        # Convert quaternion to yaw
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.ground_truth_heading = yaw  # Store the yaw

        self.gt_vx = msg.twist.twist.linear.x
        self.gt_vy = msg.twist.twist.linear.y
        self.gt_ang_vel = msg.twist.twist.angular.z

def setup_plot(ax):
    ax.axhline(0, color='black', linewidth=0.5)
    ax.axvline(0, color='black', linewidth=0.5)
    ax.grid(color='gray', linestyle='--', linewidth=0.5)
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Omnidirectional Odom vs Ground truth")
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")

def update_plot(frame, node):
    plt.cla()

    plt.plot([-y for y in node.odom_y], node.odom_x, label='Odom')  # Swap and invert
    plt.plot([-y for y in node.ground_truth_y], node.ground_truth_x, label='Ground truth')  # Swap and invert

    # Draw arrow for the latest odom position
    if node.odom_x and node.odom_y and node.odom_heading:
        x = node.odom_x[-1]
        y = node.odom_y[-1]
        yaw = node.odom_heading
        dx = 0.02 * math.cos(yaw)  # Scale arrow length
        dy = 0.02 * math.sin(yaw)
        plt.arrow(-y, x, -dy, dx, head_width=0.01, head_length=0.02, fc='blue', ec='blue')  # Swap and invert
    
    # Draw GT arrow for the latest odom position
    if node.ground_truth_x and node.ground_truth_y and node.ground_truth_heading:
        x = node.ground_truth_x[-1]
        y = node.ground_truth_y[-1]
        yaw = node.ground_truth_heading
        dx = 0.02 * math.cos(yaw)  # Scale arrow length
        dy = 0.02 * math.sin(yaw)
        plt.arrow(-y, x, -dy, dx, head_width=0.01, head_length=0.02, fc='orange', ec='orange')  # Swap and invert
    
    # Create text box to display velocities and angular velocities
    textstr = '\n'.join((
        f'Odometry (vx, vy, ang_vel): ({node.odom_vx:.2f}, {node.odom_vy:.2f}, {node.odom_ang_vel:.2f})',
        f'Ground Truth (vx, vy, ang_vel): ({node.gt_vx:.2f}, {node.gt_vy:.2f}, {node.gt_ang_vel:.2f})'
    ))
    
    # Add text box to the plot
    plt.gca().text(0.05, 0.05, textstr, transform=plt.gca().transAxes, fontsize=10, verticalalignment='bottom', bbox=dict(facecolor='white', alpha=0.5))

    plt.legend(loc='upper left')
    plt.tight_layout()


def main():
    rclpy.init()
    node = OdomGroundTruthPlotter()

    # Start the ROS 2 node in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # Set up the plot
    fig, ax = plt.subplots(figsize=(6, 6))
    setup_plot(ax)

    # Real-time visualization with FuncAnimation
    ani = FuncAnimation(fig, update_plot, fargs=(
        node,
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
