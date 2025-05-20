import yaml
import math
import numpy as np
import matplotlib.pyplot as plt

def visualize_omni_wheel_y_facing(yaml_path):
    # Load yaml config
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    tangent_radius = config.get('tangent_radius', 20)  # mm
    positions = config.get('position', [])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Sphere mesh for rollers
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)
    x_sphere = np.outer(np.cos(u), np.sin(v))
    y_sphere = np.outer(np.sin(u), np.sin(v))
    z_sphere = np.outer(np.ones(np.size(u)), np.cos(v))

    sphere_radius = tangent_radius * 0.4  # adjust sphere size

    for pos in positions:
        theta_deg, offset_y, rot_axis = pos
        print(pos)
        theta_rad = math.radians(theta_deg)

        # Roller center on rim circle in X-Z plane
        x_center = tangent_radius * math.cos(theta_rad)
        y_center = offset_y * 1000  # adjust scale if needed
        z_center = tangent_radius * math.sin(theta_rad)

        # # Draw roller sphere
        # ax.plot_surface(
        #     x_center + sphere_radius * x_sphere,
        #     y_center + sphere_radius * y_sphere,
        #     z_center + sphere_radius * z_sphere,
        #     color='cyan',
        #     alpha=0.7
        # )

        # Rotation axis vector from config, use directly
        rot_axis = np.array(rot_axis)
        # Normalize vector for arrow length consistency
        rot_axis_norm = rot_axis / np.linalg.norm(rot_axis)

        # Draw arrow starting at roller center, pointing along rot_axis
        ax.quiver(
            x_center, y_center, z_center,
            rot_axis_norm[0], rot_axis_norm[1], rot_axis_norm[2],
            length=sphere_radius * 2,
            color='red',
            normalize=False
        )

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm) - Wheel Facing Direction')
    ax.set_zlabel('Z (mm)')
    ax.set_title('Omni Wheel Rollers (Wheel facing Y axis)')

    max_range = tangent_radius + sphere_radius * 3
    ax.set_box_aspect([1,1,1])
    ax.auto_scale_xyz([-max_range, max_range], [-max_range, max_range], [-max_range, max_range])

    plt.show()


if __name__ == "__main__":
    visualize_omni_wheel_y_facing("omni_wheel_config.yaml")
