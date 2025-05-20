import yaml
import math

class InlineList(list):
    pass

def inline_list_representer(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)

yaml.add_representer(InlineList, inline_list_representer)

def generate_omni_wheel_yaml(offsets_with_angles, rollers_per_layer,roller_weight, tangent_radius, wheel_radius = "** set wheel radius **", output_path="omni_wheel_config.yaml"):
    """
    offsets_with_angles: list of [offset_translation (float), offset_angle (degrees float)]
    """
    positions = []

    for (offset_trans, offset_angle) in offsets_with_angles:
        for i in range(rollers_per_layer):
            # Calculate theta for each roller plus the offset_angle for the layer
            base_theta = 360.0 * i / rollers_per_layer
            theta = base_theta + offset_angle 
            angle_rad = math.radians(theta + 90) 
            
            # Rotation axis: x, 0, z plane
            rot_axis = [math.cos(angle_rad), 0.0, math.sin(angle_rad)]
            
            position_entry = InlineList([
                round(theta % 360, 3),           # keep theta normalized to [0,360)
                round(offset_trans, 6),
                InlineList([round(c, 6) for c in rot_axis])
            ])
            positions.append(position_entry)

    config = {
        'wheel_radius': wheel_radius,
        'tangent_radius': tangent_radius,
        'roller_weight': roller_weight,
        'roller_count': rollers_per_layer*len(offsets_with_angles),
        'position': positions
    }

    with open(output_path, 'w') as f:
        yaml.dump(config, f, sort_keys=False)

    print(f"✅ YAML saved to {output_path}")

# Example usage
if __name__ == "__main__":
    offsets_with_angles = [
        [0.004625, 0],        # Layer 2: translation offset, 45 degree angular offset
        [0.013875, 45]        # Layer 1: translation offset, no angular offset
    ]
    generate_omni_wheel_yaml(
        offsets_with_angles=offsets_with_angles,
        rollers_per_layer=4,
        roller_weight=0.01,
        tangent_radius=0.020,
        wheel_radius=0.035,
        output_path="omni_config.yml"
    )
