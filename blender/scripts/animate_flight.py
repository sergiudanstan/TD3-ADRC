"""
animate_flight.py - Blender script for TD3-ADRC quadrotor visualization

Creates animated 3D visualization of quadrotor flight data exported from MATLAB.

Usage:
    blender --background drone_scene.blend --python scripts/animate_flight.py
    blender --background --python scripts/animate_flight.py -- --data path/to/flight.json

Reference: Cai et al. (2026) "TD3-ADRC for Quadrotors"
"""

import bpy
import json
import math
import os
import sys
from mathutils import Vector, Euler


def load_flight_data(filepath):
    """Load flight data from JSON file exported by MATLAB."""
    with open(filepath, 'r') as f:
        data = json.load(f)
    return data


def create_drone_mesh(name="Quadrotor", arm_length=0.225):
    """Create a simple quadrotor mesh."""

    # Create main body (cube)
    bpy.ops.mesh.primitive_cube_add(size=0.1, location=(0, 0, 0))
    body = bpy.context.active_object
    body.name = f"{name}_body"
    body.scale = (1, 1, 0.3)

    # Create arms
    arms = []
    for i, angle in enumerate([45, 135, 225, 315]):
        rad = math.radians(angle)
        x = arm_length * math.cos(rad)
        y = arm_length * math.sin(rad)

        bpy.ops.mesh.primitive_cylinder_add(radius=0.01, depth=arm_length*2, location=(0, 0, 0))
        arm = bpy.context.active_object
        arm.name = f"{name}_arm_{i}"
        arm.rotation_euler = (0, math.pi/2, rad)
        arm.location = (x/2, y/2, 0)
        arms.append(arm)

    # Create motor/propeller discs
    motors = []
    for i, angle in enumerate([45, 135, 225, 315]):
        rad = math.radians(angle)
        x = arm_length * math.cos(rad)
        y = arm_length * math.sin(rad)

        bpy.ops.mesh.primitive_cylinder_add(radius=0.08, depth=0.02, location=(x, y, 0.02))
        motor = bpy.context.active_object
        motor.name = f"{name}_motor_{i}"
        motors.append(motor)

    # Parent all parts to body
    for obj in arms + motors:
        obj.parent = body

    # Create material
    mat = bpy.data.materials.new(name=f"{name}_material")
    mat.diffuse_color = (0.2, 0.2, 0.8, 1.0)  # Blue
    body.data.materials.append(mat)

    return body


def animate_drone(drone_obj, data, fps=60, position_offset=(0, 0, 0)):
    """Animate drone using flight data."""

    time = data.get('time', [])
    positions = data.get('position', [])
    euler_angles = data.get('euler_angles', [])

    if not time:
        print("Warning: No time data found")
        return

    # Set frame range
    bpy.context.scene.frame_start = 0
    bpy.context.scene.frame_end = int(time[-1] * fps) + 1

    # Animate each frame
    for i, t in enumerate(time):
        frame = int(t * fps)

        # Position
        if i < len(positions):
            pos = positions[i]
            if isinstance(pos, list):
                drone_obj.location = (
                    pos[0] + position_offset[0],
                    pos[1] + position_offset[1],
                    pos[2] + position_offset[2]
                )
            drone_obj.keyframe_insert(data_path='location', frame=frame)

        # Rotation (Euler angles: roll, pitch, yaw)
        if i < len(euler_angles):
            euler = euler_angles[i]
            if isinstance(euler, list):
                # MATLAB uses ZYX convention, Blender uses XYZ by default
                drone_obj.rotation_euler = Euler((euler[0], euler[1], euler[2]), 'XYZ')
            drone_obj.keyframe_insert(data_path='rotation_euler', frame=frame)


def animate_motors(drone_obj, motor_speeds, frame):
    """Animate motor speed visualization (emission intensity)."""
    if motor_speeds is None:
        return

    # Find motor objects
    for i in range(4):
        motor_name = f"{drone_obj.name.replace('_body', '')}_motor_{i}"
        motor = bpy.data.objects.get(motor_name)
        if motor and motor.data.materials:
            mat = motor.data.materials[0]
            if mat.use_nodes:
                # Set emission based on motor speed
                emission_node = mat.node_tree.nodes.get("Emission")
                if emission_node:
                    speed = motor_speeds[i] if i < len(motor_speeds) else 0
                    # Normalize speed (assuming max ~8000 rad/s)
                    intensity = min(speed / 8000, 1.0) * 5
                    emission_node.inputs['Strength'].default_value = intensity
                    emission_node.inputs['Strength'].keyframe_insert(data_path='default_value', frame=frame)


def create_trajectory_trail(data, name="Trail", color=(1, 0, 0, 1)):
    """Create a curve showing the flight trajectory."""

    positions = data.get('position', [])
    if not positions:
        return None

    # Create curve
    curve_data = bpy.data.curves.new(name=name, type='CURVE')
    curve_data.dimensions = '3D'

    # Create spline
    spline = curve_data.splines.new('NURBS')
    spline.points.add(len(positions) - 1)

    for i, pos in enumerate(positions):
        if isinstance(pos, list):
            spline.points[i].co = (pos[0], pos[1], pos[2], 1)

    # Create object
    curve_obj = bpy.data.objects.new(name, curve_data)
    bpy.context.collection.objects.link(curve_obj)

    # Material
    mat = bpy.data.materials.new(name=f"{name}_material")
    mat.diffuse_color = color
    curve_obj.data.materials.append(mat)

    # Curve settings
    curve_data.bevel_depth = 0.005

    return curve_obj


def create_comparison_scene(data_files, controller_names, spacing=5.0):
    """Create scene with multiple drones for controller comparison."""

    drones = []
    trails = []

    # Colors for each controller
    colors = [
        (0.2, 0.2, 0.8, 1.0),  # Blue - PID
        (0.2, 0.8, 0.2, 1.0),  # Green - ADRC
        (0.8, 0.2, 0.8, 1.0),  # Magenta - DDPG-ADRC
        (0.8, 0.2, 0.2, 1.0),  # Red - TD3-ADRC
    ]

    for i, (data_file, ctrl_name) in enumerate(zip(data_files, controller_names)):
        # Load data
        data = load_flight_data(data_file)

        # Create drone
        drone = create_drone_mesh(name=ctrl_name)
        drone.location.x = i * spacing
        drones.append(drone)

        # Set drone color
        if drone.data.materials:
            drone.data.materials[0].diffuse_color = colors[i % len(colors)]

        # Animate drone
        animate_drone(drone, data, fps=60, position_offset=(i * spacing, 0, 0))

        # Create trail
        trail = create_trajectory_trail(data, name=f"{ctrl_name}_trail", color=colors[i % len(colors)])
        if trail:
            trail.location.x = i * spacing
            trails.append(trail)

        # Add text label
        bpy.ops.object.text_add(location=(i * spacing, -1, -0.5))
        text = bpy.context.active_object
        text.data.body = ctrl_name
        text.data.size = 0.3
        text.rotation_euler = (math.pi/2, 0, 0)

    return drones, trails


def setup_camera_follow(camera, target, distance=5):
    """Setup camera to follow drone."""

    if camera is None:
        bpy.ops.object.camera_add(location=(distance, -distance, distance))
        camera = bpy.context.active_object

    # Track constraint
    constraint = camera.constraints.new(type='TRACK_TO')
    constraint.target = target
    constraint.track_axis = 'TRACK_NEGATIVE_Z'
    constraint.up_axis = 'UP_Y'

    return camera


def setup_lighting():
    """Setup scene lighting."""

    # Sun lamp
    bpy.ops.object.light_add(type='SUN', location=(10, 10, 20))
    sun = bpy.context.active_object
    sun.data.energy = 3

    # Fill light
    bpy.ops.object.light_add(type='AREA', location=(-5, -5, 10))
    fill = bpy.context.active_object
    fill.data.energy = 100

    return sun, fill


def render_animation(output_path, fps=60, resolution=(1920, 1080)):
    """Render animation to video file."""

    scene = bpy.context.scene
    scene.render.fps = fps
    scene.render.resolution_x = resolution[0]
    scene.render.resolution_y = resolution[1]

    # Output settings
    scene.render.image_settings.file_format = 'FFMPEG'
    scene.render.ffmpeg.format = 'MPEG4'
    scene.render.ffmpeg.codec = 'H264'
    scene.render.filepath = output_path

    # Render
    bpy.ops.render.render(animation=True)


def main():
    """Main function for command-line execution."""

    # Parse arguments
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        argv = []

    # Default paths
    data_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    results_dir = os.path.join(os.path.dirname(data_dir), 'results', 'animations', 'comparison')

    # Check for manifest file
    manifest_path = os.path.join(results_dir, 'manifest.json')
    if os.path.exists(manifest_path):
        with open(manifest_path, 'r') as f:
            manifest = json.load(f)

        controller_names = manifest.get('controllers', [])
        data_files = [os.path.join(results_dir, f) for f in manifest.get('files', [])]

        print(f"Loading comparison data for: {controller_names}")
        create_comparison_scene(data_files, controller_names)

    else:
        # Single file mode
        if len(argv) >= 2 and argv[0] == '--data':
            data_path = argv[1]
        else:
            data_path = os.path.join(results_dir, 'td3_adrc_flight.json')

        if os.path.exists(data_path):
            print(f"Loading: {data_path}")
            data = load_flight_data(data_path)
            drone = create_drone_mesh("Quadrotor")
            animate_drone(drone, data)
            setup_camera_follow(None, drone)
        else:
            print(f"Data file not found: {data_path}")
            print("Creating demo scene...")
            drone = create_drone_mesh("Demo_Drone")

    # Setup scene
    setup_lighting()

    print("Animation setup complete!")


if __name__ == "__main__":
    main()
