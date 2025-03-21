# src/simulation/ball_collision.py

import os
import numpy as np
import mujoco as mj
import imageio
import glfw
from src.visualization.data_logger import DataLogger
from src.config import load_sim_config
# ✅ Import centralized step logic
from src.physics.step_schemes import step_with_custom_collisions
from src.viewer.mujoco_viewer import (
    initialize_glfw_window, setup_mujoco_camera,
    register_callbacks, start_main_loop
)

# ✅ Load simulation-specific config
config = load_sim_config("ball_collision")
friction_coefficient = config["FRICTION_COEFFICIENT"]
restitution = config["RESTITUTION"]
timestep = config["TIMESTEP"]
camera_settings = config["CAMERA"]
output_video_path = config["RECORDING_PATH"]
record_video = config["RECORD_VIDEO"]

# ✅ Hardcoded geometry parameter (can be moved to config if needed)
ball_radius = 0.1

# ✅ Load MuJoCo model for ball collision simulation
xml_path = os.path.join("models", "ball_collision.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# ✅ Set custom initial conditions for the two balls
data.qpos[0:3] = np.array([-1.0, 0.0, 1.0])  # Ball 1 position
data.qpos[7:10] = np.array([1.0, 0.0, 1.0])  # Ball 2 position
data.qvel[0:3] = np.array([1.0, 0.0, 0.5])   # Ball 1 initial velocity
data.qvel[6:9] = np.array([-1.0, 0.0, 0.5])  # Ball 2 initial velocity

# ✅ Compute inverse inertia tensor for each ball


def compute_inverse_inertia(mass, radius):
    inertia_val = (2.0 / 5.0) * mass * radius ** 2
    return np.eye(3) / inertia_val


ball1_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "ball1")
ball2_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "ball2")
mass1, mass2 = model.body_mass[ball1_id], model.body_mass[ball2_id]
I_inv_ball1 = compute_inverse_inertia(mass1, ball_radius)
I_inv_ball2 = compute_inverse_inertia(mass2, ball_radius)

# ✅ Setup data loggers for both balls
logger_ball1 = DataLogger()
logger_ball2 = DataLogger()

# ✅ Running toggle (start/stop simulation)
running = False

# ✅ Custom keyboard handler to toggle simulation pause/resume


def custom_keyboard_handler(window, key, scancode, act, mods):
    global running
    if act == glfw.PRESS:
        if key == glfw.KEY_SPACE:
            running = not running
            print("Simulation running" if running else "Simulation paused")

# ✅ Define simulation step wrapper using centralized step_schemes


def ball_collision_step(model, data, dt):
    """
    Executes one simulation step using custom impulse-based collision resolution
    from the centralized step_schemes.py.
    Logs positions of both balls for plotting after simulation ends.
    """
    if running:
        ball1_pos, ball2_pos = step_with_custom_collisions(
            model, data, ball_radius, mass1, mass2, I_inv_ball1, I_inv_ball2,
            friction_coefficient, restitution, dt=dt
        )
        simulation_time = data.time
        logger_ball1.record(
            simulation_time, ball1_pos[2], ball1_pos[0], ball1_pos[1])
        logger_ball2.record(
            simulation_time, ball2_pos[2], ball2_pos[0], ball2_pos[1])
    return None


# ✅ Initialize GLFW window
window = initialize_glfw_window("Two Ball Collision Simulation")

# ✅ Setup MuJoCo camera using config settings
cam, opt, scene, context = setup_mujoco_camera(model, camera_settings)

# ✅ Setup video recording if enabled
video_writer = None
if record_video and output_video_path:
    os.makedirs(os.path.dirname(output_video_path), exist_ok=True)
    video_writer = imageio.get_writer(
        output_video_path, fps=30, codec="libx264"
    )

# ✅ Register callbacks (mouse, keyboard, camera)
register_callbacks(window, model, data, cam, scene,
                   custom_keyboard_handler=custom_keyboard_handler)

# ✅ Start simulation loop with centralized step function
start_main_loop(
    window, model, data, cam, opt, scene, context,
    step_function=ball_collision_step,
    logger=None,  # using separate loggers per ball
)
