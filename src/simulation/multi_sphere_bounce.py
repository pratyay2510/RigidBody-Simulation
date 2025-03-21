# src/simulation/multi_sphere_bounce.py

import os
import numpy as np
import mujoco as mj
import glfw
from src.visualization.multi_sphere_logger import MultiSphereLogger
# ✅ Import centralized step function
from src.physics.step_schemes import custom_step_multi_sphere
from src.config import load_sim_config
from src.viewer.mujoco_viewer import (
    initialize_glfw_window, setup_mujoco_camera,
    register_callbacks, start_main_loop
)

# ✅ Load simulation-specific config for multi-sphere bounce
config = load_sim_config("multi_sphere_bounce")
friction_coefficient = config["FRICTION_COEFFICIENT"]
restitution_coefficient = config["RESTITUTION"]
timestep = config["TIMESTEP"]
camera_settings = config["CAMERA"]
record_video = config["RECORD_VIDEO"]
output_video_path = config["RECORDING_PATH"]

# ✅ Load the MuJoCo model for multiple spheres
xml_path = os.path.join(os.path.dirname(__file__),
                        "../..", "models", "multi_sphere.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# ✅ Initialize logger for tracking the positions of multiple spheres
ball_names = ["ball1", "ball2", "ball3", "ball4"]
logger = MultiSphereLogger(ball_names)

# ✅ Custom keyboard handler: resets environment on SPACE key


def custom_keyboard_handler(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_SPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
        print("Multi-sphere simulation environment reset.")


# ✅ Initialize visualization window and setup camera
window = initialize_glfw_window("Multi-Sphere Rigid Body Simulation")
cam, opt, scene, context = setup_mujoco_camera(model, camera_settings)

# ✅ Register GLFW callbacks, including the custom reset handler
register_callbacks(window, model, data, cam, scene,
                   custom_keyboard_handler=custom_keyboard_handler)

# ✅ Define simulation step wrapper that uses centralized step_schemes function


def multi_sphere_simulation_step(model, data, dt):
    """
    Simulation step for multi-sphere bounce using centralized impulse-based collision resolution.
    Records trajectories using the logger.
    """
    custom_step_multi_sphere(
        model, data, ball_names,
        friction_coefficient=friction_coefficient,
        restitution=restitution_coefficient,
        dt=dt, logger=logger
    )
    return None


# ✅ Start the main simulation loop
start_main_loop(
    window, model, data, cam, opt, scene, context,
    step_function=multi_sphere_simulation_step,
    logger=logger,
    record_video=record_video,  # Enable recording if needed
    video_writer=None
)

# ✅ Save trajectory plots after simulation completion
logger.save_all_plots("data/multi_sphere/plots")
print("Multi-sphere simulation completed and plots saved.")
