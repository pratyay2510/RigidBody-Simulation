import os
import numpy as np
import mujoco as mj
from src.visualization.data_logger import DataLogger
from src.config import load_sim_config
from src.viewer.mujoco_viewer import (
    initialize_glfw_window, setup_mujoco_camera,
    register_callbacks, start_main_loop
)

# ✅ Load centralized config for single sphere bounce to reuse camera settings
config = load_sim_config("single_sphere_bounce")
camera_settings = config["CAMERA"]

# ✅ Load the MuJoCo model (sphere model for built-in simulation)
xml_path = os.path.join("models", "sphere.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# ✅ Set simulation-specific initial conditions (not part of global config)
data.qpos[2] = 1.0  # Start from height 1.0
data.qvel[3:6] = np.zeros(3)  # No initial angular velocity

# ✅ Create data logger to record height vs. time
logger = DataLogger()

# ✅ Define simulation step using Mujoco's built-in integrator (no custom impulses)


def mujoco_builtin_step(model, data, dt):
    mj.mj_step(model, data)  # Built-in Mujoco physics step
    simulation_time = data.time
    # Log height of the sphere
    logger.record(simulation_time, data.qpos[2])
    return None  # No position return needed


# ✅ Initialize GLFW window using shared viewer
window = initialize_glfw_window(
    "MuJoCo Built-in Simulation (No Custom Impulse)")

# ✅ Setup camera from centralized config using shared viewer
cam, opt, scene, context = setup_mujoco_camera(model, camera_settings)

# ✅ Register callbacks (mouse, scroll, and camera controls)
register_callbacks(window, model, data, cam, scene)

# ✅ Start simulation loop using shared main loop
start_main_loop(
    window, model, data, cam, opt, scene, context,
    step_function=mujoco_builtin_step,
    logger=None,  # Logger updates are handled inside the step function
    record_video=False  # No recording for this comparison simulation
)

# ✅ Save results: height vs. time plot
output_plot_path = "data/plots/height_vs_time_builtin.png"
logger.save_plot(output_plot_path)
print(f"Built-in simulation completed. Plot saved at: {output_plot_path}")
