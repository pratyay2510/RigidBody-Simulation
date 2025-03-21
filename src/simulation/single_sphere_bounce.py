import os
import numpy as np
import mujoco as mj
from src.visualization.data_logger import DataLogger
from src.physics.collision import custom_step_with_impulse_collision_friction
from src.config import load_sim_config  # Load centralized config
from src.viewer.mujoco_viewer import (
    initialize_glfw_window, setup_mujoco_camera,
    register_callbacks, start_main_loop
)
import imageio

# --- ✅ Load simulation-specific config ---
config = load_sim_config("single_sphere_bounce")

# Extract parameters
friction_coefficient = config["FRICTION_COEFFICIENT"]
restitution = config["RESTITUTION"]
timestep = config["TIMESTEP"]
incline_angle_rad = config["INCLINE_ANGLE_RAD"]
camera_settings = config["CAMERA"]
output_video_path = config["RECORDING_PATH"]
record_video = config["RECORD_VIDEO"]

# --- ✅ Load and modify the XML model dynamically with incline and timestep ---
model_template_path = os.path.join("models", "sphere.xml")
with open(model_template_path, "r") as f:
    xml_content = f.read()
xml_content = xml_content.replace("{INCLINE_ANGLE}", str(incline_angle_rad))
xml_content = xml_content.replace("{TIMESTEP}", str(timestep))

temp_model_path = os.path.join("models", "sphere_temp.xml")
with open(temp_model_path, "w") as f:
    f.write(xml_content)

model = mj.MjModel.from_xml_path(temp_model_path)
data = mj.MjData(model)

# ✅ Set initial conditions (unique simulation-specific)
data.qvel[:3] = np.array([0.0, 0.0, 0.0])  # No initial linear velocity
data.qvel[3:6] = np.array([2.0, 2.0, 0.0])  # Initial spin

# ✅ Setup logger
logger = DataLogger()

# ✅ Initialize GLFW window (using common handler)
window = initialize_glfw_window("Single Sphere Bounce Simulation")

# ✅ Setup MuJoCo camera from config
cam, opt, scene, context = setup_mujoco_camera(model, camera_settings)

# ✅ Setup video recording if enabled
video_writer = None
if record_video and output_video_path:
    os.makedirs(os.path.dirname(output_video_path), exist_ok=True)
    video_writer = imageio.get_writer(
        output_video_path, fps=30, codec="libx264")

# ✅ Register callbacks (reuse the viewer's callback registration)
register_callbacks(window, model, data, cam, scene)

# ✅ Define simulation step function for passing to the loop


def sphere_simulation_step(model, data, dt):
    pos = custom_step_with_impulse_collision_friction(
        model, "sphere", data, dt=dt,
        restitution=restitution, friction_coeff=friction_coefficient
    )
    return pos  # Will be used for logging by the main loop


# ✅ Start the simulation loop (using common loop runner)
start_main_loop(
    window, model, data, cam, opt, scene, context,
    step_function=sphere_simulation_step,
    logger=logger,
    record_video=record_video,
    video_writer=video_writer
)

# ✅ Save plots after simulation
logger.save_plot("data/plots/single_sphere/height_vs_time.png")
logger.save_trajectory_plot_3d("data/plots/single_sphere/3d_trajectory.png")

# ✅ Close video writer if recording
if video_writer is not None:
    video_writer.close()
    print(f"Simulation recording saved to {output_video_path}")

# ✅ Cleanup: delete temporary XML
if os.path.exists(temp_model_path):
    os.remove(temp_model_path)
