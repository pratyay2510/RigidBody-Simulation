import os
import numpy as np
import mujoco as mj
from src.visualization.data_logger import DataLogger
from src.physics.time_integeration import timestep_integration
from src.config import load_sim_config
from src.viewer.mujoco_viewer import (
    initialize_glfw_window, setup_mujoco_camera,
    register_callbacks, start_main_loop
)
import imageio

# ✅ Load simulation-specific configuration
config = load_sim_config("cube_incline")

# Extract simulation parameters from config
friction_coefficient = config["FRICTION_COEFFICIENT"]
restitution = config["RESTITUTION"]
timestep = config["TIMESTEP"]
incline_angle_rad = config["INCLINE_ANGLE_RAD"]
output_video_path = config["RECORDING_PATH"]
camera_settings = config["CAMERA"]
record_video = config["RECORD_VIDEO"]

# ✅ Set object type for this simulation (unique simulation requirement)
obj = 'cube'

# ✅ Dynamically modify XML with incline angle and timestep from config
xml_template_path = os.path.join("models", f"{obj}.xml")
with open(xml_template_path, 'r') as file:
    xml_content = file.read()

xml_content = xml_content.replace("{INCLINE_ANGLE}", str(incline_angle_rad))
xml_content = xml_content.replace("{TIMESTEP}", str(timestep))

# Save modified XML to a temporary file
modified_xml_path = os.path.join("models", f"{obj}_temp.xml")
with open(modified_xml_path, 'w') as file:
    file.write(xml_content)

# ✅ Load the MuJoCo model from the modified XML
model = mj.MjModel.from_xml_path(modified_xml_path)
data = mj.MjData(model)

# ✅ Set initial conditions: cube starts from rest
data.qvel[:6] = 0.0

# ✅ Initialize logger to record simulation height and trajectory
logger = DataLogger()

# ✅ Initialize GLFW window using shared utility
window = initialize_glfw_window("Cube on Inclined Plane Simulation")

# ✅ Set up MuJoCo camera from config using shared utility
cam, opt, scene, context = setup_mujoco_camera(model, camera_settings)

# ✅ Setup video recording if enabled
video_writer = None
if record_video and output_video_path:
    os.makedirs(os.path.dirname(output_video_path), exist_ok=True)
    video_writer = imageio.get_writer(
        output_video_path, fps=30, codec="libx264")

# ✅ Register shared callbacks
register_callbacks(window, model, data, cam, scene)

# ✅ Define simulation step function to pass into the main loop


def cube_incline_step(model, data, dt):
    """
    Single simulation step for cube on inclined plane using timestep integration.
    Returns position for logger.
    """
    pos_new = timestep_integration(
        model, obj, data, dt=dt, restitution=restitution, friction_coeff=friction_coefficient
    )
    return pos_new


# ✅ Start the simulation main loop
start_main_loop(
    window, model, data, cam, opt, scene, context,
    step_function=cube_incline_step,
    logger=logger,
    record_video=record_video,
    video_writer=video_writer
)

# ✅ Save plots after simulation completes
logger.save_plot("data/plots/cube/cube_height_vs_time.png")
logger.save_trajectory_plot_3d("data/plots/cube/cube_3d_trajectory.png")

# ✅ Close video writer if recording was enabled
if video_writer is not None:
    video_writer.close()
    print(f"Simulation recording saved to {output_video_path}")

# ✅ Clean up temporary XML
if os.path.exists(modified_xml_path):
    os.remove(modified_xml_path)
