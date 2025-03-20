import os
import time
import numpy as np
import mujoco as mj
from mujoco.glfw import glfw
from scipy.spatial.transform import Rotation as R
from data_logger import DataLogger
from simulation.collision import compute_collision_impulse_friction
from simulation.physics import apply_impulse_friction
import imageio  # ✅ Added for recording

# --- Global settings ---
last_x, last_y = 0, 0
left_pressed = False
right_pressed = False
viewport_width, viewport_height = 1200, 900
friction_coefficient = 0.5
ball_names = ["ball1", "ball2", "ball3", "ball4"]

# --- Initialize GLFW ---
if not glfw.init():
    raise RuntimeError("Could not initialize GLFW")

window = glfw.create_window(
    viewport_width, viewport_height, "Multiple Bouncing Spheres", None, None)
if not window:
    glfw.terminate()
    raise RuntimeError("Could not create GLFW window")
glfw.make_context_current(window)
glfw.swap_interval(1)

# --- Load Model ---
xml_path = os.path.join(os.path.dirname(__file__), "..",
                        "models", "multi_sphere.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# ✅ Set unique velocities & spins for each sphere
initial_conditions = {
    "ball1": (np.array([2.0, -1.0, 0.0]), np.array([-5.0, 3.0, 0.0])),
    "ball2": (np.array([-2.0, 1.0, 0.0]), np.array([4.0, -2.0, 0.0])),
    "ball3": (np.array([1.0, 2.0, 0.0]), np.array([3.0, 1.0, 0.0])),
    "ball4": (np.array([-1.0, -2.0, 0.0]), np.array([-3.0, -1.0, 0.0]))
}

for ball in ball_names:
    joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, f"{ball}_joint")
    start_idx = joint_id * 6
    data.qvel[start_idx:start_idx +
              3] = initial_conditions[ball][0]     # Linear velocity
    data.qvel[start_idx + 3:start_idx +
              6] = initial_conditions[ball][1]  # Angular velocity


# --- Recording Setup ---
output_video_path = "src/recordings/multi_sphere_bounce.mp4"
os.makedirs(os.path.dirname(output_video_path), exist_ok=True)
video_writer = imageio.get_writer(output_video_path, fps=30, codec='libx264')

# --- Visualization Setup ---
cam = mj.MjvCamera()
opt = mj.MjvOption()
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
cam.azimuth, cam.elevation, cam.distance = 90, -30, 6
cam.lookat = np.array([0.0, 0.0, 0.5])

# --- Main Simulation Loop ---
simulation_time = 0.0
while not glfw.window_should_close(window):
    mj.mj_step(model, data)
    simulation_time += model.opt.timestep

    # ✅ Capture frame for video
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    rgb_buffer = np.zeros((viewport_height, viewport_width, 3), dtype=np.uint8)
    depth_buffer = np.zeros(
        (viewport_height, viewport_width), dtype=np.float32)
    mj.mjr_readPixels(rgb_buffer, depth_buffer, viewport, context)
    frame = np.flipud(rgb_buffer)  # Flip vertically
    video_writer.append_data(frame)

    glfw.swap_buffers(window)
    glfw.poll_events()

# --- Save Video ---
video_writer.close()
print(f"Simulation recording saved to {output_video_path}")

glfw.terminate()
