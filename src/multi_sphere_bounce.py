import os
import numpy as np
import mujoco as mj
from mujoco.glfw import glfw
from scipy.spatial.transform import Rotation as R
from simulation.collision import compute_collision_impulse_friction
from simulation.physics import apply_impulse_friction
from multi_sphere_logger import MultiSphereLogger  # ✅ New import
import imageio

# --- Global settings ---
last_x, last_y = 0, 0
left_pressed = False
right_pressed = False
viewport_width, viewport_height = 1200, 900
friction_coefficient = 0.5
ball_names = ["ball1", "ball2", "ball3", "ball4"]
dt = 0.01

# --- Initialize GLFW ---
if not glfw.init():
    raise RuntimeError("Could not initialize GLFW")

window = glfw.create_window(
    viewport_width, viewport_height, "Multiple Bouncing Spheres with Custom Physics", None, None)
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

# --- Helper: Compute inertia tensor in world frame ---


def compute_inertia_tensor_world(inertia_diag, q):
    rot_matrix = R.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    return rot_matrix @ np.diag(inertia_diag) @ rot_matrix.T


# --- Initialize ball velocities & spins ---
initial_conditions = {
    "ball1": (np.array([2.0, -1.0, 0.0]), np.array([-3.0, 1.0, 0.0])),
    "ball2": (np.array([-2.0, 1.0, 0.0]), np.array([4.0, -2.0, 0.0])),
    "ball3": (np.array([1.0, 2.0, 0.0]), np.array([2.0, 2.0, 0.0])),
    "ball4": (np.array([-1.0, -2.0, 0.0]), np.array([-4.0, -1.0, 0.0]))
}
for ball in ball_names:
    joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, f"{ball}_joint")
    start_idx = model.jnt_dofadr[joint_id]
    data.qvel[start_idx:start_idx+3] = initial_conditions[ball][0]
    data.qvel[start_idx+3:start_idx+6] = initial_conditions[ball][1]

# --- Logger Setup ---
logger = MultiSphereLogger(ball_names)  # ✅ Instantiate logger

# --- Recording Setup ---
output_video_path = "src/recordings/multi_sphere_bounce_custom_physics.mp4"
os.makedirs(os.path.dirname(output_video_path), exist_ok=True)
video_writer = imageio.get_writer(output_video_path, fps=30, codec='libx264')

# --- Visualization setup ---
cam = mj.MjvCamera()
opt = mj.MjvOption()
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
cam.azimuth, cam.elevation, cam.distance = 90, -30, 6
cam.lookat = np.array([0.0, 0.0, 0.5])

# --- Custom simulation step ---


def custom_simulation_step(model, data, dt=0.01, restitution=1.0):
    mj.mj_forward(model, data)
    for ball in ball_names:
        body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, ball)
        pos = data.xpos[body_id]
        mass = model.body_mass[body_id]
        inertia_diag = model.body_inertia[body_id]
        qpos_idx = model.jnt_qposadr[model.jnt_dofadr[mj.mj_name2id(
            model, mj.mjtObj.mjOBJ_JOINT, f"{ball}_joint")]]
        quat = data.qpos[qpos_idx+3:qpos_idx+7]
        inertia_world = compute_inertia_tensor_world(inertia_diag, quat)

        vel_idx = model.jnt_dofadr[mj.mj_name2id(
            model, mj.mjtObj.mjOBJ_JOINT, f"{ball}_joint")]
        vel = data.qvel[vel_idx:vel_idx+3]
        omega = data.qvel[vel_idx+3:vel_idx+6]

        # Apply gravity
        vel += model.opt.gravity * dt

        # Check ground collision
        if pos[2] < 0.1:
            contact_point = pos - np.array([0, 0, 0.1])
            normal = np.array([0.0, 0.0, 1.0])
            jn, jt = compute_collision_impulse_friction(
                mass, inertia_world, vel, omega, contact_point, normal, restitution, friction_coefficient)
            vel, omega = apply_impulse_friction(
                vel, omega, mass, inertia_world, contact_point, normal, jn, jt)
            data.qpos[qpos_idx+2] = 0.1

        # Update positions/orientations
        data.qpos[qpos_idx:qpos_idx+3] += vel * dt
        omega_quat = np.concatenate([[0], omega])
        res = np.zeros(4)
        mj.mju_mulQuat(res, omega_quat, quat)
        quat_new = quat + 0.5 * res * dt
        quat_new /= np.linalg.norm(quat_new)
        data.qpos[qpos_idx+3:qpos_idx+7] = quat_new
        data.qvel[vel_idx:vel_idx+3] = vel
        data.qvel[vel_idx+3:vel_idx+6] = omega

        # ✅ Log ball trajectory
        logger.record(ball, simulation_time, pos)


# --- Main simulation loop ---
simulation_time = 0.0
while not glfw.window_should_close(window):
    custom_simulation_step(model, data, dt=dt)
    simulation_time += dt

    # Render and record frames
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    rgb_buffer = np.zeros((viewport_height, viewport_width, 3), dtype=np.uint8)
    depth_buffer = np.zeros(
        (viewport_height, viewport_width), dtype=np.float32)
    mj.mjr_readPixels(rgb_buffer, depth_buffer, viewport, context)
    frame = np.flipud(rgb_buffer)
    video_writer.append_data(frame)

    glfw.swap_buffers(window)
    glfw.poll_events()

# --- Save recordings and plots ---
video_writer.close()
print(f"Simulation recording saved to {output_video_path}")
logger.save_all_plots("src/plots")
glfw.terminate()
