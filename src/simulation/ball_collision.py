import os
import numpy as np
import mujoco as mj
import imageio
import glfw
from src.visualization.data_logger import DataLogger
from src.config import load_sim_config
from src.viewer.mujoco_viewer import (
    initialize_glfw_window, setup_mujoco_camera,
    register_callbacks, start_main_loop
)

# ✅ Load centralized config for ball collision
config = load_sim_config("ball_collision")
friction_coefficient = config["FRICTION_COEFFICIENT"]
restitution = config["RESTITUTION"]
timestep = config["TIMESTEP"]
camera_settings = config["CAMERA"]
output_video_path = config["RECORDING_PATH"]
record_video = config["RECORD_VIDEO"]

# Hardcoded model geometry parameter (not from config)
ball_radius = 0.1

# ✅ Load model for ball collision simulation
xml_path = os.path.join("models", "ball_collision.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# ✅ Set custom initial conditions for the two colliding balls
data.qpos[0:3] = np.array([-1.0, 0.0, 1.0])
data.qpos[7:10] = np.array([1.0, 0.0, 1.0])
data.qvel[0:3] = np.array([1.0, 0.0, 0.5])
data.qvel[6:9] = np.array([-1.0, 0.0, 0.5])

# ✅ Inverse inertia tensor computation (unique physics logic)


def compute_inverse_inertia(mass, radius):
    inertia_val = (2.0 / 5.0) * mass * radius ** 2
    return np.eye(3) / inertia_val


ball1_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "ball1")
ball2_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "ball2")
mass1, mass2 = model.body_mass[ball1_id], model.body_mass[ball2_id]
I_inv_ball1 = compute_inverse_inertia(mass1, ball_radius)
I_inv_ball2 = compute_inverse_inertia(mass2, ball_radius)

# ✅ Custom collision impulse calculation


def compute_collision_impulse(mass, I_inv, v_lin, v_ang, r, n, restitution, mu):
    v_contact = v_lin + np.cross(v_ang, r)
    v_n = np.dot(v_contact, n)
    v_t = v_contact - v_n * n
    t_norm = np.linalg.norm(v_t)

    denom_n = (1.0 / mass) + np.dot(n, np.cross(I_inv @ np.cross(r, n), r))
    jn = -(1 + restitution) * v_n / denom_n

    t_dir = v_t / t_norm if t_norm > 1e-8 else np.zeros(3)
    denom_t = (1.0 / mass) + np.dot(t_dir,
                                    np.cross(I_inv @ np.cross(r, t_dir), r))
    jt_unclamped = -t_norm / denom_t
    jt = np.clip(jt_unclamped, -mu * abs(jn), mu * abs(jn))

    return jn * n + jt * t_dir

# ✅ Custom simulation step logic for collisions


def step_with_custom_collisions(model, data, dt=0.01):
    mj.mj_forward(model, data)

    # Apply gravity
    for pos_idx, vel_idx in [(0, 0), (7, 6)]:
        data.qvel[vel_idx:vel_idx + 3] += model.opt.gravity * dt

    # Ball-ground collisions
    for pos_idx, vel_idx, ang_idx, mass, I_inv in [
        (0, 0, 3, mass1, I_inv_ball1),
        (7, 6, 9, mass2, I_inv_ball2)
    ]:
        pos = data.qpos[pos_idx:pos_idx + 3]
        linvel = data.qvel[vel_idx:vel_idx + 3]
        angvel = data.qvel[ang_idx:ang_idx + 3]
        normal = np.array([0.0, 0.0, 1.0])

        if pos[2] < ball_radius:
            contact_point = pos - ball_radius * normal
            r = contact_point - pos
            impulse = compute_collision_impulse(
                mass, I_inv, linvel, angvel, r, normal, restitution, friction_coefficient)
            data.qvel[vel_idx:vel_idx + 3] += impulse / mass
            data.qvel[ang_idx:ang_idx + 3] += I_inv @ np.cross(r, impulse)
            data.qpos[pos_idx + 2] = ball_radius

    # Ball-ball collision detection & resolution
    diff = data.qpos[7:10] - data.qpos[0:3]
    dist = np.linalg.norm(diff)
    tol = 0.01
    if dist < 2 * ball_radius + tol:
        normal = diff / (dist + 1e-8)
        contact_point = (data.qpos[0:3] + data.qpos[7:10]) / 2.0
        r1 = contact_point - data.qpos[0:3]
        r2 = contact_point - data.qpos[7:10]

        impulse = compute_collision_impulse(
            mass1, I_inv_ball1, data.qvel[0:3], data.qvel[3:6], r1, normal, restitution, friction_coefficient)
        data.qvel[0:3] += impulse / mass1
        data.qvel[3:6] += I_inv_ball1 @ np.cross(r1, impulse)
        data.qvel[6:9] -= impulse / mass2
        data.qvel[9:12] -= I_inv_ball2 @ np.cross(r2, impulse)

        correction = (2 * ball_radius + tol - dist) / 2.0
        data.qpos[0:3] -= correction * normal
        data.qpos[7:10] += correction * normal

    # Position integration
    for pos_idx, vel_idx in [(0, 0), (7, 6)]:
        data.qpos[pos_idx:pos_idx + 3] += data.qvel[vel_idx:vel_idx + 3] * dt

    # Return both ball positions for logging
    return data.qpos[0:3], data.qpos[7:10]


# ✅ Data loggers for two balls
logger_ball1 = DataLogger()
logger_ball2 = DataLogger()
running = False  # Start/pause state toggle

# ✅ Custom keyboard handler to toggle running state


def custom_keyboard_handler(window, key, scancode, act, mods):
    global running
    if act == glfw.PRESS:
        if key == glfw.KEY_SPACE:
            running = not running
            print("Simulation running" if running else "Simulation paused")

# ✅ Define step function wrapper for main loop


def ball_collision_step(model, data, dt):
    if running:
        ball1_pos, ball2_pos = step_with_custom_collisions(model, data, dt)
        simulation_time = data.time
        logger_ball1.record(
            simulation_time, ball1_pos[2], ball1_pos[0], ball1_pos[1])
        logger_ball2.record(
            simulation_time, ball2_pos[2], ball2_pos[0], ball2_pos[1])
    return None  # No single object position; two separate logs used


# ✅ Initialize viewer window
window = initialize_glfw_window("Ball Collision Simulation")

# ✅ Setup camera via shared utility
cam, opt, scene, context = setup_mujoco_camera(model, camera_settings)

# ✅ Setup recording if enabled
video_writer = None
if record_video and output_video_path:
    os.makedirs(os.path.dirname(output_video_path), exist_ok=True)
    video_writer = imageio.get_writer(
        output_video_path, fps=30, codec="libx264")

# ✅ Register callbacks
register_callbacks(window, model, data, cam, scene,
                   custom_keyboard_handler=custom_keyboard_handler)

# ✅ Start simulation loop using centralized utility
start_main_loop(
    window, model, data, cam, opt, scene, context,
    step_function=ball_collision_step,
    logger=None,  # separate loggers per ball
    record_video=record_video,
    video_writer=video_writer
)

# ✅ Save results & plots
logger_ball1.save_plot("data/plots/ball_collision/ball1_height_vs_time.png")
logger_ball1.save_trajectory_plot_3d(
    "data/plots/ball_collision/ball1_trajectory_3d.png")
logger_ball2.save_plot("data/plots/ball_collision/ball2_height_vs_time.png")
logger_ball2.save_trajectory_plot_3d(
    "data/plots/ball_collision/ball2_trajectory_3d.png")

if video_writer is not None:
    video_writer.close()
    print(f"Simulation recording saved to: {output_video_path}")
