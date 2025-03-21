import os
import numpy as np
import mujoco as mj
from scipy.spatial.transform import Rotation as R
from src.visualization.multi_sphere_logger import MultiSphereLogger
from src.physics.collision import compute_collision_impulse_friction
from src.physics.physics_utils import apply_impulse_friction
from src.config import load_sim_config
from src.viewer.mujoco_viewer import (
    initialize_glfw_window, setup_mujoco_camera,
    register_callbacks, start_main_loop
)

# --- Load simulation-specific config for multi-sphere bounce ---
config = load_sim_config("multi_sphere_bounce")
friction_coefficient = config["FRICTION_COEFFICIENT"]
restitution_coefficient = config["RESTITUTION"]
timestep = config["TIMESTEP"]
camera_settings = config["CAMERA"]
output_video_path = config["RECORDING_PATH"]

# --- Load MuJoCo model for multiple spheres ---
xml_path = os.path.join(os.path.dirname(__file__), "../..",
                        "models", "multi_sphere.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# --- Initialize logger for multiple spheres ---
ball_names = ["ball1", "ball2", "ball3", "ball4"]
logger = MultiSphereLogger(ball_names)

# --- Helper function: compute inertia tensor in world frame ---


def compute_inertia_tensor_world(inertia_diag, q):
    rot_matrix = R.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    return rot_matrix @ np.diag(inertia_diag) @ rot_matrix.T

# --- Simulation step function for multi-sphere bounce ---


def custom_step_multi_sphere(model, data, dt=timestep, restitution=restitution_coefficient):
    mj.mj_forward(model, data)
    simulation_time = data.time

    for ball_name in ball_names:
        ball_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, ball_name)
        mass = model.body_mass[ball_id]
        inertia_diag = model.body_inertia[ball_id]
        qpos = data.qpos[ball_id * 7: ball_id * 7 + 7]
        qvel = data.qvel[ball_id * 6: ball_id * 6 + 6]

        vel = qvel[:3]
        omega = qvel[3:6]
        inertia_world = compute_inertia_tensor_world(inertia_diag, qpos[3:7])

        # External forces (gravity + applied)
        force = data.xfrc_applied[ball_id, :3] + mass * model.opt.gravity
        torque = data.xfrc_applied[ball_id, 3:]
        vel += (force / mass) * dt
        omega += np.linalg.inv(inertia_world) @ (torque * dt)

        # Collision handling
        for i in range(data.ncon):
            contact = data.contact[i]
            if contact.dist < 0 and ball_name in [model.id2name(contact.geom1), model.id2name(contact.geom2)]:
                contact_point = contact.pos - qpos[:3]
                normal = contact.frame[:3]
                jn, jt = compute_collision_impulse_friction(
                    mass, inertia_world, vel, omega, contact_point, normal, restitution, friction_coefficient
                )
                vel, omega = apply_impulse_friction(
                    vel, omega, mass, inertia_world, contact_point, normal, jn, jt
                )

        # Position + rotation integration
        pos_new = qpos[:3] + vel * dt
        omega_quat = np.concatenate([[0], omega])
        res = np.zeros(4)
        mj.mju_mulQuat(res, omega_quat, qpos[3:7])
        quat_new = qpos[3:7] + 0.5 * res * dt
        quat_new /= np.linalg.norm(quat_new)

        # Update state
        data.qpos[ball_id * 7: ball_id * 7 + 3] = pos_new
        data.qpos[ball_id * 7 + 3: ball_id * 7 + 7] = quat_new
        data.qvel[ball_id * 6: ball_id * 6 + 3] = vel
        data.qvel[ball_id * 6 + 3: ball_id * 6 + 6] = omega

        logger.record(ball_name, simulation_time, pos_new)

    return None  # No single position return, multiple objects logged

# --- Custom keyboard handler (reset with SPACE) ---


def custom_keyboard_handler(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_SPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
        print("Multi-sphere environment reset.")


# --- Initialize GLFW window and viewer camera ---
window = initialize_glfw_window("Multi-Sphere Rigid Body Simulation")
cam, opt, scene, context = setup_mujoco_camera(model, camera_settings)

# --- Register callbacks (with custom keyboard reset action) ---
register_callbacks(window, model, data, cam, scene, custom_keyboard_handler)

# --- Start main simulation loop ---
start_main_loop(
    window, model, data, cam, opt, scene, context,
    step_function=custom_step_multi_sphere,
    logger=logger,
    record_video=config["RECORD_VIDEO"],  # use global config toggle
    video_writer=None  # Optional: You can add video recording support similarly
)

# --- Save simulation plots at end ---
logger.save_all_plots("data/multi_sphere/plots")
print("Multi-sphere bounce simulation completed and plots saved.")
