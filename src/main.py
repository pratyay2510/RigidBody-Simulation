import os
import time
import numpy as np
import mujoco as mj
from mujoco.glfw import glfw
from scipy.spatial.transform import Rotation as R
from data_logger import DataLogger
from simulation.collision import compute_collision_impulse, compute_collision_impulse_friction
from simulation.physics import apply_impulse, apply_impulse_friction

# --- Global state variables
last_x, last_y = 0, 0
left_pressed = False
right_pressed = False
viewport_width, viewport_height = 1200, 900
friction_coefficient = 0.5

# --- Initialize GLFW
if not glfw.init():
    raise RuntimeError("Could not initialize GLFW")

window = glfw.create_window(
    viewport_width, viewport_height, "Rigid Body Sim with Initial Angular Velocity", None, None)
if not window:
    glfw.terminate()
    raise RuntimeError("Could not create GLFW window")
glfw.make_context_current(window)
glfw.swap_interval(1)
last_x, last_y = glfw.get_cursor_pos(window)

# --- Load model and initialize
xml_path = os.path.join(os.path.dirname(__file__),
                        "..", "models", "sphere.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# ✅ Set initial angular velocity for the ball (spin around z-axis)
ball_joint_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, "ball_joint")
# Spin around z-axis at 20 rad/s
angular_velocity = np.array([3.0, 0.0, 0.0])
linear_velocity = np.array([2.0, 0.0, 0.0])
data.qvel[:3] = linear_velocity
data.qvel[3:6] = angular_velocity

# --- Initialize logger
logger = DataLogger()
simulation_time = 0.0

# --- Helper function


def compute_inertia_tensor_world(inertia_diag, q):
    rot_matrix = R.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    return rot_matrix @ np.diag(inertia_diag) @ rot_matrix.T

# --- Custom simulation step with impulse-based collision handling


def custom_step_with_impulse_collision(model, data, dt=0.01, restitution=1.0):
    mj.mj_forward(model, data)
    ball_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "ball")

    mass = model.body_mass[ball_body_id]
    inertia_diag = model.body_inertia[ball_body_id]
    inertia_world = compute_inertia_tensor_world(inertia_diag, data.qpos[3:7])

    vel = data.qvel[:3]
    omega = data.qvel[3:6]
    force = data.xfrc_applied[ball_body_id, :3] + mass * model.opt.gravity
    torque = data.xfrc_applied[ball_body_id, 3:]

    # Integrate velocities
    vel += (force / mass) * dt
    omega += np.linalg.inv(inertia_world) @ (torque * dt)

    # Collision handling via impulse
    for i in range(data.ncon):
        contact = data.contact[i]
        if not np.isnan(contact.dist) and contact.dist < 0:
            contact_point = contact.pos - data.qpos[:3]
            normal = contact.frame[:3]
            jn = compute_collision_impulse(
                mass, inertia_world, vel, omega, contact_point, normal, restitution)
            vel, omega = apply_impulse(
                vel, omega, mass, inertia_world, contact_point, normal, jn)

    # Integrate positions
    pos_new = data.qpos[:3] + vel * dt
    omega_quat = np.concatenate([[0], omega])
    res = np.zeros(4)
    mj.mju_mulQuat(res, omega_quat, data.qpos[3:7])
    quat_new = data.qpos[3:7] + 0.5 * res * dt
    quat_new /= np.linalg.norm(quat_new)

    # Update state
    data.qpos[:3] = pos_new
    data.qpos[3:7] = quat_new
    data.qvel[:3] = vel
    data.qvel[3:6] = omega

# Collision and physics updates


def custom_step_with_impulse_collision_friction(model, data, dt=0.01, restitution=1.0):
    """
    Custom simulation step with impulse-based collision and friction handling.
    Returns updated position for 3D trajectory logging.
    """
    mj.mj_forward(model, data)
    ball_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "ball")

    mass = model.body_mass[ball_body_id]
    inertia_diag = model.body_inertia[ball_body_id]
    inertia_world = compute_inertia_tensor_world(inertia_diag, data.qpos[3:7])

    vel = data.qvel[:3]
    omega = data.qvel[3:6]
    force = data.xfrc_applied[ball_body_id, :3] + mass * model.opt.gravity
    torque = data.xfrc_applied[ball_body_id, 3:]

    # Integrate velocities
    vel += (force / mass) * dt
    omega += np.linalg.inv(inertia_world) @ (torque * dt)

    # Collision and friction handling
    for i in range(data.ncon):
        contact = data.contact[i]
        if not np.isnan(contact.dist) and contact.dist < 0:
            contact_point = contact.pos - data.qpos[:3]
            normal = contact.frame[:3]

            # Compute both normal and friction impulses
            jn, jt = compute_collision_impulse_friction(
                mass, inertia_world, vel, omega, contact_point, normal, restitution, friction_coefficient
            )
            vel, omega = apply_impulse_friction(
                vel, omega, mass, inertia_world, contact_point, normal, jn, jt
            )

    # Integrate positions
    pos_new = data.qpos[:3] + vel * dt
    omega_quat = np.concatenate([[0], omega])
    res = np.zeros(4)
    mj.mju_mulQuat(res, omega_quat, data.qpos[3:7])
    quat_new = data.qpos[3:7] + 0.5 * res * dt
    quat_new /= np.linalg.norm(quat_new)

    # Update simulation state
    data.qpos[:3] = pos_new
    data.qpos[3:7] = quat_new
    data.qvel[:3] = vel
    data.qvel[3:6] = omega

    # Return the new position for logging
    return pos_new


# --- Visualization setup (unchanged) --- #
cam = mj.MjvCamera()
opt = mj.MjvOption()
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
cam.azimuth, cam.elevation, cam.distance = 90, -30, 6
cam.lookat = np.array([0.0, 0.0, 0.5])

# --- Mouse and keyboard callbacks (unchanged) --- #


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
        print("Simulation reset.")


def mouse_button(window, button, act, mods):
    global left_pressed, right_pressed, last_x, last_y
    if button == glfw.MOUSE_BUTTON_LEFT:
        left_pressed = (act == glfw.PRESS)
    if button == glfw.MOUSE_BUTTON_RIGHT:
        right_pressed = (act == glfw.PRESS)
    last_x, last_y = glfw.get_cursor_pos(window)


def mouse_move(window, xpos, ypos):
    global last_x, last_y
    dx, dy = xpos - last_x, ypos - last_y
    last_x, last_y = xpos, ypos
    if left_pressed:
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ROTATE_V,
                          dx / viewport_height, dy / viewport_height, scene, cam)
    if right_pressed:
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_MOVE_H,
                          dx / viewport_height, dy / viewport_height, scene, cam)


def scroll(window, xoffset, yoffset):
    mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ZOOM,
                      0, -0.05 * yoffset, scene, cam)


glfw.set_key_callback(window, keyboard)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_scroll_callback(window, scroll)

# --- Main simulation loop --- #
while not glfw.window_should_close(window):
    pos_new = custom_step_with_impulse_collision_friction(
        model, data, dt=model.opt.timestep)
    simulation_time += model.opt.timestep
    x, y, z = pos_new
    logger.record(simulation_time, z, x, y)

    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)
    glfw.poll_events()

# ✅ Save both plots
logger.save_plot("src/plots/height_vs_time.png")
logger.save_trajectory_plot_3d("src/plots/3d_trajectory.png")

glfw.terminate()
