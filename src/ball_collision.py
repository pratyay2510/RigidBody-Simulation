import os
import time
import numpy as np
import mujoco as mj
from mujoco.glfw import glfw
from data_logger import DataLogger
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- Global state variables ---
last_x, last_y = 0, 0
left_pressed, right_pressed = False, False
viewport_width, viewport_height = 1200, 900
restitution = 1.0
friction_coefficient = 0.3
ball_radius = 0.1
running = False  # Toggle simulation start/pause

# --- Load model ---
xml_path = os.path.join(os.path.dirname(__file__), "..",
                        "models", "ball_collision.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# Initial setup for two balls
data.qpos[0:3] = np.array([-1.0, 0.0, 1.0])
data.qpos[7:10] = np.array([1.0, 0.0, 1.0])
data.qvel[0:3] = np.array([1.0, 0.0, 0.5])
data.qvel[6:9] = np.array([-1.0, 0.0, 0.5])

# --- Inertia inverse matrix computation ---


def compute_inverse_inertia(mass, radius):
    inertia_val = (2.0 / 5.0) * mass * radius ** 2
    return np.eye(3) / inertia_val


ball1_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "ball1")
ball2_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "ball2")
mass1, mass2 = model.body_mass[ball1_id], model.body_mass[ball2_id]
I_inv_ball1 = compute_inverse_inertia(mass1, ball_radius)
I_inv_ball2 = compute_inverse_inertia(mass2, ball_radius)

# --- Collision impulse computation ---


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

# --- Custom simulation step with integration ---


def step_with_custom_collisions(model, data, dt=0.01):
    mj.mj_forward(model, data)

    # Integrate gravity
    for pos_idx, vel_idx in [(0, 0), (7, 6)]:
        data.qvel[vel_idx:vel_idx + 3] += model.opt.gravity * dt

    # Ball-ground collisions
    for pos_idx, vel_idx, ang_idx, mass, I_inv in [(0, 0, 3, mass1, I_inv_ball1), (7, 6, 9, mass2, I_inv_ball2)]:
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

    # Ball-ball collisions
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

    # Integrate positions
    for pos_idx, vel_idx in [(0, 0), (7, 6)]:
        data.qpos[pos_idx:pos_idx + 3] += data.qvel[vel_idx:vel_idx + 3] * dt


# --- Viewer and rendering setup ---
if not glfw.init():
    raise RuntimeError("GLFW init failed.")
window = glfw.create_window(viewport_width, viewport_height,
                            "Two Ball Custom Collision Simulation", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

cam = mj.MjvCamera()
opt = mj.MjvOption()
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
cam.distance, cam.lookat[:] = 5.0, [0.0, 0.0, 1.0]

# --- Callback Handlers ---


def keyboard(window, key, scancode, action, mods):
    global running
    if action == glfw.PRESS:
        if key == glfw.KEY_SPACE:
            running = not running
            print("Simulation running" if running else "Simulation paused")
        elif key == glfw.KEY_BACKSPACE:
            mj.mj_resetData(model, data)
            print("Simulation reset.")
        elif key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(window, True)


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

# --- Logger Setup ---
logger_ball1, logger_ball2 = DataLogger(), DataLogger()
simulation_time = 0.0

# --- Main simulation loop ---
while not glfw.window_should_close(window):
    if running:
        step_with_custom_collisions(model, data, dt=model.opt.timestep)
        simulation_time += model.opt.timestep

        # Log trajectories
        pos1, pos2 = data.qpos[0:3], data.qpos[7:10]
        logger_ball1.record(simulation_time, pos1[2], pos1[0], pos1[1])
        logger_ball2.record(simulation_time, pos2[2], pos2[0], pos2[1])

    # ✅ Always render, even if paused:
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)
    glfw.poll_events()


# --- Save results ---
logger_ball1.save_plot("src/plots/ball1_height_vs_time.png")
logger_ball1.save_trajectory_plot_3d("src/plots/ball1_trajectory_3d.png")
logger_ball2.save_plot("src/plots/ball2_height_vs_time.png")
logger_ball2.save_trajectory_plot_3d("src/plots/ball2_trajectory_3d.png")

glfw.terminate()
