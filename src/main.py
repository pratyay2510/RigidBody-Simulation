import os
import time
import numpy as np
import mujoco as mj
from mujoco.glfw import glfw
from scipy.spatial.transform import Rotation as R

# --- Initialize GLFW --- #
if not glfw.init():
    raise RuntimeError("Could not initialize GLFW")

window = glfw.create_window(
    1200, 900, "Custom Rigid Body Simulation with Contact Handling", None, None)
if not window:
    glfw.terminate()
    raise RuntimeError("Could not create GLFW window")
glfw.make_context_current(window)
glfw.swap_interval(1)

# --- Load MuJoCo model --- #
xml_path = os.path.join(os.path.dirname(__file__),
                        "..", "models", "sphere.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# --- Custom Dynamics Implementation (Based on Paper) --- #


def compute_inertia_tensor_world(inertia_diag, q):
    """Compute world-frame inertia tensor given quaternion orientation."""
    rotation_matrix = R.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    I_object = np.diag(inertia_diag)
    I_world = rotation_matrix @ I_object @ rotation_matrix.T
    return I_world


def custom_step_with_contact(model, data, dt=0.01):
    """Custom integration and collision resolution with MuJoCo contact detection."""
    # 1. Forward to update contact information
    mj.mj_forward(model, data)

    ball_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "ball")
    mass = model.body_mass[ball_body_id]
    inertia_diag = model.body_inertia[ball_body_id]

    # Current forces
    force = data.xfrc_applied[ball_body_id, :3] + mass * model.opt.gravity
    torque = data.xfrc_applied[ball_body_id, 3:]

    vel = data.qvel[:3]
    omega = data.qvel[3:6]
    pos = data.qpos[:3]
    quat = data.qpos[3:7]

    # 2. Advance velocities
    vel_new = vel + (force / mass) * dt
    inertia_world = compute_inertia_tensor_world(inertia_diag, quat)
    omega_new = omega + np.linalg.inv(inertia_world) @ (torque * dt)

    # 3. Use MuJoCo's contact information for shock propagation-like correction
    if data.ncon > 0:
        for i in range(data.ncon):
            contact = data.contact[i]
            if not np.isnan(contact.dist):
                normal = contact.frame[:3]
                penetration_depth = -contact.dist if contact.dist < 0 else 0
                impulse = penetration_depth * 10.0  # Scaled corrective impulse
                vel_new += normal * impulse / mass

    # 4. Advance positions
    pos_new = pos + vel_new * dt
    omega_quat = np.concatenate([[0], omega_new])
    res = np.zeros(4)
    mj.mju_mulQuat(res, omega_quat, quat)
    delta_q = 0.5 * res * dt
    quat_new = quat + delta_q
    quat_new /= np.linalg.norm(quat_new)

    # Update simulation state
    data.qpos[:3] = pos_new
    data.qpos[3:7] = quat_new
    data.qvel[:3] = vel_new
    data.qvel[3:6] = omega_new


# --- Visualization Setup --- #
cam = mj.MjvCamera()
opt = mj.MjvOption()
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
cam.azimuth = 90
cam.elevation = -30
cam.distance = 6
cam.lookat = np.array([0.0, 0.0, 0.5])

# --- Mouse & Keyboard Handlers --- #


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
    global last_x, last_y, viewport_width, viewport_height
    dx = xpos - last_x
    dy = ypos - last_y
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
    start_time = time.time()

    custom_step_with_contact(model, data, dt=model.opt.timestep)

    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    if data.ncon > 0:
        for i in range(data.ncon):
            contact = data.contact[i]
            print(
                f"Contact {i}: dist={contact.dist}, pos={contact.pos}, frame={contact.frame[:3]}")

    glfw.swap_buffers(window)
    glfw.poll_events()

    elapsed = time.time() - start_time
    if model.opt.timestep - elapsed > 0:
        time.sleep(model.opt.timestep - elapsed)

glfw.terminate()
