import os
import time
import numpy as np
import mujoco as mj
from mujoco.glfw import glfw
from scipy.spatial.transform import Rotation as R
from multi_sphere_logger import MultiSphereLogger
from simulation.collision import compute_collision_impulse_friction
from simulation.physics import apply_impulse_friction
import imageio

# --- Global parameters ---
last_x, last_y = 0, 0
left_pressed = False
right_pressed = False
viewport_width, viewport_height = 1200, 900
friction_coefficient = 0.0  # Set to zero for perfectly elastic collisions
restitution_coefficient = 1.0

# --- Initialize GLFW ---
if not glfw.init():
    raise RuntimeError("Could not initialize GLFW")

window = glfw.create_window(
    viewport_width, viewport_height, "Multi-Sphere Rigid Body Simulation", None, None)
if not window:
    glfw.terminate()
    raise RuntimeError("Could not create GLFW window")
glfw.make_context_current(window)
glfw.swap_interval(1)
last_x, last_y = glfw.get_cursor_pos(window)

# --- Load multi-sphere model ---
xml_path = os.path.join(os.path.dirname(__file__), "..",
                        "models", "multi_sphere.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# --- Initialize logger for all spheres ---
ball_names = ["ball1", "ball2", "ball3", "ball4"]
logger = MultiSphereLogger(ball_names)
simulation_time = 0.0

# --- Helper function ---


def compute_inertia_tensor_world(inertia_diag, q):
    rot_matrix = R.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    return rot_matrix @ np.diag(inertia_diag) @ rot_matrix.T


# --- Custom simulation step for multiple spheres with frictionless impulse collision ---
def custom_step_multi_sphere(model, data, dt=0.01, restitution=restitution_coefficient):
    mj.mj_forward(model, data)

    for ball_name in ball_names:
        ball_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, ball_name)

        mass = model.body_mass[ball_id]
        inertia_diag = model.body_inertia[ball_id]
        qpos = data.qpos[ball_id * 7: ball_id * 7 + 7]
        qvel = data.qvel[ball_id * 6: ball_id * 6 + 6]

        vel = qvel[:3]
        omega = qvel[3:6]
        inertia_world = compute_inertia_tensor_world(inertia_diag, qpos[3:7])

        force = data.xfrc_applied[ball_id, :3] + mass * model.opt.gravity
        torque = data.xfrc_applied[ball_id, 3:]

        vel += (force / mass) * dt
        omega += np.linalg.inv(inertia_world) @ (torque * dt)

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

        pos_new = qpos[:3] + vel * dt
        omega_quat = np.concatenate([[0], omega])
        res = np.zeros(4)
        mj.mju_mulQuat(res, omega_quat, qpos[3:7])
        quat_new = qpos[3:7] + 0.5 * res * dt
        quat_new /= np.linalg.norm(quat_new)

        data.qpos[ball_id * 7: ball_id * 7 + 3] = pos_new
        data.qpos[ball_id * 7 + 3: ball_id * 7 + 7] = quat_new
        data.qvel[ball_id * 6: ball_id * 6 + 3] = vel
        data.qvel[ball_id * 6 + 3: ball_id * 6 + 6] = omega

        logger.record(ball_name, simulation_time, pos_new)


# --- Visualization setup ---
cam = mj.MjvCamera()
opt = mj.MjvOption()
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
cam.azimuth, cam.elevation, cam.distance = 90, -30, 6
cam.lookat = np.array([0.0, 0.0, 1.0])

# --- Recording setup ---
output_video_path = "src/recordings/multi_sphere/multi_sphere_bounce.mp4"
os.makedirs(os.path.dirname(output_video_path), exist_ok=True)
video_writer = imageio.get_writer(output_video_path, fps=30, codec='libx264')

# --- Mouse and keyboard callbacks ---


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_SPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
        print("Environment reset triggered.")


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

# --- Main simulation loop ---
while not glfw.window_should_close(window):
    simulation_time += model.opt.timestep
    custom_step_multi_sphere(model, data, dt=model.opt.timestep)

    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # Capture frame for video
    rgb_buffer = np.zeros((viewport_height, viewport_width, 3), dtype=np.uint8)
    depth_buffer = np.zeros(
        (viewport_height, viewport_width), dtype=np.float32)
    mj.mjr_readPixels(rgb_buffer, depth_buffer, viewport, context)
    frame = np.flipud(rgb_buffer)
    video_writer.append_data(frame)

    glfw.swap_buffers(window)
    glfw.poll_events()

# --- Save plots and recording ---
logger.save_all_plots("data/multi_sphere/plots")
video_writer.close()
print(f"Simulation recording saved to {output_video_path}")

glfw.terminate()
