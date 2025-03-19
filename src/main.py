"""
main.py

Entry point for the rigid body simulation Phase 1 using MuJoCo for rendering.
This module initializes a MuJoCo model (defined in models/sphere.xml)
that contains a free body representing a sphere bouncing on a static floor.
We use MuJoCo's built-in physics engine (mj_step) for simulation (including gravity and collisions).
It also supports real-time adjustments of gravity, restitution, and (optionally) friction.
"""

import os
import time
import numpy as np

import mujoco as mj
from mujoco.glfw import glfw

# Global variables
last_x, last_y = 0, 0
left_pressed = False
right_pressed = False
viewport_width, viewport_height = 1200, 900

# --- Initialize GLFW --- #
if not glfw.init():
    raise RuntimeError("Could not initialize GLFW")

window = glfw.create_window(
    viewport_width, viewport_height, "MuJoCo Ball Bounce Simulation", None, None)
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

# Adjustable parameters
restitution = 1.0
gravity_modes = [
    np.array([0, -9.81, 0]),
    np.array([0, 0, 0]),
    np.array([0, -3.0, 0])
]
gravity_index = 0
model.opt.gravity[:] = gravity_modes[gravity_index]

# --- Create visualization objects --- #
cam = mj.MjvCamera()
opt = mj.MjvOption()
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# --- Set Initial Camera Parameters --- #
cam.azimuth = 90
cam.elevation = -30
cam.distance = 6
cam.lookat = np.array([0.0, 0.5, 0.0])

# --- Callback handlers --- #


def keyboard(window, key, scancode, act, mods):
    global restitution, gravity_index
    if act == glfw.PRESS:
        if key == glfw.KEY_BACKSPACE:
            mj.mj_resetData(model, data)
            mj.mj_forward(model, data)
            print("Simulation reset.")
        elif key == glfw.KEY_P:
            # Get joint ID and addresses
            joint_id = mj.mj_name2id(
                model, mj.mjtObj.mjOBJ_JOINT, "ball_free_joint")
            pos_addr = model.jnt_qposadr[joint_id]
            vel_addr = model.jnt_dofadr[joint_id]

            # Position: use camera lookat, but set y = floor_height + radius + offset
            sphere_height = 0.5 + 0.05  # radius + small margin
            target_pos = cam.lookat.copy()
            target_pos[1] = sphere_height

            # Update qpos and qvel
            data.qpos[pos_addr:pos_addr+3] = target_pos
            data.qpos[pos_addr+3:pos_addr+7] = [1,
                                                0, 0, 0]  # Identity quaternion
            data.qvel[vel_addr:vel_addr+6] = 0.0
            mj.mj_forward(model, data)
            print(f"Sphere repositioned to: {target_pos}")

        elif key == glfw.KEY_R:
            restitution = min(restitution + 0.1, 1.0)
            ball_geom_id = mj.mj_name2id(
                model, mj.mjtObj.mjOBJ_GEOM, "ball_geom")
            model.geom_solimp[ball_geom_id][2] = restitution
            print(f"Restitution (approx.) increased to {restitution:.2f}")

        elif key == glfw.KEY_G:
            gravity_index = (gravity_index + 1) % len(gravity_modes)
            model.opt.gravity[:] = gravity_modes[gravity_index]
            print(f"Gravity switched to: {model.opt.gravity}")


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


# --- Install callbacks --- #
glfw.set_key_callback(window, keyboard)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_scroll_callback(window, scroll)

# --- Main simulation loop --- #
while not glfw.window_should_close(window):
    time_start = time.time()

    mj.mj_step(model, data)

    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)
    glfw.poll_events()

    elapsed = time.time() - time_start
    if model.opt.timestep - elapsed > 0:
        time.sleep(model.opt.timestep - elapsed)

glfw.terminate()
