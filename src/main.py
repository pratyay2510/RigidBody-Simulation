"""
main.py

Entry point for the rigid body simulation Phase 1 using MuJoCo for rendering.
This module initializes a MuJoCo model (defined in models/sphere.xml)
that contains a free body representing a sphere bouncing on a static floor.
We use MuJoCo's built-in physics engine (mj_step) for simulation (including gravity and collisions).
"""

import os
import time
import numpy as np

import mujoco as mj
from mujoco.glfw import glfw

# Global variables for mouse interaction
last_x, last_y = 0, 0
left_pressed = False
right_pressed = False

# --- Initialize GLFW First --- #
if not glfw.init():
    raise RuntimeError("Could not initialize GLFW")

# --- Create Window --- #
window = glfw.create_window(
    1200, 900, "MuJoCo Ball Bounce Simulation", None, None)
if not window:
    glfw.terminate()
    raise RuntimeError("Could not create GLFW window")
glfw.make_context_current(window)
glfw.swap_interval(1)  # Enable v-sync

# --- MuJoCo Model Setup --- #
xml_path = os.path.join(os.path.dirname(__file__),
                        "..", "models", "sphere.xml")
xml_path = os.path.abspath(xml_path)
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# --- Create visualization objects --- #
cam = mj.MjvCamera()
opt = mj.MjvOption()
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# --- Set Initial Camera Parameters --- #
cam.azimuth = 90
cam.elevation = -20
cam.distance = 5
cam.lookat = np.array([0.0, 0.5, 0.0])

# --- Callback Handlers --- #


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS:
        if key == glfw.KEY_BACKSPACE:
            mj.mj_resetData(model, data)
            mj.mj_forward(model, data)
            print("Simulation reset.")
        elif key == glfw.KEY_P:
            # Place the sphere at camera lookat position
            ball_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "ball")
            data.qpos[ball_body_id * 7: ball_body_id * 7 + 3] = cam.lookat[:3]
            data.qvel[ball_body_id * 6: ball_body_id *
                      6 + 3] = 0  # Reset velocity
            mj.mj_forward(model, data)
            print(f"Sphere repositioned to camera lookat: {cam.lookat}")


def mouse_button(window, button, act, mods):
    global left_pressed, right_pressed, last_x, last_y
    if button == glfw.MOUSE_BUTTON_LEFT:
        left_pressed = (act == glfw.PRESS)
    if button == glfw.MOUSE_BUTTON_RIGHT:
        right_pressed = (act == glfw.PRESS)
    last_x, last_y = glfw.get_cursor_pos(window)


def mouse_move(window, xpos, ypos):
    global last_x, last_y
    dx = xpos - last_x
    dy = ypos - last_y
    last_x, last_y = xpos, ypos
    if left_pressed:
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ROTATE_V, dx /
                          float(viewport_height), dy / float(viewport_height), scene, cam)
    if right_pressed:
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_MOVE_H, dx /
                          float(viewport_height), dy / float(viewport_height), scene, cam)


def scroll(window, xoffset, yoffset):
    mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ZOOM,
                      0, -0.05 * yoffset, scene, cam)


# --- Install Callbacks --- #
glfw.set_key_callback(window, keyboard)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_scroll_callback(window, scroll)

# --- Main Simulation Loop (runs until the window is closed) --- #
while not glfw.window_should_close(window):
    time_start = time.time()

    # Advance simulation using MuJoCo's built-in physics engine
    mj.mj_step(model, data)

    # Render the scene
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)
    glfw.poll_events()

    # Maintain real-time simulation pace
    elapsed = time.time() - time_start
    if model.opt.timestep - elapsed > 0:
        time.sleep(model.opt.timestep - elapsed)

glfw.terminate()
