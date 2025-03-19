"""
main.py

Entry point for the rigid body simulation Phase 1 using MuJoCo for rendering.
This module initializes a MuJoCo model (assumed to be defined in models/sphere.xml)
that contains a free body representing a sphere bouncing on a static floor.
We use MuJoCo's built-in physics engine (via mj_step) for simulation (including gravity and collisions),
and MuJoCo is used only for visualization (we do not use our custom simulation routines in this version).

Assumptions:
    - The XML file (models/sphere.xml) defines a world with a static floor and a single
      free body (the sphere) at body index 1.
    - MuJoCo’s built-in physics engine is used (mj_step).
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

# --- Install GLFW Callbacks using the valid window object --- #


def keyboard(window, key, scancode, act, mods):
    global sphere  # so we can modify the sphere position
    # Reset simulation when Backspace is pressed
    if act == glfw.PRESS:
        if key == glfw.KEY_BACKSPACE:
            mj.mj_resetData(model, data)
            mj.mj_forward(model, data)
        elif key == glfw.KEY_P:
            # Reposition sphere to the current camera lookat
            sphere.position = cam.lookat.copy()
            print("Sphere repositioned to:", sphere.position)


def mouse_button(window, button, act, mods):
    global left_pressed, right_pressed, last_x, last_y
    if button == glfw.MOUSE_BUTTON_LEFT:
        left_pressed = (act == glfw.PRESS)
    if button == glfw.MOUSE_BUTTON_RIGHT:
        right_pressed = (act == glfw.PRESS)
    # Record the cursor position when a button is pressed
    last_x, last_y = glfw.get_cursor_pos(window)


def mouse_move(window, xpos, ypos):
    global last_x, last_y, cam
    dx = xpos - last_x
    dy = ypos - last_y
    last_x, last_y = xpos, ypos
    # If left mouse is pressed, rotate the camera.
    if left_pressed:
        # Use MuJoCo's camera movement function.
        # When left button is pressed, rotate the camera vertically.
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ROTATE_V, dx /
                          float(viewport_height), dy/float(viewport_height), scene, cam)
    # If right mouse is pressed, pan the camera.
    if right_pressed:
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_MOVE_H, dx /
                          float(viewport_height), dy/float(viewport_height), scene, cam)


def scroll(window, xoffset, yoffset):
    # Zoom the camera on scroll.
    mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ZOOM,
                      0, -0.05 * yoffset, scene, cam)


glfw.set_key_callback(window, keyboard)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_scroll_callback(window, scroll)

# --- MuJoCo Model Setup --- #
xml_path = os.path.join(os.path.dirname(__file__),
                        "..", "models", "sphere.xml")
xml_path = os.path.abspath(xml_path)

model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# --- Adjustable Simulation Parameters ---
# Gravity is now defined in the XML file.
# The coefficient of restitution, friction, etc., can be adjusted here if supported.
# (Currently these are defined in the XML or via model.opt.)
# For example, to adjust restitution (if supported):
restitution = 1.0  # 1.0 for perfectly elastic collisions; modify as needed.
if hasattr(model.opt, "restitution"):
    model.opt.restitution = restitution

# --- Create visualization objects --- #
cam = mj.MjvCamera()
opt = mj.MjvOption()
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# --- Set Camera Parameters --- #
cam.azimuth = 90
cam.elevation = -20
cam.distance = 5
cam.lookat = np.array([0.0, 0.5, 0.0])

# Simulation parameters
total_simulation_time = 5.0  # seconds

# --- Main Simulation Loop ---
while not glfw.window_should_close(window) and data.time < total_simulation_time:
    time_start = time.time()

    # Advance simulation using MuJoCo's built-in physics engine.
    mj.mj_step(model, data)

    # --- Render the Scene using MuJoCo ---
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)
    glfw.poll_events()

    # Sleep to enforce the time step if necessary.
    elapsed = time.time() - time_start
    if model.opt.timestep - elapsed > 0:
        time.sleep(model.opt.timestep - elapsed)

glfw.terminate()
