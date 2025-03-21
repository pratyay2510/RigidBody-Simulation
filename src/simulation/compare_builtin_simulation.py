import os
import time
import numpy as np
import mujoco as mj
from mujoco.glfw import glfw
from data_logger import DataLogger

# --- Global state variables
viewport_width, viewport_height = 1200, 900
last_x, last_y = 0, 0
left_pressed, right_pressed = False, False

# --- Initialize GLFW --- #
if not glfw.init():
    raise RuntimeError("Could not initialize GLFW")

window = glfw.create_window(
    viewport_width, viewport_height, "MuJoCo Built-in Simulation (No Custom Impulse)", None, None)
if not window:
    glfw.terminate()
    raise RuntimeError("Could not create GLFW window")
glfw.make_context_current(window)
glfw.swap_interval(1)
last_x, last_y = glfw.get_cursor_pos(window)

# --- Load MuJoCo model --- #
xml_path = os.path.join(os.path.dirname(__file__),
                        "..", "models", "sphere.xml")
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# ✅ Set initial conditions (optional small spin or initial placement)
data.qvel[3:6] = np.zeros(3)  # No initial angular velocity
data.qpos[2] = 1.0  # Start from height 1

# --- Initialize logger for height logging --- #
logger = DataLogger()
simulation_time = 0.0

# --- Camera and visualization setup --- #
cam = mj.MjvCamera()
opt = mj.MjvOption()
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
cam.azimuth, cam.elevation, cam.distance = 90, -30, 6
cam.lookat = np.array([0.0, 0.0, 0.5])

# --- Mouse and keyboard callbacks (for camera control) --- #


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

# --- Main simulation loop (using Mujoco built-in integration) --- #
while not glfw.window_should_close(window):
    # ✅ Built-in step call without custom impulse or friction modeling
    mj.mj_step(model, data)

    # Log simulation time and height
    simulation_time += model.opt.timestep
    logger.record(simulation_time, data.qpos[2])

    # Render scene
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)
    glfw.poll_events()

# --- Save results plot for comparison --- #
logger.save_plot("data/plots/height_vs_time_builtin.png")

glfw.terminate()
