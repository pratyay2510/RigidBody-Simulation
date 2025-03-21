# src/viewer/mujoco_viewer.py

import mujoco as mj
from mujoco.glfw import glfw
import numpy as np

# Common global state variables used across simulations
viewport_width, viewport_height = 1200, 900
last_x, last_y = 0, 0
left_pressed, right_pressed = False, False


def initialize_glfw_window(window_title="MuJoCo Simulation"):
    """
    Initialize a GLFW window for simulation visualization.
    Returns: window object
    """
    global last_x, last_y

    if not glfw.init():
        raise RuntimeError("GLFW initialization failed.")

    window = glfw.create_window(
        viewport_width, viewport_height, window_title, None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("Failed to create GLFW window.")
    glfw.make_context_current(window)
    glfw.swap_interval(1)
    last_x, last_y = glfw.get_cursor_pos(window)

    return window


def setup_mujoco_camera(model, camera_settings):
    """
    Initialize and set up MuJoCo camera parameters from a config dictionary.
    """
    cam = mj.MjvCamera()
    mj.mjv_defaultCamera(cam)

    cam.azimuth = camera_settings.get("azimuth", 90)
    cam.elevation = camera_settings.get("elevation", -30)
    cam.distance = camera_settings.get("distance", 6)
    cam.lookat[:] = np.array(camera_settings.get("lookat", [0.0, 0.0, 0.5]))

    opt = mj.MjvOption()
    mj.mjv_defaultOption(opt)

    scene = mj.MjvScene(model, maxgeom=10000)
    context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

    return cam, opt, scene, context


# Callbacks (shared across simulations)
def keyboard_callback(window, key, scancode, act, mods, model, data, custom_actions=None):
    """
    Keyboard callback for simulation reset and custom actions.
    """
    if act == glfw.PRESS:
        if key == glfw.KEY_BACKSPACE:
            mj.mj_resetData(model, data)
            mj.mj_forward(model, data)
            print("Simulation reset.")
        elif custom_actions:
            custom_actions(window, key, scancode, act, mods)


def mouse_button_callback(window, button, act, mods):
    global left_pressed, right_pressed, last_x, last_y
    if button == glfw.MOUSE_BUTTON_LEFT:
        left_pressed = (act == glfw.PRESS)
    if button == glfw.MOUSE_BUTTON_RIGHT:
        right_pressed = (act == glfw.PRESS)
    last_x, last_y = glfw.get_cursor_pos(window)


def mouse_move_callback(window, xpos, ypos, model, cam, scene):
    global last_x, last_y
    dx, dy = xpos - last_x, ypos - last_y
    last_x, last_y = xpos, ypos
    if left_pressed:
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ROTATE_V,
                          dx / viewport_height, dy / viewport_height, scene, cam)
    if right_pressed:
        mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_MOVE_H,
                          dx / viewport_height, dy / viewport_height, scene, cam)


def scroll_callback(window, xoffset, yoffset, model, cam, scene):
    mj.mjv_moveCamera(model, mj.mjtMouse.mjMOUSE_ZOOM,
                      0, -0.05 * yoffset, scene, cam)


def register_callbacks(window, model, data, cam, scene, custom_keyboard_handler=None):
    glfw.set_key_callback(window, lambda win, key, sc, act, mods:
                          keyboard_callback(win, key, sc, act, mods, model, data, custom_keyboard_handler))
    glfw.set_mouse_button_callback(window, mouse_button_callback)
    glfw.set_cursor_pos_callback(window, lambda win, xpos, ypos:
                                 mouse_move_callback(win, xpos, ypos, model, cam, scene))
    glfw.set_scroll_callback(window, lambda win, xoff, yoff:
                             scroll_callback(win, xoff, yoff, model, cam, scene))


def start_main_loop(window, model, data, cam, opt, scene, context, step_function, logger=None, record_video=False, video_writer=None):
    """
    Central simulation loop.
    """
    simulation_time = 0.0
    while not glfw.window_should_close(window):
        # Perform custom simulation step
        pos_new = step_function(model, data, dt=model.opt.timestep)
        simulation_time += model.opt.timestep

        if logger and pos_new is not None:
            if len(pos_new) == 3:
                x, y, z = pos_new
                logger.record(simulation_time, z, x, y)

        # Render visualization
        viewport_w, viewport_h = glfw.get_framebuffer_size(window)
        viewport = mj.MjrRect(0, 0, viewport_w, viewport_h)
        mj.mjv_updateScene(model, data, opt, None, cam,
                           mj.mjtCatBit.mjCAT_ALL.value, scene)
        mj.mjr_render(viewport, scene, context)

        # Video capture
        if record_video and video_writer is not None:
            rgb_buffer = np.zeros((viewport_h, viewport_w, 3), dtype=np.uint8)
            depth_buffer = np.zeros((viewport_h, viewport_w), dtype=np.float32)
            mj.mjr_readPixels(rgb_buffer, depth_buffer, viewport, context)
            frame = np.flipud(rgb_buffer)
            video_writer.append_data(frame)

        glfw.swap_buffers(window)
        glfw.poll_events()

    glfw.terminate()
