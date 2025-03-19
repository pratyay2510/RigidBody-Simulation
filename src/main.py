"""
main.py

Entry point for the rigid body simulation Phase 1 using MuJoCo for rendering.
This module initializes a MuJoCo model (assumed to be defined in models/sphere.xml)
that contains a free body representing a sphere bouncing on a static floor.
Our custom physics simulation (in simulation/physics.py and simulation/collision.py)
updates the sphere’s state, and we then update the corresponding MuJoCo data
so that MuJoCo’s rendering shows the simulated ball motion.

Assumptions:
    - The XML file (models/sphere.xml) defines a world with a static floor and a single
      free body (the sphere) at body index 1.
    - The custom simulation uses a forward Euler integration scheme.
    - MuJoCo is used only for visualization (we do not use its built-in dynamics engine).
"""

import os
import time
import numpy as np

import mujoco as mj
from mujoco.glfw import glfw

# Import our custom simulation modules
from simulation.physics import Sphere, TIME_STEP, GRAVITY
from simulation import collision

# --- MuJoCo Model Setup --- #

# Set the XML path to the MuJoCo model file for the sphere.
# (Make sure this XML file is set up appropriately. Here we assume it's in the "models" folder.)
xml_path = os.path.join(os.path.dirname(__file__),
                        "..", "models", "sphere.xml")

# Load the MuJoCo model and data.
model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)

# Create visualization objects
cam = mj.MjvCamera()
opt = mj.MjvOption()
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# Get the absolute path of the XML (if needed for debugging)
xml_path = os.path.abspath(xml_path)

# --- Callback Functions (optional) --- #


def keyboard(window, key, scancode, act, mods):
    # Reset simulation when Backspace is pressed
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        # Reset our custom simulation sphere (if needed, you could reinitialize it)
        # For MuJoCo, reset the data and forward the state.
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)


def mouse_button(window, button, act, mods):
    # (Optional) Implement mouse callbacks if desired.
    pass


def mouse_move(window, xpos, ypos):
    # (Optional) Implement camera control callbacks if desired.
    pass


def scroll(window, xoffset, yoffset):
    # (Optional) Implement scroll callbacks for zoom if desired.
    pass


# Install GLFW callbacks
glfw.set_key_callback(glfw.get_current_context(), keyboard)
glfw.set_mouse_button_callback(glfw.get_current_context(), mouse_button)
glfw.set_cursor_pos_callback(glfw.get_current_context(), mouse_move)
glfw.set_scroll_callback(glfw.get_current_context(), scroll)

# --- Initialize GLFW and Create Window --- #
if not glfw.init():
    raise RuntimeError("Could not initialize GLFW")

window = glfw.create_window(
    1200, 900, "MuJoCo Ball Bounce Simulation", None, None)
if not window:
    glfw.terminate()
    raise RuntimeError("Could not create GLFW window")

glfw.make_context_current(window)
glfw.swap_interval(1)  # Enable v-sync

# --- Initialize Custom Simulation State --- #
# Create our simulation sphere with default parameters.
# Its initial position is set above the floor.
sphere = Sphere(mass=1.0, radius=0.5)

# (Optional) Set initial conditions explicitly if needed:
# sphere.position = np.array([0.0, sphere.radius + 0.1, 0.0])
# sphere.velocity = np.zeros(3)

# Define which MuJoCo body index corresponds to our sphere.
# Typically, index 0 is the world body, so we assume our sphere is at index 1.
ball_body_id = 1

# Simulation parameters
total_simulation_time = 5.0  # seconds
dt = TIME_STEP             # time step (s)
sim_time = 0.0             # custom simulation time

# --- Main Simulation Loop --- #
while not glfw.window_should_close(window) and sim_time < total_simulation_time:
    time_start = time.time()

    # --- Update Custom Physics ---
    # Apply gravity and update the sphere state.
    sphere.apply_gravity(dt)
    collision.handle_sphere_floor_collision(sphere, restitution=1.0)
    sphere.integrate(dt)

    # Update our simulation time.
    sim_time += dt

    # Optionally, store our custom simulation time in data.time
    data.time = sim_time

    # --- Update MuJoCo Data with Simulation State ---
    # Overwrite the MuJoCo body position and orientation for the sphere.
    data.body_xpos[ball_body_id] = sphere.position
    # MuJoCo expects quaternions in [w, x, y, z] format.
    data.body_xquat[ball_body_id] = sphere.orientation

    # Propagate the new state throughout the MuJoCo data (e.g., update kinematics).
    mj.mj_forward(model, data)

    # --- Render the Scene using MuJoCo ---
    # Get the viewport dimensions.
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # Update the scene (using the current state in data).
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # Swap buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)
    glfw.poll_events()

    # Sleep to enforce the time step if necessary.
    elapsed = time.time() - time_start
    if dt - elapsed > 0:
        time.sleep(dt - elapsed)

# Terminate GLFW when done.
glfw.terminate()
