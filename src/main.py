"""
main.py

Entry point for the rigid body simulation Phase 1.
This module sets initial conditions for a single sphere (above a static floor),
updates its dynamics via physics and collision modules, and renders the simulation
using the custom visualization module.
"""

import time
from simulation.physics import Sphere, TIME_STEP
from simulation import collision
from visualization import viewer


def main():
    # Simulation parameters
    total_simulation_time = 5.0  # Total simulation time in seconds
    dt = TIME_STEP             # Time step for simulation

    # Initialize the sphere with default parameters
    sphere = Sphere(mass=1.0, radius=0.5)

    # Initialize the simulation viewer
    sim_viewer = viewer.Viewer()

    # Simulation time-stepping loop
    current_time = 0.0
    while current_time < total_simulation_time:
        # Update the sphere's velocity under gravity
        sphere.apply_gravity(dt)

        # Detect and process sphere-floor collision (with elastic collision)
        collision.handle_sphere_floor_collision(sphere, restitution=1.0)

        # Integrate the sphere's state (update position and orientation)
        sphere.integrate(dt)

        # Render the current state
        sim_viewer.render(sphere)

        # Increment simulation time
        current_time += dt

        # Optional: Sleep to mimic real-time simulation (adjust as needed)
        time.sleep(dt)


if __name__ == "__main__":
    main()
