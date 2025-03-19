"""
collision.py

This module implements collision detection and impulse-based collision
response routines. For Phase 1, we handle the collision of a sphere with
a static, horizontal floor at y = 0 using a simple impulse-based approach.
"""

import numpy as np
from simulation.physics import Sphere

# Configurable parameter for restitution (coefficient of elasticity)
DEFAULT_RESTITUTION = 1.0  # 1.0 for perfectly elastic collisions


def handle_sphere_floor_collision(sphere, restitution=DEFAULT_RESTITUTION):
    """
    Detects and processes a collision between a sphere and a static floor at y = 0.
    The collision is resolved using an impulse-based approach.

    Parameters:
        sphere (Sphere): The sphere object to test for collision.
        restitution (float): Coefficient of restitution (ε). A value of 1.0
                             indicates a perfectly elastic collision.
    """
    # A collision is detected if the sphere's lowest point is below the floor.
    if sphere.position[1] - sphere.radius < 0:
        # Define the contact normal (upward vertical unit vector)
        N = np.array([0.0, 1.0, 0.0])

        # For a sphere-floor collision, assume the contact point is directly below
        # the center of the sphere.
        r = np.array([0.0, -sphere.radius, 0.0])

        # Compute the relative velocity at the contact point:
        # v_rel = v + (ω × r)
        rel_vel = sphere.velocity + np.cross(sphere.angular_velocity, r)
        rel_vel_normal = np.dot(rel_vel, N)

        # Only process collision if the sphere is moving toward the floor.
        if rel_vel_normal < 0:
            # Compute the effective mass (K) at the collision point.
            # For the sphere: K = 1/m + N^T * [I_inv (r x N)_x] * (r x N)
            r_cross_N = np.cross(r, N)
            term = np.dot(N, np.cross(sphere.inertia_inv.dot(r_cross_N), r))
            effective_mass = 1.0 / sphere.mass + term

            # Compute the normal impulse scalar.
            j_n = -(1.0 + restitution) * rel_vel_normal / effective_mass

            # Update the sphere's linear velocity:
            sphere.velocity = sphere.velocity + (j_n / sphere.mass) * N

            # Update the sphere's angular velocity:
            sphere.angular_velocity = sphere.angular_velocity + sphere.inertia_inv.dot(
                np.cross(r, j_n * N)
            )

            # Correct the sphere's position to avoid interpenetration.
            penetration_depth = sphere.radius - sphere.position[1]
            if penetration_depth > 0:
                sphere.position[1] += penetration_depth
