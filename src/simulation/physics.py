"""
physics.py

This module contains functions for updating positions, orientations,
and velocities using forward Euler integration with gravity.
It also defines a Sphere class representing a rigid body sphere with
its properties, including mass, radius, and inertia tensor.
"""

import numpy as np

# Configurable parameters (can later be centralized in a config file)
GRAVITY = np.array([0.0, -9.81, 0.0])  # Gravitational acceleration [m/s^2]
TIME_STEP = 0.01  # Simulation time step [s]


# --- Quaternion Helper Functions --- #

def quaternion_multiply(q1, q2):
    """
    Multiplies two quaternions.

    Parameters:
        q1 (ndarray): First quaternion as [w, x, y, z].
        q2 (ndarray): Second quaternion as [w, x, y, z].

    Returns:
        ndarray: The product quaternion.
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w, x, y, z])


def normalize_quaternion(q):
    """
    Normalizes a quaternion to unit length.

    Parameters:
        q (ndarray): Quaternion as [w, x, y, z].

    Returns:
        ndarray: Normalized quaternion.
    """
    norm = np.linalg.norm(q)
    if norm == 0:
        return q
    return q / norm


# --- State Update Functions --- #

def update_position(position, velocity, dt):
    """
    Updates the position using forward Euler integration.

    Parameters:
        position (ndarray): Current position vector.
        velocity (ndarray): Current velocity vector.
        dt (float): Time step.

    Returns:
        ndarray: Updated position.
    """
    return position + velocity * dt


def update_velocity(velocity, force, mass, dt):
    """
    Updates the linear velocity using forward Euler integration.

    Parameters:
        velocity (ndarray): Current velocity vector.
        force (ndarray): Force vector applied.
        mass (float): Mass of the body.
        dt (float): Time step.

    Returns:
        ndarray: Updated velocity.
    """
    return velocity + (force / mass) * dt


def update_orientation(orientation, angular_velocity, dt):
    """
    Updates the orientation quaternion using forward Euler integration.
    Uses the formula:
        q_new = q + 0.5 * (q ⊗ omega_quat) * dt,
    where omega_quat = [0, ω_x, ω_y, ω_z].

    Parameters:
        orientation (ndarray): Current orientation quaternion [w, x, y, z].
        angular_velocity (ndarray): Angular velocity vector.
        dt (float): Time step.

    Returns:
        ndarray: Updated (normalized) orientation quaternion.
    """
    # Represent angular velocity as a pure quaternion (zero scalar part)
    omega_quat = np.array(
        [0.0, angular_velocity[0], angular_velocity[1], angular_velocity[2]])
    dq = quaternion_multiply(orientation, omega_quat) * 0.5 * dt
    new_orientation = orientation + dq
    return normalize_quaternion(new_orientation)


# --- Rigid Body: Sphere Class --- #

class Sphere:
    """
    Represents a rigid body sphere with physical properties and state.

    Attributes:
        mass (float): Mass of the sphere.
        radius (float): Radius of the sphere.
        inertia (ndarray): Inertia tensor (3x3 diagonal matrix).
        inertia_inv (ndarray): Inverse of the inertia tensor.
        position (ndarray): Position vector of the sphere's center.
        velocity (ndarray): Linear velocity vector.
        orientation (ndarray): Orientation quaternion [w, x, y, z].
        angular_velocity (ndarray): Angular velocity vector.
    """

    def __init__(self, mass=1.0, radius=0.5, position=None, velocity=None,
                 orientation=None, angular_velocity=None):
        """
        Initializes the Sphere with the given parameters.

        Parameters:
            mass (float): Mass of the sphere.
            radius (float): Radius of the sphere.
            position (ndarray, optional): Initial position vector.
            velocity (ndarray, optional): Initial linear velocity vector.
            orientation (ndarray, optional): Initial orientation quaternion.
            angular_velocity (ndarray, optional): Initial angular velocity vector.
        """
        self.mass = mass
        self.radius = radius

        # Inertia tensor for a solid sphere: I = (2/5) * m * r^2 (diagonal)
        I_value = (2.0 / 5.0) * self.mass * (self.radius ** 2)
        self.inertia = np.diag([I_value, I_value, I_value])
        self.inertia_inv = np.diag(
            [1.0 / I_value, 1.0 / I_value, 1.0 / I_value])

        # Initialize state variables
        if position is None:
            # Start slightly above the floor (which is at y = 0)
            self.position = np.array([0.0, self.radius + 0.1, 0.0])
        else:
            self.position = np.array(position, dtype=float)

        if velocity is None:
            self.velocity = np.zeros(3)
        else:
            self.velocity = np.array(velocity, dtype=float)

        if orientation is None:
            # Identity quaternion represents no rotation
            self.orientation = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            self.orientation = normalize_quaternion(
                np.array(orientation, dtype=float))

        if angular_velocity is None:
            self.angular_velocity = np.zeros(3)
        else:
            self.angular_velocity = np.array(angular_velocity, dtype=float)

    def integrate(self, dt=TIME_STEP):
        """
        Integrates the sphere's state over the time step dt.
        This updates the position and orientation.

        Parameters:
            dt (float): Time step.
        """
        self.position = update_position(self.position, self.velocity, dt)
        self.orientation = update_orientation(
            self.orientation, self.angular_velocity, dt)
        # Note: Velocity updates (linear and angular) are handled elsewhere (e.g., gravity, collisions)

    def apply_gravity(self, dt=TIME_STEP):
        """
        Applies gravitational force to update the sphere's linear velocity.

        Parameters:
            dt (float): Time step.
        """
        # Gravity force = mass * GRAVITY
        self.velocity = update_velocity(
            self.velocity, self.mass * GRAVITY, self.mass, dt)
