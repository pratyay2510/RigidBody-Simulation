import numpy as np


def apply_impulse(vel, omega, mass, inertia_world, contact_point, normal, impulse):
    """
    Apply linear and angular impulse to update velocities.
    Args:
        vel (np.ndarray): current linear velocity (3x1)
        omega (np.ndarray): current angular velocity (3x1)
        mass (float): body mass
        inertia_world (np.ndarray): inertia tensor in world coordinates (3x3)
        contact_point (np.ndarray): vector from center-of-mass to contact point (3x1)
        normal (np.ndarray): impulse direction (3x1)
        impulse (float): impulse magnitude

    Returns:
        Updated vel and omega (tuple of np.ndarray)
    """
    delta_v = (impulse / mass) * normal
    delta_omega = np.linalg.inv(
        inertia_world) @ np.cross(contact_point, impulse * normal)
    return vel + delta_v, omega + delta_omega
