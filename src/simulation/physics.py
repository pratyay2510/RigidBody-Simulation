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


def apply_impulse_friction(vel, omega, mass, inertia_world, contact_point, normal, jn, jt):
    """
    Apply linear and angular impulse to update velocities.

    Args:
        vel (np.ndarray): Linear velocity (3x1).
        omega (np.ndarray): Angular velocity (3x1).
        mass (float): Mass of the body.
        inertia_world (np.ndarray): Inertia tensor in world coordinates (3x3).
        contact_point (np.ndarray): Vector from center-of-mass to contact point (3x1).
        normal (np.ndarray): Normal vector at contact (3x1).
        jn (float): Normal impulse magnitude.
        jt (np.ndarray): Tangential friction impulse (3x1).

    Returns:
        Updated vel and omega (tuple of np.ndarray).
    """
    impulse_normal = jn * normal
    impulse_tangent = jt  # Frictional impulse

    delta_v = (impulse_normal + impulse_tangent) / mass
    delta_omega = np.linalg.inv(
        inertia_world) @ np.cross(contact_point, (impulse_normal + impulse_tangent))

    return vel + delta_v, omega + delta_omega
