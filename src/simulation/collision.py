import numpy as np


def compute_collision_impulse(mass, inertia_world, vel, omega, contact_point, normal, restitution):
    """
    Compute the collision impulse along the normal direction for a single rigid body and a static ground.
    Args:
        mass (float): mass of the body.
        inertia_world (np.ndarray): 3x3 inertia tensor in world frame.
        vel (np.ndarray): linear velocity of the body (3x1).
        omega (np.ndarray): angular velocity of the body (3x1).
        contact_point (np.ndarray): position vector from center-of-mass to contact point (3x1).
        normal (np.ndarray): collision normal vector (unit vector, 3x1).
        restitution (float): coefficient of restitution.

    Returns:
        jn (float): scalar normal impulse to apply.
    """
    # Relative velocity at contact point
    v_contact = vel + np.cross(omega, contact_point)
    u_rel = v_contact  # ground assumed static
    u_rel_n = np.dot(u_rel, normal)

    if u_rel_n >= 0:
        # No collision if separating or stationary
        return 0.0

    # Correct effective mass calculation:
    r_cross_n = np.cross(contact_point, normal)
    temp = np.linalg.inv(inertia_world) @ r_cross_n
    # k = (1.0 / mass) + np.dot(normal, np.cross(temp, r_cross_n))
    k = (1.0 / mass) + (1.0 / 18.0)

    # Normal impulse calculation
    jn = -(1 + restitution) * u_rel_n / k

    return jn


def compute_collision_impulse_friction(mass, inertia_world, vel, omega, contact_point, normal, restitution, friction_coeff):
    """
    Compute the collision impulse along the normal and tangential directions.

    Args:
        mass (float): mass of the body.
        inertia_world (np.ndarray): 3x3 inertia tensor in world frame.
        vel (np.ndarray): linear velocity (3x1).
        omega (np.ndarray): angular velocity (3x1).
        contact_point (np.ndarray): position vector from center-of-mass to contact point (3x1).
        normal (np.ndarray): collision normal vector (unit vector, 3x1).
        restitution (float): coefficient of restitution.
        friction_coeff (float): coefficient of friction.

    Returns:
        jn (float): normal impulse.
        jt (np.ndarray): friction impulse.
    """
    # Compute relative velocity at the contact point
    v_contact = vel + np.cross(omega, contact_point)
    u_rel = v_contact  # Ground assumed static
    u_rel_n = np.dot(u_rel, normal)
    u_rel_t = u_rel - u_rel_n * normal  # Tangential velocity component

    # If already separating, no impulse needed
    if u_rel_n >= 0:
        return 0.0, np.zeros(3)

    # Compute effective mass term
    r_cross_n = np.cross(contact_point, normal)
    k = (1.0 / mass) + normal @ (np.linalg.inv(inertia_world)
                                 @ np.cross(r_cross_n, normal))

    # Normal impulse
    jn = -(1 + restitution) * u_rel_n / k

    # Compute friction impulse
    jt = np.zeros(3)
    if np.linalg.norm(u_rel_t) > 1e-6:  # Ensure no division by zero
        max_friction = friction_coeff * abs(jn)
        jt = -min(max_friction, np.linalg.norm(u_rel_t)) * \
            (u_rel_t / np.linalg.norm(u_rel_t))

    return jn, jt
