import numpy as np
from physics.physics import apply_impulse, apply_impulse_friction
from scipy.spatial.transform import Rotation as R
import mujoco as mj


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
    k = (1.0 / mass) + (1.0/18)

    # Normal impulse
    jn = -(1 + restitution) * u_rel_n / k

    # Compute friction impulse
    jt = np.zeros(3)
    if np.linalg.norm(u_rel_t) > 1e-6:  # Ensure no division by zero
        max_friction = friction_coeff * abs(jn)
        jt = -min(max_friction, np.linalg.norm(u_rel_t)) * \
            (u_rel_t / np.linalg.norm(u_rel_t))

    return jn, jt


def compute_inertia_tensor_world(inertia_diag, q):
    rot_matrix = R.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    return rot_matrix @ np.diag(inertia_diag) @ rot_matrix.T


def custom_step_with_impulse_collision_friction(model, obj, data, dt=0.01, restitution=1.0, friction_coeff=1.0, contact_threshold=0):
    mj.mj_forward(model, data)
    body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, f"{obj}")

    mass = model.body_mass[body_id]
    inertia_diag = model.body_inertia[body_id]
    inertia_world = compute_inertia_tensor_world(inertia_diag, data.qpos[3:7])

    vel = data.qvel[:3]
    omega = data.qvel[3:6]
    force = data.xfrc_applied[body_id, :3] + mass * model.opt.gravity
    torque = data.xfrc_applied[body_id, 3:]

    vel += (force / mass) * dt
    omega += np.linalg.inv(inertia_world) @ (torque * dt)

    for i in range(data.ncon):
        contact = data.contact[i]
        if not np.isnan(contact.dist) and contact.dist < 0:
            contact_point = contact.pos - data.qpos[:3]
            normal = contact.frame[:3]

            # If distance is very small (in contact but not colliding), rely on MuJoCo contact forces
            if abs(contact.dist) < contact_threshold:
                continue  # Skip manual impulse handling, MuJoCo will handle contact forces

            # Otherwise, treat it as a collision and resolve with impulses
            jn, jt = compute_collision_impulse_friction(
                mass, inertia_world, vel, omega, contact_point, normal, restitution, friction_coeff
            )
            vel, omega = apply_impulse_friction(
                vel, omega, mass, inertia_world, contact_point, normal, jn, jt
            )

    pos_new = data.qpos[:3] + vel * dt
    omega_quat = np.concatenate([[0], omega])
    res = np.zeros(4)
    mj.mju_mulQuat(res, omega_quat, data.qpos[3:7])
    quat_new = data.qpos[3:7] + 0.5 * res * dt
    quat_new /= np.linalg.norm(quat_new)

    data.qpos[:3] = pos_new
    data.qpos[3:7] = quat_new
    data.qvel[:3] = vel
    data.qvel[3:6] = omega

    return pos_new
