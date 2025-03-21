import numpy as np
import mujoco as mj
from simulation.physics import apply_impulse_friction
from simulation.collision import compute_collision_impulse_friction
from scipy.spatial.transform import Rotation as R


def compute_inertia_tensor_world(inertia_diag, q):
    rot_matrix = R.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    return rot_matrix @ np.diag(inertia_diag) @ rot_matrix.T


def timestep_integration(model, obj, data, dt=0.01, restitution=1.0, friction_coeff=0.5, contact_threshold=1e-4):
    """
    Perform one timestep of simulation with custom impulse-based collision handling.

    Args:
        model: MuJoCo model.
        obj: Name of the object body in the model.
        data: MuJoCo data object.
        dt: Timestep.
        restitution: Restitution coefficient.
        friction_coeff: Friction coefficient.
        contact_threshold: Threshold below which contact is considered stable contact.

    Returns:
        pos_new (np.ndarray): New position of the object.
    """
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

            if abs(contact.dist) < contact_threshold:
                continue

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


def general(model, obj, data, dt=0.01, restitution=1.0, friction_coeff=0.5, contact_threshold=1e-4):
    """
    Perform one timestep of simulation using a step scheme:
    1. Update positions using old velocities.
    2. Apply external forces to update velocity.
    3. Detect and handle collisions.
    4. Resolve contact.

    Args:
        model: MuJoCo model.
        obj: Name of the object body in the model.
        data: MuJoCo data object.
        dt: Timestep.
        restitution: Restitution coefficient.
        friction_coeff: Friction coefficient.
        contact_threshold: Threshold below which contact is considered stable contact.

    Returns:
        pos_new (np.ndarray): New position of the object.
    """
    mj.mj_forward(model, data)
    body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, f"{obj}")

    mass = model.body_mass[body_id]
    inertia_diag = model.body_inertia[body_id]
    inertia_world = compute_inertia_tensor_world(inertia_diag, data.qpos[3:7])

    vel_old = data.qvel[:3]
    omega_old = data.qvel[3:6]

    # Step 1: Update positions using old velocities
    pos_predicted = data.qpos[:3] + vel_old * dt

    # Step 2: Apply external forces to update velocity
    force = data.xfrc_applied[body_id, :3] + mass * model.opt.gravity
    torque = data.xfrc_applied[body_id, 3:]

    vel_new = vel_old + (force / mass) * dt
    omega_new = omega_old + np.linalg.inv(inertia_world) @ (torque * dt)

    # Step 3: Detect and handle collisions
    for i in range(data.ncon):
        contact = data.contact[i]
        if not np.isnan(contact.dist) and contact.dist < 0:
            contact_point = contact.pos - data.qpos[:3]
            normal = contact.frame[:3]

            if abs(contact.dist) < contact_threshold:
                continue  # Stable contact, let MuJoCo handle it

            # Apply impulse-based collision resolution
            jn, jt = compute_collision_impulse_friction(
                mass, inertia_world, vel_new, omega_new, contact_point, normal, restitution, friction_coeff
            )
            vel_new, omega_new = apply_impulse_friction(
                vel_new, omega_new, mass, inertia_world, contact_point, normal, jn, jt
            )

    # Step 4: Resolve contact (adjust position if necessary)
    pos_new = pos_predicted  # Assuming perfect integration

    # Update MuJoCo state
    data.qpos[:3] = pos_new
    data.qvel[:3] = vel_new
    data.qvel[3:6] = omega_new

    return pos_new
