# src/physics/step_schemes.py

import numpy as np
import mujoco as mj
from src.physics.collision import compute_collision_impulse_friction
from src.physics.physics_utils import apply_impulse_friction
from scipy.spatial.transform import Rotation as R


def compute_inertia_tensor_world(inertia_diag, q):
    """
    Compute inertia tensor in world coordinates from diagonal form and quaternion rotation.
    """
    rot_matrix = R.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    return rot_matrix @ np.diag(inertia_diag) @ rot_matrix.T


def step_with_custom_collisions(model, data, ball_radius, mass1, mass2, I_inv_ball1, I_inv_ball2,
                                friction_coefficient, restitution, dt=0.01):
    """
    Custom collision step for two-ball collision simulation.
    Properly applies collision and friction impulses, avoiding sticking issues.
    """
    mj.mj_forward(model, data)

    # Gravity effect
    for pos_idx, vel_idx in [(0, 0), (7, 6)]:
        data.qvel[vel_idx:vel_idx + 3] += model.opt.gravity * dt

    # Ball-ground collisions (impulse application fixed)
    for pos_idx, vel_idx, ang_idx, mass, I_inv in [
        (0, 0, 3, mass1, I_inv_ball1),
        (7, 6, 9, mass2, I_inv_ball2)
    ]:
        pos = data.qpos[pos_idx:pos_idx + 3]
        linvel = data.qvel[vel_idx:vel_idx + 3]
        angvel = data.qvel[ang_idx:ang_idx + 3]
        normal = np.array([0.0, 0.0, 1.0])

        if pos[2] < ball_radius:
            contact_point = pos - ball_radius * normal
            r = contact_point - pos
            jn, jt = compute_collision_impulse_friction(
                mass, I_inv, linvel, angvel, r, normal, restitution, friction_coefficient
            )
            linvel, angvel = apply_impulse_friction(
                linvel, angvel, mass, I_inv, r, normal, jn, jt
            )
            data.qvel[vel_idx:vel_idx + 3] = linvel
            data.qvel[ang_idx:ang_idx + 3] = angvel
            data.qpos[pos_idx + 2] = ball_radius

    # Ball-ball collisions (use proper impulse application)
    diff = data.qpos[7:10] - data.qpos[0:3]
    dist = np.linalg.norm(diff)
    tol = 0.01
    if dist < 2 * ball_radius + tol:
        normal = diff / (dist + 1e-8)
        contact_point = (data.qpos[0:3] + data.qpos[7:10]) / 2.0
        r1 = contact_point - data.qpos[0:3]
        r2 = contact_point - data.qpos[7:10]

        # Ball 1 impulse
        jn, jt = compute_collision_impulse_friction(
            mass1, I_inv_ball1, data.qvel[0:3], data.qvel[3:
                                                          6], r1, normal, restitution, friction_coefficient
        )
        v1, w1 = apply_impulse_friction(
            data.qvel[0:3], data.qvel[3:6], mass1, I_inv_ball1, r1, normal, jn, jt
        )
        data.qvel[0:3] = v1
        data.qvel[3:6] = w1

        # Ball 2 impulse (opposite direction)
        v2, w2 = apply_impulse_friction(
            data.qvel[6:9], data.qvel[9:12], mass2, I_inv_ball2, r2, -
            normal, jn, jt
        )
        data.qvel[6:9] = v2
        data.qvel[9:12] = w2

        # Positional correction to avoid overlap
        correction = (2 * ball_radius + tol - dist) / 2.0
        data.qpos[0:3] -= correction * normal
        data.qpos[7:10] += correction * normal

    # Integrate updated positions
    for pos_idx, vel_idx in [(0, 0), (7, 6)]:
        data.qpos[pos_idx:pos_idx + 3] += data.qvel[vel_idx:vel_idx + 3] * dt

    return data.qpos[0:3], data.qpos[7:10]


def custom_step_multi_sphere(model, data, ball_names, friction_coefficient, restitution, dt=0.01, logger=None):
    """
    Custom simulation step for multi-sphere simulation with impulse-based collisions.
    """
    mj.mj_forward(model, data)
    simulation_time = data.time

    for ball_name in ball_names:
        ball_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, ball_name)
        mass = model.body_mass[ball_id]
        inertia_diag = model.body_inertia[ball_id]
        qpos = data.qpos[ball_id * 7: ball_id * 7 + 7]
        qvel = data.qvel[ball_id * 6: ball_id * 6 + 6]

        vel = qvel[:3]
        omega = qvel[3:6]
        inertia_world = compute_inertia_tensor_world(inertia_diag, qpos[3:7])

        force = data.xfrc_applied[ball_id, :3] + mass * model.opt.gravity
        torque = data.xfrc_applied[ball_id, 3:]
        vel += (force / mass) * dt
        omega += np.linalg.inv(inertia_world) @ (torque * dt)

        # Collision handling
        for i in range(data.ncon):
            contact = data.contact[i]
            if contact.dist < 0 and ball_name in [model.id2name(contact.geom1), model.id2name(contact.geom2)]:
                contact_point = contact.pos - qpos[:3]
                normal = contact.frame[:3]
                jn, jt = compute_collision_impulse_friction(
                    mass, inertia_world, vel, omega, contact_point, normal, restitution, friction_coefficient
                )
                vel, omega = apply_impulse_friction(
                    vel, omega, mass, inertia_world, contact_point, normal, jn, jt
                )

        # Integrate position and rotation
        pos_new = qpos[:3] + vel * dt
        omega_quat = np.concatenate([[0], omega])
        res = np.zeros(4)
        mj.mju_mulQuat(res, omega_quat, qpos[3:7])
        quat_new = qpos[3:7] + 0.5 * res * dt
        quat_new /= np.linalg.norm(quat_new)

        data.qpos[ball_id * 7: ball_id * 7 + 3] = pos_new
        data.qpos[ball_id * 7 + 3: ball_id * 7 + 7] = quat_new
        data.qvel[ball_id * 6: ball_id * 6 + 3] = vel
        data.qvel[ball_id * 6 + 3: ball_id * 6 + 6] = omega

        if logger:
            logger.record(ball_name, simulation_time, pos_new)
