import numpy as np
from controller import highLevelController
import ki_urdf as kinematic

def simulate(params, q_init, target, obs_list, axes_list):

    q = q_init.copy()
    dt = 0.01

    total_error = 0
    collision_penalty = 0

    for _ in range(300):

        # ===== FK =====
        p, _, _, _ = kinematic.JointKinematics(q)
        x = p[:,5]

        # ===== CONTROLLER =====
        try:
            qdot_virtual, theta_dot = highLevelController(
                x, q, target, obs_list, axes_list, params
            )
        except:
            return 1e9

        # ===== SAFETY =====
        if np.any(np.isnan(theta_dot)) or np.any(np.isinf(theta_dot)):
            return 1e9

        # ===== LIMIT SPEED =====
        vmax = 0.3
        theta_dot = vmax * np.tanh(theta_dot / vmax)

        # ===== UPDATE =====
        q += theta_dot.flatten() * dt

        # ===== ERROR =====
        err = np.linalg.norm(x - target)
        total_error += err**2

        # ===== SMOOTHNESS (phạt rung) =====
        total_error += 0.1 * np.linalg.norm(theta_dot)**2

        # ===== TIME PENALTY =====
        if err > 0.05:
            total_error += 0.01

        # ===== COLLISION =====
        for obs in obs_list:
            if np.linalg.norm(x - obs) < 0.03:
                collision_penalty += 1000

    return total_error + collision_penalty