import numpy as np
import matplotlib.pyplot as plt

import ki_urdf as kinematic
from controller import highLevelController
from pso_tuning import run_pso


# ===== CONFIG =====
home_deg = [-179.40, -42.14, -132.36, -92.97, 89.72, 0]
q_init = np.deg2rad(home_deg)

target = np.array([-0.475, 0.110, 0.200])

obs_list = [
    np.array([-0.355, 0.110, 0.235])
]

axes_list = [
    np.array([0.025, 0.025, 0.025])
]

dt = 0.01
steps = 400


# ===== AUTO TUNING =====
print("=== RUN PSO (OFFLINE) ===")
best = run_pso(q_init, target, obs_list, axes_list)

# test nhanh nếu cần:
# best = [15, 1, 8, 10]

gamma, eta, mu, ai = best
params = [gamma, eta, mu, ai]

print("\nBest params:", params)


# ===== SIMULATION =====
q = q_init.copy()

trajectory = []
error_list = []

for i in range(steps):

    # ===== FK =====
    p, _, _, _ = kinematic.JointKinematics(q)
    x = p[:, 5]

    trajectory.append(x.copy())
    error_list.append(np.linalg.norm(x - target))

    # ===== CONTROLLER =====
    try:
        qdot_virtual, theta_dot = highLevelController(
            x, q, target, obs_list, axes_list, params
        )
    except Exception as e:
        print("Controller error:", e)
        break

    # ===== SAFETY =====
    if np.any(np.isnan(theta_dot)) or np.any(np.isinf(theta_dot)):
        print("NaN detected → STOP")
        break

    # ===== LIMIT SPEED (QUAN TRỌNG: đặt trước update) =====
    vmax = 0.3
    theta_dot = vmax * np.tanh(theta_dot / vmax)

    # ===== UPDATE =====
    q += theta_dot.flatten() * dt

    # ===== STOP =====
    if np.linalg.norm(x - target) < 0.01:
        print(f"Reached target at step {i}")
        break


trajectory = np.array(trajectory)


# ===== PLOT =====
plt.figure(figsize=(10, 5))

# --- trajectory ---
plt.subplot(1, 2, 1)
plt.plot(trajectory[:, 0], trajectory[:, 2], 'b-', label="Trajectory")
plt.scatter(target[0], target[2], c='g', label="Target")

# obstacle (tránh lặp label)
for idx, obs in enumerate(obs_list):
    if idx == 0:
        plt.scatter(obs[0], obs[2], c='r', label="Obstacle")
    else:
        plt.scatter(obs[0], obs[2], c='r')

plt.xlabel("X")
plt.ylabel("Z")
plt.title("End-effector trajectory (XZ)")
plt.legend()
plt.grid()

# --- error ---
plt.subplot(1, 2, 2)
plt.plot(error_list)
plt.title("Tracking Error")
plt.xlabel("Step")
plt.ylabel("Error")
plt.grid()

plt.tight_layout()
plt.show()