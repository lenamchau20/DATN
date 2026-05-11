from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
import numpy as np
import time
import os

import ki_urdf as kinematic
from controller import highLevelController
from pso_tuning import run_pso

IP = "169.254.200.99"

rtde_r = RTDEReceiveInterface(IP)
rtde_c = RTDEControlInterface(IP)

print("Running")

# ===== HOME =====
home_deg = [-179.40, -42.14, -132.36, -92.97, 89.72, 0]
home_q = np.deg2rad(home_deg).tolist()

print("Moving to home position...")
rtde_c.moveJ(home_q, 0.5, 1.0)
time.sleep(2)
print("Reached home. Start control!")

# ===== TARGET =====
target = np.array([-0.475, 0.110, 0.200])

# ===== OBSTACLE =====
obs_list = [
    np.array([-0.355, 0.110, 0.235])
]

axes_list = [
    np.array([0.025, 0.025, 0.025])
]

# ===== AUTO TUNING (PSO) =====
print("\n=== PSO AUTO-TUNING ===")
q_init = np.array(home_q)

# ⚠️ nếu muốn test nhanh thì comment dòng dưới
best = run_pso(q_init, target, obs_list, axes_list)

# test nhanh:
# best = [15, 1, 8, 10]

gamma, eta, mu, ai = best
params = [gamma, eta, mu, ai]

print("\n=== OPTIMIZED PARAM ===")
print(f"gamma = {gamma:.3f}")
print(f"eta   = {eta:.3f}")
print(f"mu    = {mu:.3f}")
print(f"ai    = {ai:.3f}")

# ===== LOG =====
folder = "logs_pro_2104"
os.makedirs(folder, exist_ok=True)

filename = os.path.join(folder, "log.csv")
log_f = open(filename, "w")

header = "time"
for j in range(6):
    header += f",J{j+1}_x,J{j+1}_y,J{j+1}_z"
for j in range(6):
    header += f",qdot{j+1}"

log_f.write(header + "\n")

# ===== CONTROL =====
dt = 0.008
step_count = 0

try:
    rtde_c.setWatchdog(0.2)

    while True:
        t_start = rtde_c.initPeriod()

        step_count += 1
        elapsed = step_count * dt

        tcp = rtde_r.getActualTCPPose()
        x_curr = np.array(tcp[:3])
        theta = np.array(rtde_r.getActualQ())

        # ===== CONTROLLER =====
        try:
            qdot_virtual, theta_dot = highLevelController(
                x_curr,
                theta,
                target,
                obs_list,
                axes_list,
                params
            )
        except Exception as e:
            print("\nController ERROR:", e)
            break

        # ===== SAFETY =====
        if np.any(np.isnan(theta_dot)) or np.any(np.isinf(theta_dot)):
            print("\nNaN detected → STOP")
            break

        # ===== LIMIT SPEED =====
        vmax = 0.3
        theta_dot = vmax * np.tanh(theta_dot / vmax)

        # ===== FK (LOG) =====
        p, _, _, _ = kinematic.JointKinematics(theta)

        log_line = f"{elapsed:.4f}"

        for i in range(6):
            log_line += f",{p[0,i]:.4f},{p[1,i]:.4f},{p[2,i]:.4f}"

        for i in range(6):
            log_line += f",{theta_dot[i,0]:.4f}"

        log_f.write(log_line + "\n")

        # ===== STOP CONDITION =====
        dist = np.linalg.norm(x_curr - target)
        if dist < 0.01:
            rtde_c.speedJ([0]*6, acceleration=0.1, time=dt)
            print(f"\nReached target! Time = {elapsed:.2f}s")
            break

        # ===== SEND =====
        safe_speed = theta_dot.flatten().astype(float).tolist()
        rtde_c.speedJ(safe_speed, acceleration=0.1, time=dt)

        # ===== PRINT =====
        if step_count % int(1.0/dt) == 0:
            print(f"\rdist={dist:.4f} | pos=({x_curr[0]:.3f}, {x_curr[1]:.3f}, {x_curr[2]:.3f})",
                  end="", flush=True)

        rtde_c.waitPeriod(t_start)
        rtde_c.kickWatchdog()

except KeyboardInterrupt:
    print("Stopped")

finally:
    print("\nStopping robot safely...")

    try:
        log_f.close()
    except:
        pass

    try:
        rtde_c.speedStop()
    except:
        pass

    try:
        rtde_c.stopScript()
    except:
        pass