from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
import numpy as np
import time
import os
import kinematic as kinematic

IP = "169.254.200.99"

rtde_r = RTDEReceiveInterface(IP)
rtde_c = RTDEControlInterface(IP)

print("Running")
#cho về home trước khi bắt đầu
home_q = [0, 0, 0, 0, 0, 0]
print("Moving to home position...")
# moveJ: đi tới vị trí joint an toàn
rtde_c.moveJ(home_q, speed=1.0, acceleration=1.0)
# đợi robot ổn định
time.sleep(2)
print("Reached home. Start control!")
# === LOG ===
folder = "logs_pro_1504"
os.makedirs(folder, exist_ok=True)

base_name = "0obs"
i = 1
while True:
    filename = os.path.join(folder, f"{base_name}_{i}.csv")
    if not os.path.exists(filename):
        break
    i += 1

record_vars = ["timestamp", "actual_TCP_pose", "actual_TCP_speed"]
rtde_r.startFileRecording(filename, record_vars)
# ===== TARGET =====
target = np.array([-0.217, -0.196, 0.562])

# ===== OBSTACLE =====
obs_list = []
axes_list = []

# ===== PARAM =====
no_obs_printed = False
dt = 0.008

theta_min = np.array([-2*np.pi]*6)
theta_max = np.array([ 2*np.pi]*6)

# ===== CONTROL PARAM =====
lamda = 1
gamma = 50
eta   = 1
mu    = 8
ai    = 180
beta  = 0.9

Ki = np.diag([1,1,1,1,1,3])
K = np.eye(6)

def to2d_xz(p3):
    return np.array([p3[0], p3[2]])

def jointMapping(q_v):
    #chống overflow exp
    exp_term = np.exp(-lamda*q_v)
    exp_term = np.clip(exp_term, 1e-9, 1e9)

    theta = (theta_max - theta_min)/(1+exp_term) + theta_min

    dg = lamda*(theta_max-theta_min)*exp_term/((1+exp_term)**2)
    dg = np.clip(dg, 1e-9, None)

    Jg = np.diag(dg)
    return theta, Jg

def highLevelController(x, theta):
    global no_obs_printed

    # ===== SAFE MAPPING =====
    theta_safe = np.clip(theta, theta_min + 1e-6, theta_max - 1e-6)

    ratio = (theta_max - theta_min) / (theta_safe - theta_min)
    inner = ratio - 1
    inner = np.maximum(inner, 1e-9)

    q_virtual = -np.log(inner) / lamda

    # clamp tránh overflow
    #q_virtual = np.clip(q_virtual, -20, 20)

    # ===== ERROR =====
    e = (np.array([x[0], x[2]]) - np.array([target[0], target[2]])).reshape(1, -1)

    # ===== JACOBIAN =====
    p, Jp = kinematic.JointKinematics(theta)
    Jv = Jp[[0,2], :, 5]

    A_task = gamma * e @ Jv
    B = 0.0
    A_obs_total = np.zeros(6)

    # ===== OBSTACLE CHECK =====
    if len(obs_list) == 0:
        if not no_obs_printed:
            print("No obstacles, moving straight to target.")
            no_obs_printed = True
    else:
        no_obs_printed = False

    A = A_task - A_obs_total
    A = A.reshape(1, -1)

    if (A @ A.T) < 1e-6:
        Aplus = np.zeros((6,1))
    else:
        Aplus = A.T / (A @ A.T + 1e-4)

    rhs = -eta * A.T - Aplus * B

    _, Jg = jointMapping(q_virtual)

    #check NaN trước pinv
    if np.any(np.isnan(Jg)) or np.any(np.isinf(Jg)):
        print("Invalid Jg!")
        return np.zeros((6,1)), np.zeros((6,1))

    qdot_virtual = K @ np.linalg.pinv(Jg) @ rhs

    # ===== GIỮ 2D =====
    qdot_virtual[0] = 0
    qdot_virtual[4] = 0
    qdot_virtual[5] = 0

    theta_dot = Jg @ qdot_virtual

    return qdot_virtual, theta_dot

# ===== MAIN LOOP =====
try:
    step_count = 0
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
            qdot_virtual, theta_dot = highLevelController(x_curr, theta)
        except Exception as e:
            print("\nController ERROR:", e)
            break

        # check NaN
        if np.any(np.isnan(theta_dot)) or np.any(np.isinf(theta_dot)):
            print("\nNaN detected → STOP")
            break

        theta_dot = np.clip(theta_dot, -0.3, 0.3)

        # ===== STOP =====
        dist = np.linalg.norm(x_curr - target)
        if dist < 0.005:
            rtde_c.speedJ([0]*6, 0.2, dt)
            print(f"\nReached target! Time={elapsed:.2f}s")
            break

        # ===== SEND =====
        rtde_c.speedJ(theta_dot.flatten().tolist(), acceleration=0.5, time=dt)

        # ===== PRINT =====
        if step_count % int(1.0/dt) == 0:
            print(f"\rv={theta_dot.round(3)} | "
                  f"dist={dist:.4f} | "
                  f"pos=({x_curr[0]:.3f}, {x_curr[1]:.3f}, {x_curr[2]:.3f})",
                  end="", flush=True)

        rtde_c.waitPeriod(t_start)
        rtde_c.kickWatchdog()

except KeyboardInterrupt:
    print("Stopped")

finally:
    print("\nStopping robot safely...")
    try:
        rtde_r.stopFileRecording()
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