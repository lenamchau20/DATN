from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
import numpy as np
import time
import os
import ki_urdf as kinematic

IP = "169.254.200.99"

rtde_r = RTDEReceiveInterface(IP)
rtde_c = RTDEControlInterface(IP)

print("Running")
#cho về home trước khi bắt đầu
#initialPosition = [-0.457; -0.196; 0.069];
#home_deg = [-28.87, -84.67, 109.64, -114.14, -90, 0]        # ← Bạn chỉnh ở đây (đơn vị: độ)
home_deg = [-179.40, -42.14, -132.36, -92.97, 89.72, 0] 
# Tự động chuyển sang radian
home_q = np.deg2rad(home_deg).tolist()
print("Moving to home position...")
# moveJ: đi tới vị trí joint an toàn
rtde_c.moveJ(home_q, 0.5, 1.0)
print("Reached home position.")
# đợi robot ổn định
time.sleep(2)
print("Reached home. Start control!")
# === LOG ===
folder = "logs_pro_0905"
os.makedirs(folder, exist_ok=True)

base_name = "obs"
i = 1
while True:
    filename = os.path.join(folder, f"{base_name}_{i}.csv")
    joint_filename = filename.replace(".csv", "_joint.csv")

    if not os.path.exists(filename):
        # mở file log joint
        log_f = open(joint_filename, "w")

        # header
        header = "time"
        for j in range(6):
            header += f",J{j+1}_x,J{j+1}_y,J{j+1}_z"
        for j in range(6):
            header += f",qdot{j+1}"

        log_f.write(header + "\n")
        break

    i += 1

record_vars = ["timestamp", "actual_TCP_pose", "actual_TCP_speed"]
rtde_r.startFileRecording(filename, record_vars)
# ===== TARGET =====
#target = np.array([-0.217, -0.195, 0.562])
target = np.array([-0.475, 0.110, 0.300])
# ===== OBSTACLE =====
# ===== OBSTACLE (ELLIPSOID) =====
obs_list = [
    # np.array([-0.235, 0.110, 0.250])
     np.array([-0.355, 0.110, 0.235])
]

axes_list = [
     np.array([0.025, 0.025, 0.025])
]

# ===== PARAM =====
no_obs_printed = False
dt = 0.008

theta_min = np.array([-2*np.pi]*6)
theta_max = np.array([ 2*np.pi]*6)

# ===== CONTROL PARAM =====
lamda = 1
gamma = 15
eta   = 1
mu    = 8   #8
ai    = 10  #20 
beta  = 0.9 #0.9
sigma = 0.1
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
    #==========DEBUG==========
    if not hasattr(highLevelController, "lastPrintTime"):
        highLevelController.lastPrintTime = time.time()
    # ===== SAFE MAPPING =====
    theta_safe = np.clip(theta, theta_min + 1e-6, theta_max - 1e-6)

    ratio = (theta_max - theta_min) / (theta_safe - theta_min)
    q_virtual = -np.log(np.maximum(ratio - 1, 1e-9)) / lamda

    # ===== ERROR =====
    e = (np.array([x[0], x[2]]) - np.array([target[0], target[2]])).reshape(1, -1)
    # debugStr = f"\n[EE] ||e|| = {np.linalg.norm(e):.3f} "
    # ===== JACOBIAN =====
    p, Jp, T06, _ = kinematic.JointKinematics(theta)
    Jv = Jp[[0,2], :, 5]  # end-effector

    A_task = gamma * e @ Jv
    xdot_d = np.zeros((2,1))
    #xdot_d = -5 * e.T
    B = (-gamma * e @ xdot_d).item()
    A_obs_total = np.zeros((1, 6))

    # ===== OBSTACLE HANDLING =====
    if len(obs_list) == 0:
        if not no_obs_printed:
            print("No obstacles, moving straight to target.")
            no_obs_printed = True
    else:
        no_obs_printed = False

        for obs_pos, axes in zip(obs_list, axes_list):

            xobs_2d = np.array([obs_pos[0], obs_pos[2]])
            xdot_obs = np.zeros((2,1))
            R1 = axes[0]
            R2 = 1.5 * R1
 
            alpha = np.exp(-(R2**2 - R1**2)) / beta
            # debugStr = f"\n alpha = {alpha:.3f} "
            for i in range(5,6):
                if i == 5:
                    xi = np.array([x[0], x[2]])
                    Jvi = Jv
                else:
                    xi = np.array([p[0,i], p[2,i]])
                    Jvi = Jp[[0,2], :, i]

                dist_vec = (xi - xobs_2d).reshape(1, -1)
                d_iobs = (dist_vec @ dist_vec.T).item()
                dist = np.sqrt(d_iobs)

                
                d_i = np.exp(-(d_iobs - R1**2)/sigma)

                expTerm = np.exp(-mu*(d_i - alpha))
                aii = ai * mu * expTerm / (1 + expTerm)**2

                # ===== OBSTACLE FORCE =====
                A_obs_i = (aii * d_i * dist_vec @ Jvi) @ Ki
                
                A_obs_total += A_obs_i

                B += (aii * d_i * (dist_vec @ xdot_obs)).item()

                # ===== DEBUG =====
                status = ""
                if dist < R1:
                    print("\n COLLISION!")
                elif dist < 1.5 * R1:
                    print("\n NEAR OBSTACLE")
                # debugStr += f"\n L{i+1} | dist={dist:6.3f} | d_i={d_i:6.3f} | aii={aii:7.3f} | di-alpha={d_i - alpha:7.3f} | A_obs_i = {A_obs_i[0,0]:7.3f} {status}"

    # ===== FINAL CONTROL =====
    A = A_task - A_obs_total
    A = A.reshape(1, -1)

    if (A @ A.T) < 1e-6:
        Aplus = np.zeros((6,1))
    else:
        Aplus = A.T / (A @ A.T + 1e-3)

    rhs = -eta * A.T - Aplus * B

    _, Jg = jointMapping(q_virtual)

    # ===== SAFETY CHECK =====
    if np.any(np.isnan(Jg)) or np.any(np.isinf(Jg)):
        print("Invalid Jg!")
        return np.zeros((6,1)), np.zeros((6,1))

    qdot_virtual = K @ np.linalg.pinv(Jg) @ rhs

    # ===== GIỮ 2D =====
    qdot_virtual[0] = 0
    qdot_virtual[4] = 0
    qdot_virtual[5] = 0

    theta_dot = Jg @ qdot_virtual
    # debugStr += f"\n >>> AVOIDANCE ACTIVE <<<" if np.linalg.norm(A_obs_total) > 1e-3 else ""
    # debugStr += f"\n[CTRL] A_t={np.linalg.norm(A_task):6.2f} | " \
    #             f"A_o={np.linalg.norm(A_obs_total):6.2f} | " \
    #             f"A={np.linalg.norm(A):6.2f} | " \
    #             f"qdot={np.linalg.norm(qdot_virtual):6.2f}\n"
    # if time.time() - highLevelController.lastPrintTime > 1.0:
    #     print(debugStr)
    #     highLevelController.lastPrintTime = time.time()

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

        vmax = 0.3
        theta_dot = vmax * np.tanh(theta_dot / vmax)
        # ===== TÍNH FK để lấy vị trí joint =====
        p, _, _, _ = kinematic.JointKinematics(theta)

        # ===== LOG =====
        log_line = f"{elapsed:.4f}"

        # vị trí joint
        for i in range(6):
            log_line += f",{p[0,i]:.4f},{p[1,i]:.4f},{p[2,i]:.4f}"

        # vận tốc joint
        for i in range(6):
            log_line += f",{theta_dot[i,0]:.4f}"

        log_f.write(log_line + "\n")
        # ===== STOP =====
        dist = np.linalg.norm(x_curr - target)
        if dist < 0.01:
            total_time = elapsed
            rtde_c.speedJ([0]*6, acceleration=0.1, time=dt)
            print(f"\nReached target! x=[{x_curr[0]:.4f}, {x_curr[1]:.4f}, {x_curr[2]:.4f}] Time = {total_time:.2f} s")
            break

        # ===== SEND =====
        safe_speed = theta_dot.flatten().astype(float).tolist()
        rtde_c.speedJ(safe_speed, acceleration=0.1, time=dt)
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
        log_f.close()
    except:
        pass
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