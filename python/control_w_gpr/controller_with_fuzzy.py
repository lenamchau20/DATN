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
home_deg = [-179.40, -42.14, -132.36, -92.97, 89.72, 0]
home_q = np.deg2rad(home_deg).tolist()
print("Moving to home position...")
rtde_c.moveJ(home_q, 0.5, 1.0)
print("Reached home position.")
time.sleep(2)
print("Reached home. Start control!")

# ===== LOG =====
folder = "logs_fuzzy"
os.makedirs(folder, exist_ok=True)

base_name = "obs"
i = 1
while True:
    filename = os.path.join(folder, f"{base_name}_{i}.csv")
    joint_filename = filename.replace(".csv", "_joint.csv")
    if not os.path.exists(filename):
        log_f = open(joint_filename, "w")
        header = "time"
        for j in range(6):
            header += f",J{j+1}_x,J{j+1}_y,J{j+1}_z"
        for j in range(6):
            header += f",qdot{j+1}"
        header += ",mu_fuzzy,ai_fuzzy,eta_fuzzy,d_norm"
        log_f.write(header + "\n")
        break
    i += 1

record_vars = ["timestamp", "actual_TCP_pose", "actual_TCP_speed"]
rtde_r.startFileRecording(filename, record_vars)

# ===== TARGET & OBSTACLES =====
target = np.array([-0.475, 0.110, 0.300])

obs_list = [
    # np.array([-0.355, 0.110, 0.235])
    # np.array([-0.235, 0.110, 0.290])
    # np.array([-0.235, 0.110, 0.308]),
    np.array([-0.355, 0.110, 0.278])
         ]

axes_list = [
     np.array([0.025, 0.025, 0.025]),
    #  np.array([0.025, 0.025, 0.025])
]

# ===== PARAM =====
no_obs_printed = False
dt = 0.008

theta_min = np.array([-2*np.pi]*6)
theta_max = np.array([ 2*np.pi]*6)

# ===== CONTROL PARAM (fixed) =====
lamda = 1
gamma = 15
beta  = 0.9
sigma = 0.05
Ki = np.diag([1,1,1,1,1,3])
K  = np.eye(6)

# ═══════════════════════════════════════════════════════════
# FUZZY SCHEDULING MODULE
# ═══════════════════════════════════════════════════════════
# Ý tưởng:
#   d_norm ∈ [0,1] — 0 = sát vật cản, 1 = rìa vùng ảnh hưởng
#   Fuzzy chia 3 vùng membership: NEAR / MID / FAR
#   Mỗi vùng có rule → (mu, ai, eta)
#   Output = weighted average theo membership degree
#
# Không cần thư viện ngoài, toàn bộ dùng numpy thuần.
# ═══════════════════════════════════════════════════════════

# --- Membership function: hình thang (trapezoid) ---
def _trap(x, a, b, c, d):
    """
    Hình thang [a,b,c,d]:
        x < a  → 0
        a..b   → tăng từ 0 lên 1
        b..c   → = 1
        c..d   → giảm từ 1 về 0
        x > d  → 0
    """
    if x <= a or x >= d:
        return 0.0
    elif x < b:
        return (x - a) / (b - a + 1e-9)
    elif x <= c:
        return 1.0
    else:
        return (d - x) / (d - c + 1e-9)

# --- Membership functions cho d_norm ---
def membership(d_norm):
    """
    Ba vùng không chồng chéo hoàn toàn, có overlap mượt ở biên:
        NEAR : d_norm ∈ [0.0, 0.35]   trọng tâm 0.15
        MID  : d_norm ∈ [0.20, 0.75]  trọng tâm 0.50
        FAR  : d_norm ∈ [0.60, 1.00]  trọng tâm 0.85
    """
    near = _trap(d_norm, 0.00, 0.00, 0.20, 0.35)
    mid  = _trap(d_norm, 0.20, 0.35, 0.60, 0.75)
    far  = _trap(d_norm, 0.60, 0.75, 1.00, 1.00)
    return near, mid, far

# --- Fuzzy rules: singleton output cho mỗi vùng ---
#           mu    ai    eta
RULE_NEAR = (8.0, 10.0, 0.40)   # gần → mu/ai cao (đẩy mạnh), eta thấp (thận trọng)
RULE_MID  = (3.5,  5.0, 0.80)   # trung bình → giữ nguyên baseline gốc
RULE_FAR  = (1.0,  2.5, 1.50)   # xa → mu/ai thấp, eta cao (nhanh tới target)

def fuzzy_params(d_norm):
    """
    Trả về (mu, ai, eta) theo Fuzzy Weighted Average.
    Hoạt động thuần numpy, không cần thư viện.
    """
    near, mid, far = membership(d_norm)
    total = near + mid + far

    if total < 1e-9:
        # d_norm ngoài mọi vùng → fallback baseline
        return RULE_MID

    mu  = (near*RULE_NEAR[0] + mid*RULE_MID[0] + far*RULE_FAR[0]) / total
    ai  = (near*RULE_NEAR[1] + mid*RULE_MID[1] + far*RULE_FAR[1]) / total
    eta = (near*RULE_NEAR[2] + mid*RULE_MID[2] + far*RULE_FAR[2]) / total
    return mu, ai, eta

# ═══════════════════════════════════════════════════════════
# JOINT MAPPING 
# ═══════════════════════════════════════════════════════════
def jointMapping(q_v):
    exp_term = np.exp(-lamda*q_v)
    exp_term = np.clip(exp_term, 1e-9, 1e9)
    theta = (theta_max - theta_min)/(1+exp_term) + theta_min
    dg = lamda*(theta_max-theta_min)*exp_term/((1+exp_term)**2)
    dg = np.clip(dg, 1e-9, None)
    Jg = np.diag(dg)
    return theta, Jg

# ═══════════════════════════════════════════════════════════
# HIGH LEVEL CONTROLLER
# ═══════════════════════════════════════════════════════════
def highLevelController(x, theta):
    global no_obs_printed

    # ===== SAFE MAPPING =====
    theta_safe = np.clip(theta, theta_min + 1e-6, theta_max - 1e-6)
    ratio = (theta_max - theta_min) / (theta_safe - theta_min)
    q_virtual = -np.log(np.maximum(ratio - 1, 1e-9)) / lamda

    # ===== ERROR =====
    e = (np.array([x[0], x[2]]) - np.array([target[0], target[2]])).reshape(1, -1)

    # ===== JACOBIAN =====
    p, Jp, T06, _ = kinematic.JointKinematics(theta)
    Jv = Jp[[0,2], :, 5]

    A_task = gamma * e @ Jv
    xdot_d = np.zeros((2,1))
    B = (-gamma * e @ xdot_d).item()

    A_obs_total = np.zeros((1, 6))

    # output log (default = baseline)
    mu_log, ai_log, eta_log, dnorm_log = 3.5, 5.0, 0.8, 1.0

    # ===== OBSTACLE HANDLING =====
    if len(obs_list) == 0:
        if not no_obs_printed:
            print("No obstacles, moving straight to target.")
            no_obs_printed = True
    else:
        no_obs_printed = False

        for obs_pos, axes in zip(obs_list, axes_list):

            xobs_2d  = np.array([obs_pos[0], obs_pos[2]])
            xdot_obs = np.zeros((2,1))

            R1 = axes[0]
            R2 = 2.5 * R1

            for i in range(5,6):

                if i == 5:
                    xi  = np.array([x[0], x[2]])
                    Jvi = Jv
                else:
                    xi  = np.array([p[0,i], p[2,i]])
                    Jvi = Jp[[0,2], :, i]

                dist_vec = (xi - xobs_2d).reshape(1, -1)
                dist     = np.linalg.norm(dist_vec)

                # ===== [1.1] NORMALIZE d_i theo bán kính vật cản =====
                # d_norm ∈ [0,1]:  0 = sát R1,  1 = rìa R2
                # → mu/ai/eta không phụ thuộc đơn vị tuyệt đối của dist
                d_norm = (dist - R1) / (R2 - R1)
                d_norm = float(np.clip(d_norm, 0.0, 1.0))

                # scale-invariant d_i
                d_i = 1.0 - d_norm

                # normalize hướng
                dist_vec_n = dist_vec / (dist + 1e-6)

                # ===== [2.2] FUZZY SCHEDULING =====
                # mu, ai, eta tự scale theo d_norm
                # không cần chỉnh tay khi vật cản thay đổi vị trí/kích thước
                mu_f, ai_f, eta_f = fuzzy_params(d_norm)

                # ghi log
                mu_log, ai_log, eta_log, dnorm_log = mu_f, ai_f, eta_f, d_norm

                # ===== STABLE SIGMOID =====
                expTerm = np.exp(-mu_f*(d_norm - beta))
                aii = ai_f * mu_f * expTerm / (1 + expTerm)**2

                # ===== OBSTACLE FORCE =====
                A_obs_i      = (aii * d_i * dist_vec_n @ Jvi) @ Ki
                A_obs_total += A_obs_i
                B           += (aii * d_i * (dist_vec_n @ xdot_obs)).item()

                # ===== DEBUG =====
                if dist < R1:
                    print("\n COLLISION!")
                elif dist < R2:
                    near, mid, far = membership(d_norm)
                    print(f"\n NEAR OBS | d_norm={d_norm:.2f} "
                          f"[N={near:.2f} M={mid:.2f} F={far:.2f}] "
                          f"→ mu={mu_f:.2f} ai={ai_f:.2f} eta={eta_f:.2f}",
                          end="")

    # ===== FINAL CONTROL =====
    A = A_task - A_obs_total
    A = A.reshape(1, -1)

    if (A @ A.T) < 1e-6:
        Aplus = np.zeros((6,1))
    else:
        Aplus = A.T / (A @ A.T + 1e-3)

    # eta từ Fuzzy
    rhs = -eta_log * A.T - Aplus * B

    _, Jg = jointMapping(q_virtual)

    if np.any(np.isnan(Jg)) or np.any(np.isinf(Jg)):
        print("Invalid Jg!")
        return np.zeros((6,1)), np.zeros((6,1)), mu_log, ai_log, eta_log, dnorm_log

    qdot_virtual = K @ np.linalg.pinv(Jg) @ rhs

    # ===== GIỮ 2D =====
    qdot_virtual[0] = 0
    qdot_virtual[4] = 0
    qdot_virtual[5] = 0

    theta_dot = Jg @ qdot_virtual

    return qdot_virtual, theta_dot, mu_log, ai_log, eta_log, dnorm_log

# ===== MAIN LOOP =====
try:
    step_count = 0
    rtde_c.setWatchdog(0.2)

    while True:
        t_start = rtde_c.initPeriod()

        step_count += 1
        elapsed = step_count * dt

        tcp    = rtde_r.getActualTCPPose()
        x_curr = np.array(tcp[:3])
        theta  = np.array(rtde_r.getActualQ())

        # ===== CONTROLLER =====
        try:
            qdot_virtual, theta_dot, mu_p, ai_p, eta_p, dn = \
                highLevelController(x_curr, theta)
        except Exception as e:
            print("\nController ERROR:", e)
            break

        # check NaN
        if np.any(np.isnan(theta_dot)) or np.any(np.isinf(theta_dot)):
            print("\nNaN detected → STOP")
            break

        vmax = 0.3
        theta_dot = vmax * np.tanh(theta_dot / vmax)

        # ===== FK để lấy vị trí joint =====
        p, _, _, _ = kinematic.JointKinematics(theta)

        # ===== LOG =====
        log_line = f"{elapsed:.4f}"
        for i in range(6):
            log_line += f",{p[0,i]:.4f},{p[1,i]:.4f},{p[2,i]:.4f}"
        for i in range(6):
            log_line += f",{theta_dot[i,0]:.4f}"
        log_line += f",{mu_p:.4f},{ai_p:.4f},{eta_p:.4f},{dn:.4f}"
        log_f.write(log_line + "\n")

        # ===== STOP =====
        dist = np.linalg.norm(x_curr - target)
        if dist < 0.01:
            rtde_c.speedJ([0]*6, acceleration=0.1, time=dt)
            print(f"\nReached target! x=[{x_curr[0]:.4f}, {x_curr[1]:.4f}, {x_curr[2]:.4f}] "
                  f"Time = {elapsed:.2f} s")
            break

        # ===== SEND =====
        safe_speed = theta_dot.flatten().astype(float).tolist()
        rtde_c.speedJ(safe_speed, acceleration=0.1, time=dt)

        # ===== PRINT =====
        if step_count % int(1.0/dt) == 0:
            print(f"\rv={theta_dot.flatten().round(3)} | "
                  f"dist={dist:.4f} | "
                  f"pos=({x_curr[0]:.3f},{x_curr[1]:.3f},{x_curr[2]:.3f}) | "
                  f"Fuzzy[d={dn:.2f} mu={mu_p:.1f} ai={ai_p:.1f} eta={eta_p:.2f}]",
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