from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
import numpy as np
import time
import os

IP = "192.168.1.102"

rtde_r = RTDEReceiveInterface(IP)
rtde_c = RTDEControlInterface(IP)

# === THIẾT LẬP GHI DỮ LIỆU ===
folder = "logs2403"
os.makedirs(folder, exist_ok=True)

base_name = "mds_test"
i = 1

while True:
    filename = os.path.join(folder, f"{base_name}_{i}.csv")
    if not os.path.exists(filename):
        break
    i += 1

record_vars = ["timestamp", "actual_TCP_pose"]
rtde_r.startFileRecording(filename, record_vars)
print(f"Running - Ghi dữ liệu")

#Initial_TCP_Pose = [-200; 110; 200; 1.35; 2.82; 0]

# ===== TARGET =====
target = np.array([-0.475, 0.110, 0.200])

# ===== OBSTACLE (ELLIPSOID) =====
obs_list = [
    #np.array([-0.305, 0.105, 0.200]),
    #np.array([-0.385, 0.112, 0.200]),
    np.array([-0.35, 0.110, 0.200])
]

axes_list = [
    #np.array([0.025, 0.025, 0.025]), #0.025
    np.array([0.02, 0.02, 0.02])
]

rho_list = [
    #1,
    1
]

# ===== PARAM =====
#techpendant velocity bar: 50% 
kc = 0.5   
dt = 0.008 #125hz
v_max = 0.15

# ===== GAMMA FUNCTION =====
def gamma(x, obs, axes):
    dx = x - obs
    return np.sqrt(
        (dx[0]/axes[0])**2 +
        (dx[1]/axes[1])**2 +
        (dx[2]/axes[2])**2
    )
# ==== GRADIENT GAMMA ======
def gradient_gamma(x, obs, axes):
    dx = x - obs
    g = gamma(x, obs, axes)

    if g < 1e-6:
        return np.zeros(3)

    return np.array([
        dx[0] / (axes[0]**2 * g),
        dx[1] / (axes[1]**2 * g),
        dx[2] / (axes[2]**2 * g)
    ])

# ===== MODULATION MATRIX =====
def modulatedDS(x, obs_list, axes_list, rho_list):

    kObst = len(obs_list)

    if kObst == 0:
        return np.eye(3)   # ko obstacle -> ko modulate: thoát hàm modulatedDS
    
    gamma_vals = np.zeros(kObst)
    grad_gamma = np.zeros((3, kObst))

    # ===== COMPUTE GAMMA + GRAD =====
    for k in range(kObst):
        gamma_vals[k] = gamma(x, obs_list[k], axes_list[k])
        grad_gamma[:, k] = gradient_gamma(x, obs_list[k], axes_list[k])

    # ===== g - 1 =====
    eps_d = 1e-9
    gminus1 = np.maximum(gamma_vals - 1, eps_d)

    M_total = np.eye(3)

    # ===== LOOP OBSTACLES =====
    for k in range(kObst):

        # ===== WEIGHT FUNCTION =====
        if kObst > 1:
            gk = gminus1[k]
            gi = gminus1.copy()

            frac = gi / (gk + gi)
            frac[k] = 1.0

            weight = np.prod(frac)
        else:
            weight = 1.0

        # ===== NORMAL =====
        n = grad_gamma[:, k]
        norm_n = np.linalg.norm(n)

        if norm_n < 1e-6:
            continue

        n = n / norm_n

        # ===== BASIS (GIỐNG MATLAB) =====
        I = np.eye(3)
        P = I - np.outer(n, n)

        t1 = P @ np.array([1, 0, 0])
        if np.linalg.norm(t1) < 1e-6:
            t1 = P @ np.array([0, 1, 0])

        t1 = t1 / np.linalg.norm(t1)

        t2 = np.cross(n, t1)
        t2 = t2 / np.linalg.norm(t2)

        E = np.column_stack((n, t1, t2))

        # ===== EIGENVALUES (CHUẨN MATLAB) =====
        g = max(gamma_vals[k], 1.0001)

        lambda1 = 1 - weight / (g ** (1 / rho_list[k]))
        lambda2 = 1 + weight / (g ** (1 / rho_list[k]))
        lambda3 = lambda2

        D = np.diag([lambda1, lambda2, lambda3])

        Mk = E @ D @ E.T

        # ===== MULTIPLY =====
        M_total = Mk @ M_total

    return M_total

def check_collision(x, obs_list, axes_list, threshold=1.0):
    for i in range(len(obs_list)):
        g = gamma(x, obs_list[i], axes_list[i])

        if g <= threshold:
            return True

    return False
# ===== MAIN LOOP =====
try:
    step_count = 0
    rtde_c.setWatchdog(0.1)

    while True:
        loop_start = time.time()

        t_start = rtde_c.initPeriod()

        step_count += 1
        elapsed = step_count * dt

        # ===== Lấy TCP =====
        try:
            tcp = rtde_r.getActualTCPPose()
            x_curr = np.array(tcp[:3])

        except Exception as e:
            print(f"\nMất kết nối robot! Lỗi: {e}")

            try:
                rtde_c.speedStop()   # dừng robot ngay
            except:
                pass

            break   # thoát while
        
        # ===== COLLISION CHECK =====
        if check_collision(x_curr, obs_list, axes_list, threshold=1.0):
            print("\n COLLISION WARNING!")

            try:
                rtde_c.speedStop()
            except:
                pass

            break
        # ===== NOMINAL DS =====
        v_nominal = -kc * (x_curr - target)

        # ===== MDS =====
        M_total = modulatedDS(x_curr, obs_list, axes_list, rho_list)
        v_safe = M_total @ v_nominal

        # ===== LIMIT SPEED =====
        norm_v = np.linalg.norm(v_safe)
        if norm_v > v_max:
            v_safe = v_safe / norm_v * v_max

        # ===== STOP CONDITION =====
        dist = np.linalg.norm(x_curr - target)
        if dist < 0.005:
            total_time = elapsed
            rtde_c.speedL([0,0,0,0,0,0], 0.2, dt)
            print(f"\nReached target! x=[{x_curr[0]:.4f}, {x_curr[1]:.4f}, {x_curr[2]:.4f}] Time = {total_time:.2f} s")
            break

        # ===== SEND COMMAND =====
        cmd_vel = [v_safe[0], v_safe[1], v_safe[2], 0, 0, 0]
        rtde_c.speedL(cmd_vel, acceleration=0.2, time=dt)

        # Print
        if step_count % int(1.0/dt) == 0:   # 1 giây in 1 lần
            print(f"\rv={v_safe.round(3)} | dist={dist:.4f}", end="")

        # tcp_speed = rtde_r.getActualTCPSpeed()
        # v = np.array(tcp_speed[:3])   # chỉ lấy linear

        # speed = np.linalg.norm(v)

        # print(f"\nv_tcp = {speed:.3f} m/s", end="")

        rtde_c.waitPeriod(t_start)
        rtde_c.kickWatchdog()
        loop_time = time.time() - loop_start

        if loop_time > 0.05:
            print(f"\n LOOP SLOW: {loop_time:.4f}s")

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