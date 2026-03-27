from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
import numpy as np
import time
import os

IP = "192.168.1.102"

rtde_r = RTDEReceiveInterface(IP)
rtde_c = RTDEControlInterface(IP)

# === THIẾT LẬP GHI DỮ LIỆU ===
folder = "logs2703"
os.makedirs(folder, exist_ok=True)

base_name = "1obs_ree_0.05"
i = 1

while True:
    filename = os.path.join(folder, f"{base_name}_{i}.csv")
    if not os.path.exists(filename):
        break
    i += 1

record_vars = ["timestamp", "actual_TCP_pose", "actual_TCP_speed"]
rtde_r.startFileRecording(filename, record_vars)
print(f"Running")

#Initial_TCP_Pose = [-200; 110; 200; 1.35; 2.82; 0] velocity_bar=100%

# ===== TARGET =====
target = np.array([-0.475, 0.110, 0.200])

# ===== OBSTACLE (ELLIPSOID) =====
obs_list = [
    #np.array([-0.305, 0.108, 0.200]),
    #np.array([-0.385, 0.112, 0.200])
    np.array([-0.35, 0.112, 0.198])
]

axes_list = [
    #np.array([0.025, 0.025, 0.025]),
    np.array([0.02, 0.02, 0.02])
]


rho_list = [
    #1,
    0.01
]

ree = 0.05
# ===== PARAM =====
#techpendant velocity bar: 50% 
kc = 0.5   
dt = 0.008 #125hz
v_max = 0.15

def to2d_xz(p3):
    return np.array([p3[0], p3[2]])

def mu2d(x2d, obs2d, axes2d):
    dx = x2d - obs2d

    val = np.sqrt(
        (dx[0]/axes2d[0])**2 +
        (dx[1]/axes2d[1])**2 
    )

    if val < 1e-6:
        return 1.0  # tránh chia 0

    return 1.0 / val  #mu=1/sqrt
def gamma_distance2d(x2d, obs2d, axes2d):
    dx = x2d - obs2d
    d = np.linalg.norm(dx)

    mu_val = mu2d(x2d, obs2d, axes2d)

    return 1 + max(d * (1 - mu_val), 0)

# ==== GRADIENT GAMMA ======
def gradient_gamma2d(x2d, obs2d, axes2d):
    dx = x2d - obs2d

    g = np.sqrt(
        (dx[0]/axes2d[0])**2 +
        (dx[1]/axes2d[1])**2 
        
    )  #tính lấy gamma 

    if g < 1e-6:
        return np.zeros(2)

    return np.array([
        dx[0] / (axes2d[0]**2 * g),
        dx[1] / (axes2d[1]**2 * g)
    ]) 
# ===== MODULATION MATRIX =====
def modulatedDS2d(x2d, obs_list, axes_list, rho_list, ree):

    kObst = len(obs_list)

    if kObst == 0:
        return np.eye(2)   # ko obstacle -> ko modulate: thoát hàm modulatedDS
    
    gamma_eff = np.ones(kObst)
    grad_gamma = np.zeros((2, kObst))

    # ===== COMPUTE GAMMA + GRAD =====
    for k in range(kObst):
        obs2d = to2d_xz(obs_list[k]) 
        axes2d = np.array([axes_list[k][0], axes_list[k][2]])

        g = gamma_distance2d(x2d, obs2d, axes2d)
        grad_gamma[:, k] = gradient_gamma2d(x2d, obs2d, axes2d)

        # ===== APPLY REE (margin) =====
        if g > 1 + ree:
            gamma_eff[k] = g - ree
        else:
            gamma_eff[k] = 1.0

    # ===== g - 1 =====
    eps_d = 1e-9
    #gminus1 = np.maximum(gamma_vals - 1, eps_d)
    gminus1 = np.maximum(gamma_eff - 1, eps_d)
    M_total = np.eye(2)

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

        n = n / norm_n #(nx, nz)

        # Basis 2D
        t = np.array([-n[1], n[0]]) #(-nz, nx)
        E = np.column_stack((n, t))

        g_eff = max(gamma_eff[k], 1.0001)

        lam1 = 1 - weight / (g_eff ** (1 / rho_list[k]))
        lam2 = 1 + weight / (g_eff ** (1 / rho_list[k]))

        D = np.diag([lam1, lam2])
        Mk = E @ D @ E.T

        M_total = Mk @ M_total

    return M_total

def check_collision_2d(x2d, obs_list, axes_list):
    for k in range(len(obs_list)):
        obs2d  = to2d_xz(obs_list[k])
        axes2d = np.array([axes_list[k][0], axes_list[k][2]])

        if gamma_distance2d(x2d, obs2d, axes2d) <= 1.0:
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
        tcp = rtde_r.getActualTCPPose()
        x_curr = np.array(tcp[:3])

        # ===== CHUYỂN SANG 2D =====
        x2d = to2d_xz(x_curr)
        target2d = to2d_xz(target)

         # ===== COLLISION CHECK =====
        if check_collision_2d(x2d, obs_list, axes_list):
            print("\n COLLISION WARNING!")
            rtde_c.speedStop()
            break
        # ===== NOMINAL 2D DS =====
        v_nominal_2d = -kc * (x2d - target2d)

        # ===== MDS 2D =====
        M2d = modulatedDS2d(x2d, obs_list, axes_list, rho_list, ree)
        v_safe_2d = M2d @ v_nominal_2d

         # ===== LIMIT SPEED =====
        norm_v = np.linalg.norm(v_safe_2d)
        if norm_v > v_max:
            v_safe_2d = v_safe_2d / norm_v * v_max
        # ===== MAP LẠI 3D =====
        #v_y_nominal = -kc * (x_curr[1] - target[1])
        v_safe = np.array([v_safe_2d[0], 0.0, v_safe_2d[1]])

        # ===== STOP CONDITION =====
        dist = np.linalg.norm(x2d - target2d)
        if dist < 0.01:
            total_time = elapsed
            rtde_c.speedL([0,0,0,0,0,0], 0.2, dt)
            print(f"\nReached target! x=[{x_curr[0]:.4f}, {x_curr[1]:.4f}, {x_curr[2]:.4f}] Time = {total_time:.2f} s")
            break

        # ===== SEND COMMAND =====
        cmd_vel = [v_safe[0], v_safe[1], v_safe[2], 0, 0, 0]
        rtde_c.speedL(cmd_vel, acceleration=1.0, time=dt)

        # Print
        if step_count % int(1.0/dt) == 0:   # 1 giây in 1 lần
            print(f"\rv={v_safe.round(3)} | dist={dist:.4f}", end="")

        rtde_c.waitPeriod(t_start)
        rtde_c.kickWatchdog()
        loop_time = time.time() - loop_start

        if loop_time > 0.1:
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