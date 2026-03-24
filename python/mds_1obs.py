from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
import numpy as np
import time
import os

IP = "192.168.1.102"

rtde_r = RTDEReceiveInterface(IP)
rtde_c = RTDEControlInterface(IP)

# --- THIẾT LẬP GHI DỮ LIỆU ---
# Chọn các biến cần thiết để vẽ quỹ đạo
folder = "logs"
os.makedirs(folder, exist_ok=True)

base_name = "mds_1obs"
i = 1

while True:
    filename = os.path.join(folder, f"{base_name}_{i}.csv")
    if not os.path.exists(filename):
        break
    i += 1

record_vars = ["timestamp", "actual_TCP_pose"]
rtde_r.startFileRecording(filename, record_vars)
print(f"Running MDS - Bắt đầu ghi dữ liệu")


#initialposition = [-250; 110; 200]
# ===== TARGET =====
target = np.array([-0.475, 0.110, 0.200])

# ===== OBSTACLE (ELLIPSOID) =====
obs = np.array([-0.315, 0.108, 0.200])  #vị trí vật cản
axes = np.array([0.025, 0.025, 0.025])   # bán trục ellipsoid

# ===== PARAM =====
k = 0.6
dt = 0.008 #125hz
v_max = 0.15
#rho = 1


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
def get_modulation_matrix(x, obs, axes):

    g = gamma(x, obs, axes)

    g = max(g, 1.0001)   # tránh singularity

    # ===== NORMAL =====
    n = gradient_gamma(x, obs, axes)
    norm_n = np.linalg.norm(n)

    if norm_n < 1e-6:
        return np.eye(3)

    n = n / norm_n

    # ===== BASIS =====
    I = np.eye(3)
    P = I - np.outer(n, n)

    t1 = P @ np.array([1, 0, 0])

    # nếu vector này quá nhỏ -> đổi trục
    if np.linalg.norm(t1) < 1e-6:
        t1 = P @ np.array([0, 1, 0])

    t1 = t1 / np.linalg.norm(t1)

    t2 = np.cross(n, t1)
    t2 = t2 / np.linalg.norm(t2)

    E = np.column_stack((n, t1, t2))

    # ===== EIGENVALUES =====
    lambda1 = 1 - 1/(g)
    lambda2 = 1 + 1/(g)
    lambda3 = lambda2

    D = np.diag([lambda1, lambda2, lambda3])

    M = E @ D @ E.T
    return M

# ===== MAIN LOOP =====
try:

    step_count = 0

    while True:
        step_count += 1
        elapsed = step_count * dt
        t_start = rtde_c.initPeriod()

        tcp = rtde_r.getActualTCPPose()
        x_curr = np.array(tcp[:3])

        # ===== NOMINAL DS =====
        v_nominal = -k * (x_curr - target)

        # ===== MDS =====
        M = get_modulation_matrix(x_curr, obs, axes)
        v_safe = M @ v_nominal

        # ===== LIMIT SPEED =====
        norm_v = np.linalg.norm(v_safe)
        if norm_v > v_max:
            v_safe = v_safe / norm_v * v_max

        # ===== STOP CONDITION =====
        dist = np.linalg.norm(x_curr - target)
        if dist < 0.005:
            total_time = elapsed
            rtde_c.speedL([0,0,0,0,0,0], 0.3, dt)
            print(f"\nReached target! Time = {total_time:.2f} s")
            break                                        

        # ===== SEND COMMAND =====
        cmd_vel = [v_safe[0], v_safe[1], v_safe[2], 0, 0, 0]
        rtde_c.speedL(cmd_vel, acceleration=0.3, time=dt)

        # In thông tin để theo dõi
        if int(time.time()*10) % 5 == 0:   # ~0.5s in 1 lần
            print(f"\rx={x_curr.round(3)} | v={v_safe.round(3)} | dist={dist:.4f} | t={elapsed:.2f}s", end="")

        rtde_c.waitPeriod(t_start)

except KeyboardInterrupt:
    print("Stopped")

finally:
    rtde_r.stopFileRecording()
    rtde_c.speedStop()
    rtde_c.stopScript()