from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
import numpy as np
import time
import os
import ki_urdf as kinematic

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C, WhiteKernel
from sklearn.preprocessing import StandardScaler
import warnings
warnings.filterwarnings("ignore")

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
folder = "logs_pro_2104"
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
        header += ",mu_pred,ai_pred,eta_pred,gpr_std"
        log_f.write(header + "\n")
        break
    i += 1

record_vars = ["timestamp", "actual_TCP_pose", "actual_TCP_speed"]
rtde_r.startFileRecording(filename, record_vars)

# ===== TARGET =====
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
sigma = 0.1
Ki = np.diag([1,1,1,1,1,3])
K  = np.eye(6)

# mu, ai, eta sẽ do GPR quyết định mỗi step
# Giữ lại giá trị gốc làm fallback an toàn
MU_DEFAULT  = 3.0
AI_DEFAULT  = 5.0
ETA_DEFAULT = 1.0

# ═══════════════════════════════════════════════════════════
# GPR MODULE
# ═══════════════════════════════════════════════════════════
class GPRParamPredictor:
    """
    Input  : [d_norm, R1_norm, cos_angle, obs_x_norm, obs_z_norm]
    Output : [mu, ai, eta]

    d_norm     – khoảng cách chuẩn hóa trong vùng [R1,R2] → [0,1]
    R1_norm    – bán kính vật cản / 0.1 (m)
    cos_angle  – góc tiếp cận robot→obs so với hướng đến target
    obs_x_norm – vị trí X tuyệt đối obstacle (chuẩn hóa workspace)
    obs_z_norm – vị trí Z tuyệt đối obstacle (chuẩn hóa workspace)
    """

    MU_BOUNDS  = (3.0, 10.0)
    AI_BOUNDS  = (10.0, 30.0)
    ETA_BOUNDS = (0.3,  2.0)
    MAX_SAMPLES = 80
    N_PRIOR     = 11   # số điểm prior, luôn giữ lại

    def __init__(self):
        self.scaler_X = StandardScaler()
        self.scaler_y = StandardScaler()

        # Prior data – domain knowledge từ tham số gốc (mu=3, ai=5, eta=1)
        # Feature: [d_norm, R1_norm, cos_angle, obs_x_norm, obs_z_norm]
        self.X_data = np.array([
            [0.05, 0.25,  0.9, -0.35, 0.23],   # rất gần, thẳng hướng
            [0.15, 0.25,  0.8, -0.35, 0.23],   # gần
            [0.30, 0.25,  0.7, -0.35, 0.23],   # trung bình-gần
            [0.50, 0.25,  0.5, -0.35, 0.23],   # trung bình (baseline gốc)
            [0.70, 0.25,  0.3, -0.35, 0.23],   # trung bình-xa
            [0.90, 0.25,  0.1, -0.35, 0.23],   # xa
            [0.05, 0.50,  0.9, -0.30, 0.20],   # obstacle lớn, rất gần
            [0.30, 0.50,  0.7, -0.30, 0.20],   # obstacle lớn, trung bình
            [0.70, 0.50,  0.3, -0.30, 0.20],   # obstacle lớn, xa
            [0.05, 0.25, -0.5, -0.40, 0.25],   # gần, góc lệch
            [0.50, 0.25, -0.8, -0.40, 0.25],   # trung bình, góc lệch
        ])

        # Target: [mu, ai, eta]
        self.y_data = np.array([
            [9.0, 28.0, 0.35],   # rất gần → mu/ai cao, eta thấp (thận trọng)
            [7.0, 20.0, 0.50],
            [5.0, 12.0, 0.70],
            [3.5,  8.0, 1.00],   # ← baseline gốc: mu=3, ai=5, eta=1
            [2.0,  5.0, 1.30],
            [1.0,  2.0, 1.70],   # xa → mu/ai thấp, eta cao (nhanh hơn)
            [9.5, 29.0, 0.30],
            [4.5, 10.0, 0.80],
            [1.5,  3.5, 1.50],
            [6.0, 14.0, 0.65],
            [1.2,  3.0, 1.60],
        ])

        # Weight: prior = 0.5, sample thực tế = performance score
        self.weights = np.ones(self.N_PRIOR) * 0.5

        self._fit()
        print(f"[GPR] Ready | {len(self.X_data)} prior samples")

    def _fit(self):
        kernel = (
            C(1.0, (0.1, 10.0))
            * RBF([0.3, 0.3, 0.5, 0.3, 0.3], (0.05, 5.0))
            + WhiteKernel(0.01, (1e-4, 0.5))
        )
        # alpha per-sample: weight thấp → noise lớn → ảnh hưởng ít
        alpha_vec = 1e-4 + (1.0 - self.weights) * 0.5

        self.gpr = GaussianProcessRegressor(
            kernel=kernel,
            alpha=alpha_vec,
            n_restarts_optimizer=5,
            normalize_y=True,
        )
        X_sc = self.scaler_X.fit_transform(self.X_data)
        y_sc = self.scaler_y.fit_transform(self.y_data)
        self.gpr.fit(X_sc, y_sc)

    def _feature(self, d_norm, R1, obs_vec_norm, obs_pos_abs):
        R1_norm  = R1 / 0.1
        move_dir = np.array([target[0], target[2]]) - obs_pos_abs
        n        = np.linalg.norm(move_dir)
        if n > 1e-6:
            move_dir /= n
        cos_angle  = float(np.dot(obs_vec_norm, move_dir))
        # Chuẩn hóa workspace: X∈[-0.6,0.0] → [-1,1], Z∈[0.1,0.4] → [-1,1]
        obs_x_norm = (obs_pos_abs[0] - (-0.3)) / 0.3
        obs_z_norm = (obs_pos_abs[1] -   0.25) / 0.15
        return np.array([[d_norm, R1_norm, cos_angle, obs_x_norm, obs_z_norm]])

    def predict(self, d_norm, R1, obs_vec_norm, obs_pos_abs):
        feat      = self._feature(d_norm, R1, obs_vec_norm, obs_pos_abs)
        feat_sc   = self.scaler_X.transform(feat)
        y_sc, std = self.gpr.predict(feat_sc, return_std=True)
        std_mean  = float(std.mean())

        # Uncertainty cao → fallback về giá trị an toàn (gần baseline gốc)
        if std_mean > 0.8:
            print(f"[GPR] High uncertainty ({std_mean:.2f}) → safe fallback")
            return MU_DEFAULT, AI_DEFAULT, ETA_DEFAULT, std_mean

        y = self.scaler_y.inverse_transform(y_sc)[0]
        mu_p  = float(np.clip(y[0], *self.MU_BOUNDS))
        ai_p  = float(np.clip(y[1], *self.AI_BOUNDS))
        eta_p = float(np.clip(y[2], *self.ETA_BOUNDS))
        return mu_p, ai_p, eta_p, std_mean

    def add_observation(self, d_norm, R1, obs_vec_norm, obs_pos_abs,
                        mu_used, ai_used, eta_used, performance_score):
        if performance_score < 0.6:
            print(f"[GPR] Sample rejected (score={performance_score:.2f})")
            return

        feat  = self._feature(d_norm, R1, obs_vec_norm, obs_pos_abs)
        new_y = np.array([[mu_used, ai_used, eta_used]])

        self.X_data  = np.vstack([self.X_data,  feat])
        self.y_data  = np.vstack([self.y_data,  new_y])
        self.weights = np.append(self.weights, performance_score)

        # Sliding window: xóa sample thực tế có weight thấp nhất
        if len(self.X_data) > self.MAX_SAMPLES:
            surplus      = len(self.X_data) - self.MAX_SAMPLES
            real_idx     = np.arange(self.N_PRIOR, len(self.X_data))
            drop_idx     = real_idx[np.argsort(self.weights[real_idx])[:surplus]]
            mask         = np.ones(len(self.X_data), dtype=bool)
            mask[drop_idx] = False
            self.X_data  = self.X_data[mask]
            self.y_data  = self.y_data[mask]
            self.weights = self.weights[mask]
            print(f"[GPR] Pruned {surplus} low-quality sample(s)")

        self._fit()
        print(f"[GPR] Updated | {len(self.X_data)} samples | "
              f"score={performance_score:.2f} | "
              f"mu={mu_used:.2f} ai={ai_used:.2f} eta={eta_used:.2f}")

    def save(self, path="gpr_data.npz"):
        np.savez(path, X=self.X_data, y=self.y_data, weights=self.weights)
        print(f"[GPR] Saved {len(self.X_data)} samples → {path}")

    def load(self, path="gpr_data.npz"):
        if os.path.exists(path):
            data         = np.load(path)
            self.X_data  = data["X"]
            self.y_data  = data["y"]
            self.weights = data["weights"]
            self._fit()
            print(f"[GPR] Loaded {len(self.X_data)} samples from {path}")
        else:
            print(f"[GPR] No saved data at '{path}', using prior only.")

# Khởi tạo và load dữ liệu cũ nếu có
gpr_predictor = GPRParamPredictor()
gpr_predictor.load("gpr_data.npz")

# ===== JOINT MAPPING =====
def jointMapping(q_v):
    exp_term = np.exp(-lamda*q_v)
    exp_term = np.clip(exp_term, 1e-9, 1e9)
    theta = (theta_max - theta_min)/(1+exp_term) + theta_min
    dg = lamda*(theta_max-theta_min)*exp_term/((1+exp_term)**2)
    dg = np.clip(dg, 1e-9, None)
    Jg = np.diag(dg)
    return theta, Jg

# ===== HIGH LEVEL CONTROLLER =====
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
    Jv = Jp[[0,2], :, 5]  # end-effector

    A_task = gamma * e @ Jv
    xdot_d = np.zeros((2,1))
    B = (-gamma * e @ xdot_d).item()

    A_obs_total = np.zeros((1, 6))

    # GPR output cho bước này (default = baseline gốc)
    mu_out  = MU_DEFAULT
    ai_out  = AI_DEFAULT
    eta_out = ETA_DEFAULT
    std_out = 0.0

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

                # ===== NORMALIZE DISTANCE =====
                d_norm = (dist - R1) / (R2 - R1)
                d_norm = np.clip(d_norm, 0, 1)

                # ===== SCALE-INVARIANT d_i =====
                d_i = 1 - d_norm

                # ===== NORMALIZE DIRECTION =====
                dist_vec_norm = dist_vec / (dist + 1e-6)

                # ===== GPR PREDICT tham số =====
                obs_pos_abs  = xobs_2d
                obs_vec_norm = dist_vec_norm[0]
                mu_out, ai_out, eta_out, std_out = gpr_predictor.predict(
                    float(d_norm), R1, obs_vec_norm, obs_pos_abs
                )

                # ===== STABLE SIGMOID (dùng mu từ GPR) =====
                expTerm = np.exp(-mu_out*(d_norm - 0.9))
                aii = ai_out * mu_out * expTerm / (1 + expTerm)**2

                # ===== OBSTACLE FORCE =====
                A_obs_i      = (aii * d_i * dist_vec_norm @ Jvi) @ Ki
                A_obs_total += A_obs_i
                B           += (aii * d_i * (dist_vec_norm @ xdot_obs)).item()

                # ===== DEBUG =====
                if dist < R1:
                    print("\n COLLISION!")
                elif dist < R2:
                    print(f"\n NEAR OBS | mu={mu_out:.2f} ai={ai_out:.2f} "
                          f"eta={eta_out:.2f} std={std_out:.2f}", end="")

    # ===== FINAL CONTROL (eta từ GPR) =====
    A = A_task - A_obs_total
    A = A.reshape(1, -1)

    if (A @ A.T) < 1e-6:
        Aplus = np.zeros((6,1))
    else:
        Aplus = A.T / (A @ A.T + 1e-3)

    rhs = -eta_out * A.T - Aplus * B

    _, Jg = jointMapping(q_virtual)

    if np.any(np.isnan(Jg)) or np.any(np.isinf(Jg)):
        print("Invalid Jg!")
        return np.zeros((6,1)), np.zeros((6,1)), mu_out, ai_out, eta_out, std_out

    qdot_virtual = K @ np.linalg.pinv(Jg) @ rhs

    # ===== GIỮ 2D =====
    qdot_virtual[0] = 0
    qdot_virtual[4] = 0
    qdot_virtual[5] = 0

    theta_dot = Jg @ qdot_virtual

    return qdot_virtual, theta_dot, mu_out, ai_out, eta_out, std_out

# ===== MAIN LOOP =====
perf_buffer    = []   # (d_norm, R1, obs_dir, obs_abs, mu, ai, eta)
min_dist_obs   = np.inf
reached_target = False

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
            qdot_virtual, theta_dot, mu_p, ai_p, eta_p, gpr_std = \
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

        # ===== Track khoảng cách nhỏ nhất tới obstacle =====
        for obs_pos in obs_list:
            d_tmp = np.linalg.norm(
                np.array([x_curr[0], x_curr[2]])
                - np.array([obs_pos[0], obs_pos[2]])
            )
            min_dist_obs = min(min_dist_obs, d_tmp)

        # ===== Thu thập perf sample mỗi 50 step =====
        if step_count % 50 == 0 and len(obs_list) > 0:
            obs_pos = obs_list[0]
            R1      = axes_list[0][0]
            R2      = 2.5 * R1
            xi      = np.array([x_curr[0], x_curr[2]])
            xo      = np.array([obs_pos[0], obs_pos[2]])
            dist_s  = np.linalg.norm(xi - xo)
            d_norm_s = float(np.clip((dist_s - R1) / (R2 - R1 + 1e-9), 0.0, 1.0))
            obs_dir  = (xi - xo) / (dist_s + 1e-6)
            perf_buffer.append((d_norm_s, R1, obs_dir, xo.copy(), mu_p, ai_p, eta_p))

        # ===== LOG =====
        log_line = f"{elapsed:.4f}"
        for i in range(6):
            log_line += f",{p[0,i]:.4f},{p[1,i]:.4f},{p[2,i]:.4f}"
        for i in range(6):
            log_line += f",{theta_dot[i,0]:.4f}"
        log_line += f",{mu_p:.4f},{ai_p:.4f},{eta_p:.4f},{gpr_std:.4f}"
        log_f.write(log_line + "\n")

        # ===== STOP =====
        dist = np.linalg.norm(x_curr - target)
        if dist < 0.01:
            reached_target = True
            rtde_c.speedJ([0]*6, acceleration=0.1, time=dt)
            print(f"\nReached target! x=[{x_curr[0]:.4f}, {x_curr[1]:.4f}, {x_curr[2]:.4f}] "
                  f"Time = {elapsed:.2f}s | min_dist_obs={min_dist_obs:.4f}m")
            break

        # ===== SEND =====
        safe_speed = theta_dot.flatten().astype(float).tolist()
        rtde_c.speedJ(safe_speed, acceleration=0.1, time=dt)

        # ===== PRINT =====
        if step_count % int(1.0/dt) == 0:
            print(f"\rv={theta_dot.flatten().round(3)} | "
                  f"dist={dist:.4f} | "
                  f"pos=({x_curr[0]:.3f},{x_curr[1]:.3f},{x_curr[2]:.3f}) | "
                  f"GPR[mu={mu_p:.1f} ai={ai_p:.1f} eta={eta_p:.2f} std={gpr_std:.2f}]",
                  end="", flush=True)

        rtde_c.waitPeriod(t_start)
        rtde_c.kickWatchdog()

except KeyboardInterrupt:
    print("Stopped")

finally:
    print("\nStopping robot safely...")

    # ===== GPR ONLINE UPDATE =====
    if reached_target and len(perf_buffer) > 0 and len(obs_list) > 0:
        R1_ref = axes_list[0][0]

        # Tính performance score từ khoảng cách nhỏ nhất tới obstacle
        if min_dist_obs < R1_ref:
            perf_score = 0.0    # va chạm
        elif min_dist_obs < 1.5 * R1_ref:
            perf_score = 0.5    # quá gần
        elif min_dist_obs < 3.0 * R1_ref:
            perf_score = 0.85   # tốt
        else:
            perf_score = 1.0    # hoàn hảo

        print(f"[GPR] Score={perf_score:.2f} | min_dist_obs={min_dist_obs:.4f}m")

        # Lấy 3 sample đại diện: 25%, 50%, 75% hành trình
        n = len(perf_buffer)
        for idx in [n//4, n//2, 3*n//4]:
            d_n, R1_, obs_d, obs_abs, mu_, ai_, eta_ = perf_buffer[idx]
            gpr_predictor.add_observation(
                d_n, R1_, obs_d, obs_abs,
                mu_, ai_, eta_, perf_score
            )

        gpr_predictor.save("gpr_data.npz")
    else:
        if not reached_target:
            print("[GPR] Target not reached — skipping update.")

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