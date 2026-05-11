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

# ===== HOME POSITION =====
home_deg = [-179.40, -42.14, -132.36, -92.97, 89.72, 0] 
home_q = np.deg2rad(home_deg).tolist()
print("Moving to home position...")
rtde_c.moveJ(home_q, 0.5, 1.0)
print("Reached home position.")
time.sleep(2)
print("Reached home. Start control!")

# ===== LOG SETUP =====
folder = "logs_pro_2204"
os.makedirs(folder, exist_ok=True)

base_name = "1obs_adaptive"
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
        # Thêm log cho adaptive parameters
        header += ",eta_adaptive,R1_adaptive,R2_adaptive,mu_adaptive,ai_adaptive"
        log_f.write(header + "\n")
        break
    i += 1

record_vars = ["timestamp", "actual_TCP_pose", "actual_TCP_speed"]
rtde_r.startFileRecording(filename, record_vars)

# ===== TARGET & OBSTACLE =====
target = np.array([-0.475, 0.110, 0.300])

obs_list = [
    # np.array([-0.355, 0.110, 0.235])
    # np.array([-0.235, 0.110, 0.290])
    np.array([-0.235, 0.110, 0.308]),
    np.array([-0.355, 0.110, 0.278])
         ]

axes_list = [
     np.array([0.025, 0.025, 0.025]),
     np.array([0.025, 0.025, 0.025])
]

# ===== FIXED PARAM (KHÔNG THAY ĐỔI) =====
no_obs_printed = False
dt = 0.008

theta_min = np.array([-2*np.pi]*6)
theta_max = np.array([ 2*np.pi]*6)

# ===== BASE CONTROL PARAM (CỐ ĐỊNH - KHÔNG CẦN THAY ĐỔI) =====
lamda = 1
gamma = 15
eta_base   = 2.0    # ← Base value, sẽ tự động scale
mu_base    = 5.0    # ← Base value, sẽ tự động scale
ai_base    = 5.0   # ← Base value, sẽ tự động scale
beta  = 0.9
sigma = 0.15 #
Ki = np.diag([1,1,1,1,1,3])
K = np.eye(6)

# ===== ADAPTIVE PARAMETER CONFIG =====
class AdaptiveConfig:
    """Cấu hình cho adaptive parameters"""
    # Tỷ lệ vùng ảnh hưởng / vùng nguy hiểm (cố định)
    influence_ratio = 1.5
    
    # Hệ số cho adaptive scaling (có thể tinh chỉnh)
    proximity_strength = 5.0      # Ảnh hưởng của khoảng cách
    goal_priority_strength = 10.0 # Ưu tiên goal khi gần
    size_sensitivity = 1.0        # Độ nhạy với kích thước obstacle
    
    @staticmethod
    def compute_reference_scale(start_pos, goal_pos, obs_list, axes_list):
        """Tính thang đo tham chiếu của workspace"""
        # Khoảng cách start-goal (chỉ dùng XZ)
        start_2d = np.array([start_pos[0], start_pos[2]])
        goal_2d = np.array([goal_pos[0], goal_pos[2]])
        workspace_scale = np.linalg.norm(goal_2d - start_2d)
        
        # Xét kích thước obstacle lớn nhất
        if obs_list:
            max_obs_size = max([max(axes) for axes in axes_list])
            workspace_scale = max(workspace_scale, 10 * max_obs_size)
        
        return max(workspace_scale, 0.05)  # Minimum 5cm
    
    @staticmethod
    def normalize_distance(distance, reference_scale):
        """Chuẩn hóa khoảng cách"""
        return distance / max(reference_scale, 0.001)
    
    @staticmethod
    def compute_adaptive_eta(distance_to_obs, obs_radius, distance_to_goal, 
                            reference_scale, eta_base):
        """
        η tự động điều chỉnh:
        - Gần obstacle → tăng
        - Gần goal → giảm (ưu tiên đến đích)
        - Obstacle lớn → tăng
        """
        norm_dist_obs = AdaptiveConfig.normalize_distance(distance_to_obs, reference_scale)
        norm_dist_goal = AdaptiveConfig.normalize_distance(distance_to_goal, reference_scale)
        
        # Gần obstacle → lực mạnh hơn
        proximity_factor = np.exp(-AdaptiveConfig.proximity_strength * norm_dist_obs)
        
        # Gần goal → giảm repulsion
        goal_priority_factor = 1.0 / (1.0 + np.exp(-AdaptiveConfig.goal_priority_strength * 
                                                    (norm_dist_goal - 0.1)))
        
        # Obstacle lớn → lực mạnh hơn
        size_factor = 1.0 + AdaptiveConfig.size_sensitivity * obs_radius / reference_scale
        
        eta_adaptive = eta_base * proximity_factor * goal_priority_factor * size_factor
        
        return max(eta_adaptive, 0.1)
    
    @staticmethod
    def compute_adaptive_radii(obs_pos, goal_pos, obs_radius, reference_scale):
        """
        R1 (nguy hiểm) và R2 (ảnh hưởng) tự động điều chỉnh:
        - Obstacle gần goal → vùng an toàn rộng hơn
        """
        obs_2d = np.array([obs_pos[0], obs_pos[2]])
        goal_2d = np.array([goal_pos[0], goal_pos[2]])
        
        dist_obs_to_goal = np.linalg.norm(goal_2d - obs_2d)
        norm_dist = AdaptiveConfig.normalize_distance(dist_obs_to_goal, reference_scale)
        
        # Obstacle gần goal → vùng an toàn lớn hơn
        goal_proximity_scale = 1.0 + np.exp(-4.0 * norm_dist)
        
        R1 = obs_radius * goal_proximity_scale
        R2 = R1 * AdaptiveConfig.influence_ratio
        
        return R1, R2
    
    @staticmethod
    def compute_adaptive_mu(distance_to_obs, distance_to_goal, reference_scale, mu_base):
        """
        μ tự động điều chỉnh:
        - Gần obstacle → tăng độ nhạy
        """
        norm_dist_obs = AdaptiveConfig.normalize_distance(distance_to_obs, reference_scale)
        
        # Gần obstacle → mu tăng (hàm mượt hơn)
        sensitivity_factor = 1.0 + 2.0 * np.exp(-3.0 * norm_dist_obs)
        
        mu_adaptive = mu_base * sensitivity_factor
        
        return mu_adaptive
    
    @staticmethod
    def compute_adaptive_ai(distance_to_obs, obs_radius, reference_scale, ai_base):
        """
        ai tự động điều chỉnh:
        - Gần obstacle → tăng gain
        """
        norm_dist_obs = AdaptiveConfig.normalize_distance(distance_to_obs, reference_scale)
        
        # Gần obstacle → ai tăng
        gain_factor = 1.0 + 3.0 * np.exp(-4.0 * norm_dist_obs)
        
        ai_adaptive = ai_base * gain_factor
        
        return ai_adaptive


def to2d_xz(p3):
    return np.array([p3[0], p3[2]])


def jointMapping(q_v):
    exp_term = np.exp(-lamda*q_v)
    exp_term = np.clip(exp_term, 1e-9, 1e9)

    theta = (theta_max - theta_min)/(1+exp_term) + theta_min
    dg = lamda*(theta_max-theta_min)*exp_term/((1+exp_term)**2)
    dg = np.clip(dg, 1e-9, None)

    Jg = np.diag(dg)
    return theta, Jg


def highLevelController(x, theta, target, obs_list, axes_list, start_pos, reference_scale):
    """
    Controller với ADAPTIVE PARAMETERS
    
    Args:
        x: Current end-effector position
        theta: Current joint angles
        target: Target position
        obs_list: List of obstacle positions
        axes_list: List of obstacle radii (ellipsoid axes)
        start_pos: Starting position (for reference scale)
        reference_scale: Workspace scale (computed once)
    """
    global no_obs_printed
    
    # ===== SAFE MAPPING =====
    theta_safe = np.clip(theta, theta_min + 1e-6, theta_max - 1e-6)
    ratio = (theta_max - theta_min) / (theta_safe - theta_min)
    q_virtual = -np.log(np.maximum(ratio - 1, 1e-9)) / lamda

    # ===== ERROR (2D: XZ plane) =====
    x_2d = np.array([x[0], x[2]])
    target_2d = np.array([target[0], target[2]])
    e = (x_2d - target_2d).reshape(1, -1)
    
    distance_to_goal = np.linalg.norm(e)

    # ===== JACOBIAN =====
    p, Jp, T06, _ = kinematic.JointKinematics(theta)
    Jv = Jp[[0,2], :, 5]  # end-effector (XZ plane)
    # ===== Variables for logging adaptive params =====
    eta_adaptive_log = eta_base
    R1_adaptive_log = 0.0
    R2_adaptive_log = 0.0
    mu_adaptive_log = mu_base
    ai_adaptive_log = ai_base
    gamma_used = gamma
    if len(obs_list) == 0:
        gamma_used = 5.0  # Giảm gain khi không có obstacle để tránh overshoot
        eta_adaptive_log = 1.0  # Sử dụng eta cơ bản khi không có obstacle
    A_task = gamma_used * e @ Jv
    xdot_d = np.zeros((2,1))
    # xdot_d = -0.5 * e.T
    B = (-gamma_used * e @ xdot_d).item()
    A_obs_total = np.zeros((1, 6))
    


    # ===== OBSTACLE HANDLING WITH ADAPTIVE PARAMETERS =====
    if len(obs_list) == 0:
        if not no_obs_printed:
            print("No obstacles, moving straight to target.")
            no_obs_printed = True
    else:
        no_obs_printed = False

        for obs_pos, axes in zip(obs_list, axes_list):
            xobs_2d = np.array([obs_pos[0], obs_pos[2]])
            xdot_obs = np.zeros((2,1))
            
            # Lấy bán kính obstacle (trung bình các axes)
            obs_radius = np.mean(axes)
            
            # ===== ADAPTIVE R1, R2 =====
            R1, R2 = AdaptiveConfig.compute_adaptive_radii(
                obs_pos, target, obs_radius, reference_scale
            )
            
            # Log adaptive params
            R1_adaptive_log = R1
            R2_adaptive_log = R2
            
            alpha = np.exp(-(R2**2 - R1**2)) / beta
            
            # Xử lý cho từng link (ở đây chỉ end-effector)
            for i in range(5, 6):
                if i == 5:
                    xi = x_2d
                    Jvi = Jv
                else:
                    xi = np.array([p[0,i], p[2,i]])
                    Jvi = Jp[[0,2], :, i]

                dist_vec = (xi - xobs_2d).reshape(1, -1)
                d_iobs = (dist_vec @ dist_vec.T).item()
                dist = np.sqrt(d_iobs)
                
                # ===== ADAPTIVE η (ETA) =====
                eta_adaptive = AdaptiveConfig.compute_adaptive_eta(
                    dist, obs_radius, distance_to_goal, reference_scale, eta_base
                )
                eta_adaptive_log = eta_adaptive
                
                # ===== ADAPTIVE μ (MU) =====
                mu_adaptive = AdaptiveConfig.compute_adaptive_mu(
                    dist, distance_to_goal, reference_scale, mu_base
                )
                mu_adaptive_log = mu_adaptive
                
                # ===== ADAPTIVE ai =====
                ai_adaptive = AdaptiveConfig.compute_adaptive_ai(
                    dist, obs_radius, reference_scale, ai_base
                )
                ai_adaptive_log = ai_adaptive
                
                # ===== Compute d_i và aii với adaptive params =====
                d_i = np.exp(-(d_iobs - R1**2)/sigma)
                
                expTerm = np.exp(-mu_adaptive * (d_i - alpha))
                aii = ai_adaptive * mu_adaptive * expTerm / (1 + expTerm)**2

                # ===== OBSTACLE FORCE =====
                A_obs_i = (aii * d_i * dist_vec @ Jvi) @ Ki
                A_obs_total += A_obs_i
                B += (aii * d_i * (dist_vec @ xdot_obs)).item()

                # ===== DEBUG =====
                if dist < R1:
                    print(f"\n COLLISION! dist={dist:.3f} < R1={R1:.3f}")
                elif dist < R2:
                    print(f"\n NEAR OBSTACLE: dist={dist:.3f}, R1={R1:.3f}, η={eta_adaptive:.3f}")

    # ===== FINAL CONTROL LAW (không đổi) =====
    A = A_task - A_obs_total
    A = A.reshape(1, -1)

    if (A @ A.T) < 1e-6:
        Aplus = np.zeros((6,1))
    else:
        Aplus = A.T / (A @ A.T + 1e-3)

    rhs = -eta_adaptive_log * A.T - Aplus * B  # Sử dụng eta_adaptive

    _, Jg = jointMapping(q_virtual)

    # ===== SAFETY CHECK =====
    if np.any(np.isnan(Jg)) or np.any(np.isinf(Jg)):
        print("Invalid Jg!")
        return np.zeros((6,1)), np.zeros((6,1)), (0, 0, 0, 0, 0)

    qdot_virtual = K @ np.linalg.pinv(Jg) @ rhs

    # ===== GIỮ 2D (XZ plane) =====
    qdot_virtual[0] = 0
    qdot_virtual[4] = 0
    qdot_virtual[5] = 0

    theta_dot = Jg @ qdot_virtual
    
    # Return adaptive params for logging
    adaptive_params = (eta_adaptive_log, R1_adaptive_log, R2_adaptive_log, 
                      mu_adaptive_log, ai_adaptive_log)
    
    return qdot_virtual, theta_dot, adaptive_params


# ===== MAIN LOOP =====
try:
    step_count = 0
    rtde_c.setWatchdog(0.2)
    
    # ===== Tính reference scale MỘT LẦN duy nhất =====
    tcp_start = rtde_r.getActualTCPPose()
    start_pos = np.array(tcp_start[:3])
    
    reference_scale = AdaptiveConfig.compute_reference_scale(
        start_pos, target, obs_list, axes_list
    )
    
    print(f"\n Reference scale: {reference_scale:.4f} m")
    print(f" Adaptive parameters ENABLED")
    print(f"   - η will adapt based on obstacle proximity")
    print(f"   - R1, R2 will adapt based on obstacle position relative to goal")
    print(f"   - μ, ai will adapt based on distance\n")

    while True:
        t_start = rtde_c.initPeriod()

        step_count += 1
        elapsed = step_count * dt

        tcp = rtde_r.getActualTCPPose()
        x_curr = np.array(tcp[:3])
        theta = np.array(rtde_r.getActualQ())

        # ===== CONTROLLER WITH ADAPTIVE PARAMS =====
        try:
            qdot_virtual, theta_dot, adaptive_params = highLevelController(
                x_curr, theta, target, obs_list, axes_list, start_pos, reference_scale
            )
        except Exception as e:
            print("\nController ERROR:", e)
            import traceback
            traceback.print_exc()
            break

        # Check NaN
        if np.any(np.isnan(theta_dot)) or np.any(np.isinf(theta_dot)):
            print("\nNaN detected → STOP")
            break

        # Velocity saturation
        vmax = 0.5
        theta_dot = vmax * np.tanh(theta_dot / vmax)
        
        # ===== FK để lấy vị trí joint =====
        p, _, _, _ = kinematic.JointKinematics(theta)

        # ===== LOG (bao gồm adaptive parameters) =====
        log_line = f"{elapsed:.4f}"

        # Vị trí joint
        for i in range(6):
            log_line += f",{p[0,i]:.4f},{p[1,i]:.4f},{p[2,i]:.4f}"

        # Vận tốc joint
        for i in range(6):
            log_line += f",{theta_dot[i,0]:.4f}"
        
        # Adaptive parameters
        eta_adp, R1_adp, R2_adp, mu_adp, ai_adp = adaptive_params
        log_line += f",{eta_adp:.4f},{R1_adp:.4f},{R2_adp:.4f},{mu_adp:.4f},{ai_adp:.4f}"

        log_f.write(log_line + "\n")

        # ===== STOP CONDITION =====
        dist = np.linalg.norm(x_curr - target)
        if dist < 0.01:
            total_time = elapsed
            rtde_c.speedJ([0]*6, acceleration=0.1, time=dt)
            print(f"\n Reached target! x=[{x_curr[0]:.4f}, {x_curr[1]:.4f}, {x_curr[2]:.4f}]")
            print(f"   Time: {total_time:.2f} s")
            break

        # ===== SEND COMMAND =====
        safe_speed = theta_dot.flatten().astype(float).tolist()
        rtde_c.speedJ(safe_speed, acceleration=0.1, time=dt)
        
        # ===== PRINT STATUS =====
        if step_count % int(1.0/dt) == 0:
            eta_adp, R1_adp, R2_adp, mu_adp, ai_adp = adaptive_params
            print(f"\r dist={dist:.4f} | pos=({x_curr[0]:.3f},{x_curr[2]:.3f}) | "
                  f"η={eta_adp:.3f} | R1={R1_adp:.3f}",
                  end="", flush=True)

        rtde_c.waitPeriod(t_start)
        rtde_c.kickWatchdog()

except KeyboardInterrupt:
    print("\n Stopped by user")

finally:
    print("\n🔧 Stopping robot safely...")

    try:
        log_f.close()
        print("    Log file closed")
    except:
        pass
    try:
        rtde_r.stopFileRecording()
        print("    RTDE recording stopped")
    except:
        pass
    try:
        rtde_c.speedStop()
        print("    Speed stopped")
    except:
        pass
    try:
        rtde_c.stopScript()
        print("    Script stopped")
    except:
        pass
    
    print(" Shutdown complete")