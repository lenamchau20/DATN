import numpy as np
import ki_urdf as kinematic

# ===== GLOBAL PARAM (GIỮ NGUYÊN NHƯ BẠN) =====
lamda = 1

theta_min = np.array([-2*np.pi]*6)
theta_max = np.array([ 2*np.pi]*6)

sigma = 0.1
beta  = 0.9

Ki = np.diag([1,1,1,1,1,3])
K  = np.eye(6)

no_obs_printed = False


# ===== JOINT MAPPING (EQ.6) =====
def jointMapping(q_v):
    exp_term = np.exp(-lamda*q_v)
    exp_term = np.clip(exp_term, 1e-9, 1e9)

    theta = (theta_max - theta_min)/(1+exp_term) + theta_min

    dg = lamda*(theta_max-theta_min)*exp_term/((1+exp_term)**2)
    dg = np.clip(dg, 1e-9, None)

    Jg = np.diag(dg)

    return theta, Jg


# ===== MAIN CONTROLLER =====
def highLevelController(x, theta, target, obs_list, axes_list, params):

    global no_obs_printed

    gamma, eta, mu, ai = params

    # ===== INVERSE MAPPING theta → q_virtual =====
    theta_safe = np.clip(theta, theta_min + 1e-6, theta_max - 1e-6)

    ratio = (theta_max - theta_min) / (theta_safe - theta_min)
    q_virtual = -np.log(np.maximum(ratio - 1, 1e-9)) / lamda

    # ===== FORWARD KINEMATICS =====
    p, Jp, _, _ = kinematic.JointKinematics(theta)
    Jv = Jp[[0,2], :, 5]

    # ===== ERROR (EQ.7) =====
    e = (np.array([x[0], x[2]]) - np.array([target[0], target[2]])).reshape(1,-1)

    # ===== TASK TERM =====
    A_task = gamma * e @ Jv
    xdot_d = np.zeros((2,1))
    B = (-gamma * e @ xdot_d).item()

    # ===== OBSTACLE =====
    A_obs_total = np.zeros((1,6))

    if len(obs_list) == 0:
        if not no_obs_printed:
            print("No obstacles, moving straight.")
            no_obs_printed = True
    else:
        no_obs_printed = False

        for obs_pos, axes in zip(obs_list, axes_list):

            xobs_2d = np.array([obs_pos[0], obs_pos[2]])
            xdot_obs = np.zeros((2,1))

            R1 = axes[0]
            R2 = 1.5 * R1

            # ===== alpha (paper) =====
            alpha = np.exp(-(R2**2 - R1**2)) / beta

            # ===== LOOP LINKS =====
            for i in range(2,6):

                if i == 5:
                    xi = np.array([x[0], x[2]])
                    Jvi = Jv
                else:
                    xi = np.array([p[0,i], p[2,i]])
                    Jvi = Jp[[0,2], :, i]

                dist_vec = (xi - xobs_2d).reshape(1,-1)
                d_iobs = (dist_vec @ dist_vec.T).item()
                dist = np.sqrt(d_iobs)

                # ===== d_i =====
                d_i = np.exp(-(d_iobs - R1**2)/sigma)

                # ===== a_i (sigmoid derivative) =====
                expTerm = np.exp(-mu*(d_i - alpha))
                aii = ai * mu * expTerm / (1 + expTerm)**2

                # ===== A_obs =====
                A_obs_i = (aii * d_i * dist_vec @ Jvi) @ Ki
                A_obs_total += A_obs_i

                # ===== B term =====
                B += (aii * d_i * (dist_vec @ xdot_obs)).item()

                # ===== DEBUG =====
                if dist < R1:
                    print("\n COLLISION!")
                elif dist < 1.5 * R1:
                    print("\n NEAR OBSTACLE")

    # ===== FINAL A =====
    A = A_task - A_obs_total
    A = A.reshape(1,-1)

    # ===== PSEUDO INVERSE A+ =====
    if (A @ A.T) < 1e-6:
        Aplus = np.zeros((6,1))
    else:
        Aplus = A.T / (A @ A.T + 1e-3)

    # ===== CONTROL LAW (EQ.8) =====
    rhs = -eta * A.T - Aplus * B

    # ===== MAPPING =====
    _, Jg = jointMapping(q_virtual)

    if np.any(np.isnan(Jg)) or np.any(np.isinf(Jg)):
        print("Invalid Jg!")
        return np.zeros((6,1)), np.zeros((6,1))

    # ===== FINAL qdot =====
    qdot_virtual = K @ np.linalg.pinv(Jg) @ rhs

    # ===== GIỮ 2D =====
    qdot_virtual[0] = 0
    qdot_virtual[4] = 0
    qdot_virtual[5] = 0

    theta_dot = Jg @ qdot_virtual

    return qdot_virtual, theta_dot