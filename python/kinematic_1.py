import numpy as np

def DH_transform(a, alpha, d, theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    T = np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -d * sa],
        [st * sa, ct * sa, ca, d * ca],
        [0, 0, 0, 1]
    ])
    return T


def JointKinematics(q):
    """
    Forward Kinematics + Jacobian bằng Finite Difference (khớp thực tế nhất)
    """
    # ================== DH ĐÃ TUNE (bạn có thể tiếp tục chỉnh nhỏ) ==================
    d1 = 0.1519 + 0.00137
    d2 = 0.1198 - 0.00186
    a3 = 0.24365 - 0.00080
    a4 = 0.21325 - 0.00027
    d3 = -0.0925 - 0.00312
    d4 = 0.08535 + 0.00521
    d5 = 0.08535 - 0.00087
    d6 = 0.0819  + 0.00102

    # Forward Kinematics (giữ nguyên)
    T01 = DH_transform(0,      0,      d1, q[0])
    T12 = DH_transform(0, np.pi/2,     d2, q[1] + np.pi)
    T23 = DH_transform(a3,     0,      d3, q[2])
    T34 = DH_transform(a4,     0,      d4, q[3] - np.pi)
    T45 = DH_transform(0, np.pi/2,     d5, q[4] + np.pi)
    T56 = DH_transform(0, np.pi/2,     d6, q[5] - np.pi)

    T02 = T01 @ T12
    T03 = T02 @ T23
    T04 = T03 @ T34
    T05 = T04 @ T45
    T06 = T05 @ T56

    Ts = [np.eye(4), T01, T02, T03, T04, T05, T06]

    p = np.zeros((3, 6))
    z = np.zeros((3, 6))
    for i in range(6):
        p[:, i] = Ts[i+1][:3, 3]
        z[:, i] = Ts[i][:3, 2]

    # ====================== JACOBIAN BẰNG FINITE DIFFERENCE (SỬA CHÍNH) ======================
    delta = 1e-6   # 1e-6 thường ổn định và nhanh
    Jp = np.zeros((3, 6, 6))

    for i in range(6):           # cho từng link (i=5 là end-effector)
        pos_nom = p[:, i].copy()
        
        for j in range(6):       # perturb từng joint j
            q_pert = q.copy()
            q_pert[j] += delta
            
            # Tính lại FK cho q_pert
            T01p = DH_transform(0, 0, d1, q_pert[0])
            T12p = DH_transform(0, np.pi/2, d2, q_pert[1] + np.pi)
            T23p = DH_transform(a3, 0, d3, q_pert[2])
            T34p = DH_transform(a4, 0, d4, q_pert[3] - np.pi)
            T45p = DH_transform(0, np.pi/2, d5, q_pert[4] + np.pi)
            T56p = DH_transform(0, np.pi/2, d6, q_pert[5] - np.pi)

            T06p = T01p @ T12p @ T23p @ T34p @ T45p @ T56p
            pos_pert = T06p[:3, 3]
            
            Jp[:, j, i] = (pos_pert - pos_nom) / delta

    return p, Jp, T06, z