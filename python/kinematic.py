import numpy as np
from math import log
#ĐỘNG HỌC 
def DH_transform(a, alpha, d, theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    T = np.array([
        [ct, -st, 0, a],
        [st*ca, ct*ca, -sa, -d*sa],
        [st*sa, ct*sa,  ca,  d*ca],
        [0, 0, 0, 1]
    ])
    return T
def dh_transform(a, d, alpha, theta):
    T = np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        
        [np.sin(theta)*np.cos(alpha), 
         np.cos(theta)*np.cos(alpha), 
         -np.sin(alpha), 
         -np.sin(alpha)*d],
        
        [np.sin(theta)*np.sin(alpha), 
         np.cos(theta)*np.sin(alpha), 
         np.cos(alpha), 
         np.cos(alpha)*d],
        
        [0, 0, 0, 1]
    ])
    return T

def forward_kinematics_ur3(q):

    # ===== DH theo bảng của bạn =====
    d = [0.1519, 0.1198, -0.0925, 0.08535, 0.08535, 0.0819]
    a = [0, 0, 0.24365, 0.21325,  0, 0]
    alpha = [ 0, -np.pi/2, 0, 0, np.pi/2, -np.pi/2]

    # ===== apply offset góc =====
    theta = [
        q[0] + np.pi,   # q1 + 180°
        q[1],
        q[2],
        q[3] + np.pi,   # q4 + 180°
        q[4],
        q[5]
    ]

    T = np.eye(4)
    positions = []

    print("\n=== JOINT POSITIONS (FK - YOUR DH) ===")

    # base
    positions.append(T[:3, 3].copy())
    print(f"Joint 0: {positions[-1]}")

    for i in range(6):
        Ti = dh_transform(a[i], d[i], alpha[i], theta[i])
        T = T @ Ti

        pos = T[:3, 3].copy()
        positions.append(pos)

        print(f"Joint {i+1}: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")

    return np.array(positions)  # return as numpy array