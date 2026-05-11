import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm

# 1. Khởi tạo model UR3 bằng thông số DH chuẩn
# Cách này ổn định hơn nhiều so với đọc URDF vì nó lược bỏ các link ảo
robot = rtb.models.DH.UR3()

#robot.base = sm.SE3.Rz(np.pi)

# def JointKinematics(q):

#     q = np.array(q).reshape(-1)

#     n = 6
#     p = np.zeros((3, n))
#     z = np.zeros((3, n))
#     Jp = np.zeros((3, 6, n))

#     T = sm.SE3()  # bắt đầu từ base

#     for i in range(n):

#         A_i = robot.links[i].A(q[i])
#         T = T * A_i

#         # vị trí link i
#         p[:, i] = T.t

#         # trục z của joint i
#         z[:, i] = T.R[:, 2]

#         # Jacobian đúng cho link i
#         Ji = robot.jacob0(q, end=robot.links[i])
#         Jp[:, :, i] = Ji[:3, :]

#     T06 = T.A

#     return p, Jp, T06, z
def JointKinematics(q):

    q = np.array(q).reshape(-1)

    n = 6
    p = np.zeros((3, n))
    z = np.zeros((3, n))
    Jp = np.zeros((3, 6, n))

    T = sm.SE3()  # base

    for i in range(n):

        # 🔥 Lấy z trước khi nhân (quan trọng)
        z[:, i] = T.R[:, 2]

        # Tính transform
        A_i = robot.links[i].A(q[i])
        T = T * A_i

        # vị trí link i
        p[:, i] = T.t

        # Jacobian tại link i
        Ji = robot.jacob0(q, end=robot.links[i])
        Jp[:, :, i] = Ji[:3, :]

    T06 = T.A

    return p, Jp, T06, z