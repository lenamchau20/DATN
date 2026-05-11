from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
import numpy as np
import time
import os 
from kinematic import forward_kinematics_ur3, jacobian_all_joints # Đảm bảo file kinematic.py nằm cùng thư mục

IP = "169.254.200.99"

try:
    rtde_r = RTDEReceiveInterface(IP)
    # rtde_c = RTDEControlInterface(IP) # Tạm thời comment nếu chỉ test đọc
    print(f"✅ Kết nối Robot tại {IP} thành công!")
except Exception as e:
    print(f"❌ Lỗi kết nối Robot: {e}")
    exit()

try:
    while True:
        # 1. ĐỌC GÓC KHỚP THỰC TẾ
        q = np.array(rtde_r.getActualQ())
        J,p = jacobian_all_joints(q)
        pos=forward_kinematics_ur3(q)

        tcp_real = np.array(rtde_r.getActualTCPPose()[:3])

        e = np.linalg.norm(tcp_real - pos[6])  # TCP là joint 6 (index 6)
        print("\n=== TCP POSITION COMPARISON ===")
        print(f"TCP from FK: x={pos[6][0]:.4f}, y={pos[6][1]:.4f}, z={pos[6][2]:.4f}")
        print(f"TCP from Robot: x={tcp_real[0]:.4f}, y={tcp_real[1]:.4f}, z={tcp_real[2]:.4f}")
        print(f"Sai số tổng hợp: {e*1000:.2f} mm")
        print(f"Jacobian at current q:\n{J[6]}")  # In Jacobian của joint 6 (TCP)
        for i in range(6):
            print(f"Joint {i+1} position: {p[i]}")

        time.sleep(0.5) # Cập nhật mỗi 0.5 giây
except KeyboardInterrupt:
    print("\nĐã dừng chương trình.")
    rtde_r.disconnect()
