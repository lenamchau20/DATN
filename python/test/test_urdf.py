from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
import numpy as np
import time
import os
from ki_urdf import JointKinematics # Đảm bảo file ki_urdf.py nằm cùng thư mục

# --- CẤU HÌNH ROBOT ---
IP = "169.254.200.99"

try:
    rtde_r = RTDEReceiveInterface(IP)
    # rtde_c = RTDEControlInterface(IP) # Tạm thời comment nếu chỉ test đọc
    print(f"✅ Kết nối Robot tại {IP} thành công!")
except Exception as e:
    print(f"❌ Lỗi kết nối Robot: {e}")
    exit()

print("\n" + "="*60)
print(f"{'THÔNG SỐ':<15} | {'RTDE (ROBOT THẬT)':<20} | {'RTB (MODEL)':<20}")
print("="*60)

try:
    while True:
        # 1. ĐỌC GÓC KHỚP THỰC TẾ
        q = np.array(rtde_r.getActualQ())

        # 2. TÍNH TOÁN QUA MÔ HÌNH (Đã xoay Base 180 độ trong ki_urdf)
        # p: vị trí 6 link, Jp: Jacobian 6 link, T06: Ma trận EE, z: trục khớp
        p, Jp, T06, z = JointKinematics(q)
        
        # 3. ĐỌC VỊ TRÍ TCP THỰC TẾ TỪ ROBOT
        tcp_real = np.array(rtde_r.getActualTCPPose()[:3])
        
        # 4. TRÍCH XUẤT VỊ TRÍ EE TỪ MÔ HÌNH (Cột cuối cùng của p)
        tcp_model = p[:, 5] 

        # 5. IN SO SÁNH TRỰC TIẾP
        os.system('cls' if os.name == 'nt' else 'clear') # Xóa màn hình để nhìn cho sạch
        print("=== KIỂM TRA ĐỒNG BỘ TỌA ĐỘ UR3 ===")
        print(f"{'THÔNG SỐ':<15} | {'RTDE (ROBOT THẬT)':<20} | {'RTB (MODEL)':<20}")
        print("-" * 60)
        print(f"{'X (m)':<15} | {tcp_real[0]:>18.4f} | {tcp_model[0]:>18.4f}")
        print(f"{'Y (m)':<15} | {tcp_real[1]:>18.4f} | {tcp_model[1]:>18.4f}")
        print(f"{'Z (m)':<15} | {tcp_real[2]:>18.4f} | {tcp_model[2]:>18.4f}")
        
        # Kiểm tra sai số
        error = np.linalg.norm(tcp_real - tcp_model)
        print("-" * 60)
        print(f"Sai số tổng hợp: {error*1000:.2f} mm")
        
        # In thêm vị trí các Link trung gian (để phục vụ tránh vật cản)
        print("\n--- TỌA ĐỘ CÁC LINK (MODEL) ---")
        for i in range(6):
            print(f"Link {i+1}: {np.round(p[:,i], 3)}")

        time.sleep(0.5) # Cập nhật mỗi 0.5 giây

except KeyboardInterrupt:
    print("\nĐã dừng chương trình.")
finally:
    # Luôn ngắt kết nối an toàn
    rtde_r.disconnect()
    print("Đã ngắt kết nối RTDE.")