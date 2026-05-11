import numpy as np
import time
import csv
import os
from rtde_receive import RTDEReceiveInterface
from scipy.optimize import least_squares

IP = "169.254.200.99"
DATA_FILE = "dh_calibration_data.csv"

# ====================== NOMINAL DH ======================
nominal_dh = {
    'd1': 0.1519, 'd2': 0.1198, 'a3': 0.24365, 'a4': 0.21325,
    'd3': -0.0925, 'd4': 0.08535, 'd5': 0.08535, 'd6': 0.0819
}

offsets = [0.0, np.pi, 0.0, -np.pi, np.pi, -np.pi]  # offset góc hiện tại

# ====================== FORWARD KINEMATICS (có tham số biến đổi) ======================
def fk_with_params(q, params):
    d1, d2, a3, a4, d3, d4, d5, d6 = params
    def DH(a, alpha, d, theta):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st, 0, a],
            [st*ca, ct*ca, -sa, -d*sa],
            [st*sa, ct*sa, ca, d*ca],
            [0, 0, 0, 1]
        ])
    
    T01 = DH(0,      0,      d1, q[0] + offsets[0])
    T12 = DH(0, np.pi/2,     d2, q[1] + offsets[1])
    T23 = DH(a3,     0,      d3, q[2] + offsets[2])
    T34 = DH(a4,     0,      d4, q[3] + offsets[3])
    T45 = DH(0, np.pi/2,     d5, q[4] + offsets[4])
    T56 = DH(0, np.pi/2,     d6, q[5] + offsets[5])

    T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56
    return T06[:3, 3]   # chỉ vị trí x,y,z

# ====================== COLLECT DATA ======================
def collect_data(n_points=12):
    rtde_r = RTDEReceiveInterface(IP)
    print("=== BẮT ĐẦU THU THẬP DỮ LIỆU DH ===")
    print("Di chuyển robot bằng tay (jog) đến các tư thế khác nhau rồi nhấn ENTER để ghi...")
    print("Gợi ý: dùng ít nhất 8-12 tư thế khác nhau, tránh singular.")

    data = []
    for i in range(n_points):
        input(f"\nPose {i+1}/{n_points} - Di chuyển robot rồi nhấn ENTER để ghi...")
        q = np.array(rtde_r.getActualQ())
        tcp = np.array(rtde_r.getActualTCPPose())
        pos_real = tcp[:3]
        data.append(np.concatenate([q, pos_real]))
        print(f"  ✓ Ghi pose {i+1}: q={q.round(4)} | pos={pos_real.round(5)}")

    rtde_r.disconnect()

    # Lưu file
    with open(DATA_FILE, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['q1','q2','q3','q4','q5','q6','x_real','y_real','z_real'])
        writer.writerows(data)
    print(f"\nĐã lưu {len(data)} pose vào {DATA_FILE}")
    return data

# ====================== OPTIMIZATION ======================
def objective(params, q_list, pos_real_list):
    error = []
    for q, pos_real in zip(q_list, pos_real_list):
        pos_calc = fk_with_params(q, params)
        error.append(pos_calc - pos_real)
    return np.concatenate(error)   # vector lỗi 3*N

def run_calibration():
    if not os.path.exists(DATA_FILE):
        print("Chưa có file dữ liệu! Chạy collect_data trước.")
        return

    data = np.loadtxt(DATA_FILE, delimiter=',', skiprows=1)
    q_list = data[:, :6]
    pos_real_list = data[:, 6:9]

    print(f"Đang tối ưu với {len(q_list)} pose...")

    # Khởi tạo từ nominal
    initial_params = np.array(list(nominal_dh.values()))

    # Giới hạn ±10mm (an toàn)
    bounds = ([p-0.01 for p in initial_params], [p+0.01 for p in initial_params])

    res = least_squares(objective, initial_params, args=(q_list, pos_real_list),
                        bounds=bounds, ftol=1e-12, xtol=1e-12, gtol=1e-12)

    tuned_params = res.x
    final_error_mm = np.linalg.norm(res.fun) / len(q_list) * 1000

    print("\n" + "="*60)
    print("KẾT QUẢ HIỆU CHỈNH DH")
    print("="*60)
    param_names = ['d1', 'd2', 'a3', 'a4', 'd3', 'd4', 'd5', 'd6']
    for name, old, new in zip(param_names, initial_params, tuned_params):
        print(f"{name:3s}: {old:.5f} → {new:.5f}  (delta = {new-old:+.5f})")
    print(f"\nLỗi trung bình sau hiệu chỉnh: {final_error_mm:.3f} mm")
    print("="*60)

    # Lưu tham số mới
    np.save("dh_tuned.npy", tuned_params)
    print("Đã lưu tham số tối ưu vào dh_tuned.npy")

    return tuned_params

# ====================== MAIN ======================
if __name__ == "__main__":
    print("1. Thu thập dữ liệu (collect)")
    print("2. Chạy tối ưu DH (calibrate)")
    choice = input("Chọn (1 hoặc 2): ").strip()
    
    if choice == "1":
        collect_data(n_points=12)      # bạn có thể thay số pose
    elif choice == "2":
        run_calibration()
    else:
        print("Chọn 1 hoặc 2!")