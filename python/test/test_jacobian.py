from rtde_receive import RTDEReceiveInterface
import numpy as np
import time
import kinematic_1 as kinematic   # file kinematic của bạn (đã có Jacobian FD)

IP = "169.254.200.99"

def rotation_to_rpy(R):
    """Chuyển Rotation Matrix sang Roll-Pitch-Yaw (đơn vị độ)"""
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6
    if not singular:
        roll  = np.arctan2(R[2,1], R[2,2])
        pitch = np.arctan2(-R[2,0], sy)
        yaw   = np.arctan2(R[1,0], R[0,0])
    else:
        roll  = np.arctan2(-R[1,2], R[1,1])
        pitch = np.arctan2(-R[2,0], sy)
        yaw   = 0
    
    return np.degrees(np.array([roll, pitch, yaw]))


def test_with_real_robot():
    print("=== TEST KINEMATICS SO SÁNH VỚI ROBOT THẬT (RTDE) ===\n")
    
    rtde_r = RTDEReceiveInterface(IP)
    time.sleep(0.5)  # đợi kết nối ổn định
    
    # Lấy dữ liệu thực tế từ robot
    q_real = np.array(rtde_r.getActualQ())
    tcp_pose = np.array(rtde_r.getActualTCPPose())   # [x, y, z, rx, ry, rz]
    
    print(f"Joint angles hiện tại (rad):")
    print(np.round(q_real, 6))
    print(f"Joint angles (deg): {np.degrees(q_real).round(3)}\n")
    
    # Tính từ code của bạn
    p, Jp, T06, z = kinematic.JointKinematics(q_real)
    
    pos_calc = T06[:3, 3]
    R_calc   = T06[:3, :3]
    rpy_calc = rotation_to_rpy(R_calc)
    
    # Dữ liệu thực tế từ robot
    pos_real = tcp_pose[:3]
    # rx, ry, rz là rotation vector (axis-angle), ta chuyển sang RPY để so sánh dễ
    from scipy.spatial.transform import Rotation as R
    rot_real = R.from_rotvec(tcp_pose[3:]).as_matrix()
    rpy_real = rotation_to_rpy(rot_real)
    
    # ====================== SO SÁNH ======================
    print(f"{'='*85}")
    print(f"SO SÁNH VỊ TRÍ END-EFFECTOR")
    print(f"{'='*85}")
    print(f"{' ':15} {'X (m)':>10} {'Y (m)':>10} {'Z (m)':>10} {'Error (mm)':>12}")
    print(f"Code tính:     {pos_calc[0]:10.5f} {pos_calc[1]:10.5f} {pos_calc[2]:10.5f} "
          f"{np.linalg.norm(pos_calc - pos_real)*1000:10.2f}")
    print(f"Robot thực:    {pos_real[0]:10.5f} {pos_real[1]:10.5f} {pos_real[2]:10.5f}")
    print(f"Error:         {np.linalg.norm(pos_calc - pos_real)*1000:10.2f} mm\n")
    
    print(f"{'='*85}")
    print(f"SO SÁNH GÓC ORIENTATION (RPY - độ)")
    print(f"{'='*85}")
    print(f"{' ':15} {'Roll':>10} {'Pitch':>10} {'Yaw':>10}")
    print(f"Code tính:     {rpy_calc[0]:10.3f} {rpy_calc[1]:10.3f} {rpy_calc[2]:10.3f}")
    print(f"Robot thực:    {rpy_real[0]:10.3f} {rpy_real[1]:10.3f} {rpy_real[2]:10.3f}")
    print(f"Error (deg):   {np.linalg.norm(rpy_calc - rpy_real):10.3f}\n")
    
    # Jacobian
    Jv = Jp[:, :, 5]   # 3x6 Jacobian tuyến tính
    
    print(f"{'='*85}")
    print(f"JACOBIAN TUYẾN TÍNH CỦA END-EFFECTOR (3x6)")
    print(f"{'='*85}")
    print("          J1       J2       J3       J4       J5       J6")
    for axis, name in zip(range(3), ['dX/dq', 'dY/dq', 'dZ/dq']):
        print(f"{name:6} {Jv[axis,0]:8.5f} {Jv[axis,1]:8.5f} {Jv[axis,2]:8.5f} "
              f"{Jv[axis,3]:8.5f} {Jv[axis,4]:8.5f} {Jv[axis,5]:8.5f}")
    
    print(f"\nNorm của từng cột Jacobian (tác động của mỗi joint):")
    for j in range(6):
        print(f"  Joint {j+1:2d}: {np.linalg.norm(Jv[:,j]):8.5f} m/rad")
    
    rtde_r.disconnect()


if __name__ == "__main__":
    try:
        test_with_real_robot()
    except Exception as e:
        print(f"Lỗi kết nối hoặc chạy: {e}")
        print("Kiểm tra lại IP robot và đảm bảo rtde_receive đã cài đặt.")