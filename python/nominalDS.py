from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
import numpy as np
import time
import os


IP = "192.168.1.102"


rtde_r = RTDEReceiveInterface(IP)
rtde_c = RTDEControlInterface(IP)

# --- THIẾT LẬP GHI DỮ LIỆU ---
# Chọn các biến cần thiết để vẽ quỹ đạo
folder = "logs"
os.makedirs(folder, exist_ok=True)

base_name = "trajectory_ds"
i = 1

while True:
    filename = os.path.join(folder, f"{base_name}_{i}.csv")
    if not os.path.exists(filename):
        break
    i += 1

record_vars = ["timestamp", "actual_TCP_pose"]
rtde_r.startFileRecording(filename, record_vars)
print(f"Bắt đầu ghi dữ liệu vào file: {filename}")

# Target 
target = np.array([-0.446, -0.013, 0.207])

k = 0.5        # gain (tốc độ hội tụ)
dt = 0.008      # 125 Hz

print("Moving to target")

try:
    while True:
        # Sử dụng initPeriod để giữ nhịp thời gian thực chính xác hơn time.sleep
        t_start = rtde_c.initPeriod()

        tcp = rtde_r.getActualTCPPose()
        x_curr = np.array(tcp[:3])

        #DS
        v_nominal = -k * (x_curr - target)
        v_safe = v_nominal

        # limit velocity 
        v_max = 0.15
        norm_v = np.linalg.norm(v_safe)
        if norm_v > v_max:
            v_safe = v_safe / norm_v * v_max

        # điều kiện dừng
        dist = np.linalg.norm(x_curr - target)
        if dist < 0.005:
            rtde_c.speedL([0,0,0,0,0,0], 0.2, dt)
            print("Reached target!")
            break

        #Send command
        cmd_vel = [v_safe[0], v_safe[1], v_safe[2], 0, 0, 0]
        rtde_c.speedL(cmd_vel, acceleration=0.2, time=dt)

        # In thông tin để theo dõi
        if int(time.time()*10) % 5 == 0:   # ~0.5s in 1 lần
            print(f"\rx={x_curr.round(3)} | v={v_safe.round(3)} | dist={dist:.4f}", end="")

        # Chờ cho đến hết chu kỳ dt
        rtde_c.waitPeriod(t_start)

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    # --- DỪNG GHI DỮ LIỆU ---
    rtde_r.stopFileRecording()
    rtde_c.speedStop()
    rtde_c.stopScript()