from rtde_receive import RTDEReceiveInterface
import numpy as np
import time

IP = "192.168.1.102"
rtde_r = RTDEReceiveInterface(IP)

print(" Bật freedrive và kéo robot...")

v_list = []

try:
    while True:
        tcp_speed = rtde_r.getActualTCPSpeed()
        v = np.array(tcp_speed[:3])   # chỉ lấy linear

        speed = np.linalg.norm(v)
        v_list.append(speed)

        print(f"\rv = {speed:.3f} m/s", end="")

        time.sleep(0.008)

except KeyboardInterrupt:
    print("\nStopped")

    v_array = np.array(v_list)

    print("\n===== RESULT =====")
    print(f"Max speed: {np.max(v_array):.3f} m/s")
    print(f"Mean speed: {np.mean(v_array):.3f} m/s")
    print(f"95% percentile: {np.percentile(v_array, 95):.3f} m/s")