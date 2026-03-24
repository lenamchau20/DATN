from rtde_receive import RTDEReceiveInterface
from rtde_control import RTDEControlInterface
import time

IP = "192.168.1.102"

rtde_r = RTDEReceiveInterface(IP)
rtde_c = RTDEControlInterface(IP)

print("Testing motion along +X...")

try:
    # In vị trí ban đầu
    tcp = rtde_r.getActualTCPPose()
    print("Start position:", [round(v, 4) for v in tcp[:3]])

    #  Cho robot đi theo +X trong 2 giây
    vx = 0.02  # 5 cm/s
    duration = 2

    t0 = time.time()
    while time.time() - t0 < duration:
        rtde_c.speedL([vx, 0, 0, 0, 0, 0], 0.2, 0.01)
        time.sleep(0.01)

    rtde_c.speedStop()

    # In vị trí sau khi chạy
    tcp = rtde_r.getActualTCPPose()
    print("End position:", [round(v, 4) for v in tcp[:3]])

except KeyboardInterrupt:
    print("Stopped")

finally:
    rtde_c.speedStop()
    rtde_c.stopScript()