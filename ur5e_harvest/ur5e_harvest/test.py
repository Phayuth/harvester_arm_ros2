import sys
import numpy as np
import rtde_control
import rtde_receive
import time
import cv2
import matplotlib.pyplot as plt
import pytransform3d.camera as pc
import pytransform3d.transformations as pt

np.set_printoptions(suppress=True)
sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")
from spatial_geometry.spatial_transformation import RigidBodyTransformation as rbt

rtde_c = rtde_control.RTDEControlInterface("192.168.0.3")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.0.3")


# q = rtde_r.getActualQ()


def tcp_to_h(tcp):
    R, _ = cv2.Rodrigues(np.array(tcp[3:6]))
    H = np.eye(4) @ rbt.ht(tcp[0], tcp[1], tcp[2])
    H[:3, :3] = R
    return H


def h_to_tcp(h):
    tvec = h[0:3, 3].tolist()
    rvec, _ = cv2.Rodrigues(h[:3, :3])
    return tvec + rvec.flatten().tolist()


def get_actual_h():
    t = rtde_r.getActualTCPPose()
    return tcp_to_h(t)


def move_tcp(tcp, asyn=False):
    rtde_c.moveL(tcp, 0.25, 0.5, asyn)


# d  = 0.483594594
# h =


x1 = np.array([0.27807259335283196, -0.30019781096384157, 0.6473946314428765, -1.6166917736115658, 1.4040813480080727, -1.0089979438086432])
x2 = np.array([0.27805100450130654, 0.0019913585353435563, 0.6473984922940137, -1.616586865451318, 1.4041997172743135, -1.0090335750800299])
x3 = np.array([0.27806122057702065, 0.0020114963649783164, 0.559859533380667, -1.6166187065971134, 1.4041989683614604, -1.0090056371005565])
x4 = np.array([0.27806122057702065, -0.30019781096384157, 0.559859533380667, -1.6166187065971134, 1.4041989683614604, -1.0090056371005565])

seg1 = np.linspace(x1, x2, num=3, endpoint=False)
seg2 = np.linspace(x2, x3, num=1, endpoint=False)
seg3 = np.linspace(x3, x4, num=3, endpoint=False)
seg4 = np.linspace(x4, x1, num=1, endpoint=False)

tcplists = np.vstack((seg1, seg2, seg3, seg4))

# plot
ax = pt.plot_transform(name="base")
for tcpi in tcplists:
    H = tcp_to_h(tcpi)
    pt.plot_transform(ax, H, name="TCP")
plt.show()

timesave = []

try:
    for i in range(10):
        timestart = time.perf_counter_ns()
        for tcpi in tcplists:
            move_tcp(tcpi)
        timestop = time.perf_counter_ns()
        timesave.append(timestop - timestart)
    timemean_ns = np.mean(timesave)
    timemean_ms = timemean_ns*1e-6

    print(f"> timemean_ns: {timemean_ns}")
    print(f"> timemean_ms: {timemean_ms}")

except KeyboardInterrupt:
    print("killed")
finally:
    rtde_c.servoStop()
    rtde_c.stopScript()
