import numpy as np
import rtde_control
import rtde_receive
import time
import matplotlib.pyplot as plt


rtde_c = rtde_control.RTDEControlInterface("192.168.0.3")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.0.3")

# from techpendant
# -10.0 -80.0 -97.0 -30.0 9.0 0.0

# it seem ros joint is not start in order
# elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint
rosjointvalue = np.rad2deg([-1.692978858947754, -1.3962847751430054, -0.17454559007753545, -0.5235684675029297, 0.15709757804870605, -3.987947572881012e-05])

rtdejointvalue = np.rad2deg(rtde_r.getActualQ())

# > rosjointvalue: [-9.70005434e+01 -8.00012246e+01 -1.00007256e+01 -2.99982635e+01 9.00102819e+00 -2.28492565e-03]
# > rtdejointvalue: [-9.99796625e+00 -8.00012246e+01 -9.70033028e+01 -3.00010229e+01 9.00034518e+00 -8.91567058e-04]
