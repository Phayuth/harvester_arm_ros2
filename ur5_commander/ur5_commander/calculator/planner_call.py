import sys
sys.path.append("/home/yuth/ws_yuthdev/robotics_manipulator")

import time
import numpy as np
from planner_dev.devplaner_copsim import DevPlanner
from planner_util.extract_path_class import extract_path_class_6d
from util.dictionary_pretty import print_dict

np.random.seed(9)
# UR5
# thetaInit = np.array([0, 0, 0, 0, 0, 0]).reshape(6, 1)
# thetaApp = np.array([-1.57, -2.1, -1.6, 0.1, 1.57, 0]).reshape(6, 1)
# thetaGoal = np.array([-1.57, -2.3, -1.4, 0.1, 1.57, 0]).reshape(6, 1)

# UR5e
thetaInit = np.array([np.deg2rad(-27.68), np.deg2rad(2.95), np.deg2rad(-26.58), np.deg2rad(-15.28), np.deg2rad(87.43), np.deg2rad(0.0)]).reshape(6, 1)
thetaGoal = np.array([np.deg2rad(124.02), np.deg2rad(-70.58), np.deg2rad(-91.21), np.deg2rad(-50.84), np.deg2rad(105.67), np.deg2rad(-0.02)]).reshape(6, 1)
thetaApp = np.array([np.deg2rad(124.02), np.deg2rad(-56.47), np.deg2rad(-91.21), np.deg2rad(-59.58), np.deg2rad(103.13), np.deg2rad(-0.08)]).reshape(6, 1)

planner = DevPlanner(thetaInit, thetaApp, thetaGoal, eta=0.1, maxIteration=5000)
path = planner.planning()
print(f"==>> path: \n{path}")
print_dict(planner.perfMatrix)

time.sleep(3)

# play back
planner.copHandle.start_sim()
pathX, pathY, pathZ, pathP, pathQ, pathR = extract_path_class_6d(path)
pathX = np.array(pathX)
pathY = np.array(pathY)
pathZ = np.array(pathZ)
pathP = np.array(pathP)
pathQ = np.array(pathQ)
pathR = np.array(pathR)

# loop in simulation
for i in range(len(pathX)):
    jointVal = np.array([pathX[i], pathY[i], pathZ[i], pathP[i], pathQ[i], pathR[i]]).reshape(6, 1)
    planner.copHandle.set_joint_value(jointVal)
    time.sleep(0.5)
    # triggers next simulation step
    # client.step()

# stop simulation
planner.copHandle.stop_sim()
