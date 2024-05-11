import rtde_control
import rtde_receive
import numpy as np
import time
import numpy as np
import matplotlib.pyplot as plt


rtde_c = rtde_control.RTDEControlInterface("192.168.0.3")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.0.3")

# from ros2 joint state
# -1.4845843315124512, -1.8072849712767542, -1.5749934355365198, -0.23041661203417974, 1.3987674713134766, 0.015793323516845703

# from rtde rtde_r.getActualQ()
# -1.5749571959124964, -1.8072849712767542, -1.484571933746338, -0.23039277017626958, 1.3987674713134766, 0.015793323516845703

# the 1st and 3rd joint value is switched

def servoj_sinewave():
    poshist = []
    poscmd = []

    velocity = 0.5  # rad/s
    acceleration = 0.5  # rad/ss
    dt = 1.0 / 500  # 2ms
    lookahead_time = 0.1  # s
    gain = 300
    joint_q = [0.0, -1.5708, 0.0, 0.0, 1.5708, 0.0]

    rtde_c.moveJ(joint_q, speed=0.1, acceleration=0.1)
    freq = np.pi / 6.0
    try:
        # Execute 500Hz control loop for 2 seconds, each cycle is 2ms
        for i in range(30000):
            t_start = rtde_c.initPeriod()
            joint_q[0] = np.sin(freq * i * dt)

            poscmd.append(joint_q.copy())
            poshist.append(rtde_r.getActualQ())

            rtde_c.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain)
            rtde_c.waitPeriod(t_start)

    except KeyboardInterrupt:
        pass

    rtde_c.servoStop()
    rtde_c.stopScript()

    poshist = np.array(poshist)
    poscmd = np.array(poscmd)

    pq = poshist[:, 0]
    poscmd = poscmd[:, 0]
    t = np.linspace(0, 1, len(pq))

    plt.plot(t, pq)
    plt.show()

    plt.plot(t, pq, label="actual pose")
    plt.plot(t, poscmd, label="cmd pose")
    plt.legend()
    plt.show()


# servoj_sinewave()


def speedj_sinewave():
    poshist = []
    velhist = []
    velcmdhist = []

    acceleration = 1.5
    dt = 1.0 / 500  # 2ms
    joint_q = [0.0, -1.5708, 0.0, 0.0, 1.5708, 0.0]
    rtde_c.moveJ(joint_q, speed=0.1, acceleration=0.3)

    freq = np.pi / 2.2

    try:
        # Execute 500Hz control loop for 2 seconds, each cycle is 2ms
        for i in range(30000):
            print(i)
            t_start = rtde_c.initPeriod()
            # rtde_c.speedJ(joint_speed, acceleration, dt)
            joint_speed = [0.95 * np.sin(freq * i * dt), 0.0, 0.0, 0.0, 0.0, 0.0]

            poshist.append(rtde_r.getActualQ())
            velhist.append(rtde_r.getActualQd())
            velcmdhist.append(joint_speed)

            rtde_c.speedJ(joint_speed, acceleration, dt)
            rtde_c.waitPeriod(t_start)

    except KeyboardInterrupt:
        pass

    rtde_c.speedStop()
    rtde_c.stopScript()
    print("stop clean")

    poshist = np.array(poshist)
    velhist = np.array(velhist)
    velcmdhist = np.array(velcmdhist)

    pq = poshist[:, 0]
    vq = velhist[:, 0]
    vcq = velcmdhist[:, 0]
    t = np.linspace(0, 1, len(pq))

    plt.plot(t, pq)
    plt.show()

    plt.plot(t, vq, label="actual v")
    plt.plot(t, vcq, label="cmd v")
    plt.legend()
    plt.show()


# speedj_sinewave()


def position_control_velo_cmd():
    poshist = []
    velhist = []
    velcmdhist = []

    posed = 2.2

    acceleration = 1.5
    dt = 1.0 / 500  # 2ms
    joint_q = [0.0, -1.5708, 0.0, 0.0, 1.5708, 0.0]
    rtde_c.moveJ(joint_q, speed=0.2, acceleration=0.3)
    k_d = 1.0

    try:
        while True:
            t_start = rtde_c.initPeriod()

            # p controller
            posec = rtde_r.getActualQ()
            e = k_d * (posed - posec[0])

            # rtde_c.speedJ(joint_speed, acceleration, dt)
            joint_speed = [e, 0.0, 0.0, 0.0, 0.0, 0.0]

            poshist.append(rtde_r.getActualQ())
            velhist.append(rtde_r.getActualQd())
            velcmdhist.append(joint_speed)

            rtde_c.speedJ(joint_speed, acceleration, dt)
            rtde_c.waitPeriod(t_start)

    except KeyboardInterrupt:
        pass

    rtde_c.speedStop()
    rtde_c.stopScript()
    print("stop clean")

    poshist = np.array(poshist)
    velhist = np.array(velhist)
    velcmdhist = np.array(velcmdhist)

    pq = poshist[:, 0]
    vq = velhist[:, 0]
    vcq = velcmdhist[:, 0]
    t = np.linspace(0, 1, len(pq))

    posed = np.full_like(t, posed)
    plt.plot(t, pq, label="actual pose")
    plt.plot(t, posed, label="cmd pose")
    plt.legend()
    plt.show()

    plt.plot(t, vq, label="actual v")
    plt.plot(t, vcq, label="cmd v")
    plt.legend()
    plt.show()


# position_control_velo_cmd()
