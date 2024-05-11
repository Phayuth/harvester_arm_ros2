import urx
import logging
import numpy as np
import time

logging.basicConfig(level=logging.WARN)

rob = urx.Robot("192.168.0.3")
print(f"> rob: {rob}")
rob.set_tcp((0, 0, 0, 0, 0, 0))
rob.set_payload(1.5, (0, 0, 0))


def send_speedj(qd, a, t=0.008):
    """
    Joint speed

    Accelerate linearly in joint space and continue with constant joint speed. The time t is optional; if
    provided the function will return after time t, regardless of the target speed has been reached. If the
    time t is not provided, the function will return when the target speed is reached.

    Example command: `speedj([0.2,0.3,0.1,0.05,0,0], 0.5, 0.5)`
    Example Parameters:
        - qd -> Joint speeds of: base=0.2 rad/s, shoulder=0.3 rad/s, elbow=0.1 rad/s, wrist1=0.05 rad/s, wrist2 and wrist3=0 rad/s
        - a = 0.5 rad/s^2 -> acceleration of the leading axis (shoulder in this case)
        - t = 0.5 s -> time before the function returns

    Parameters
    ----------
    qd : _type_
        joint speeds [rad/s]
    a : _type_
        joint acceleration [rad/s^2] (of leading axis)
    t : _type_
        time [s] before the function returns (optional)
    """
    prog = f"speedj([{qd[0]},{qd[1]},{qd[2]},{qd[3]},{qd[4]},{qd[5]}], {a}, {t})"
    rob.send_program(prog)


def send_servoj(q, a=0, v=0, t=0.008, lookahead_time=0.1, gain=300): #0.008 = 125hz
    """
    Servoj can be used for online realtime control of joint positions.
    The gain parameter works the same way as the P-term of a PID controller, where it adjusts the
    current position towards the desired (q). The higher the gain, the faster reaction the robot will have.
    The parameter lookahead_time is used to project the current position forward in time with the
    current velocity. A low value gives fast reaction, a high value prevents overshoot.
    Note: A high gain or a short lookahead time may cause instability and vibrations. Especially if the
    target positions are noisy or updated at a low frequency
    It is preferred to call this function with a new setpoint (q) in each time step (thus the default t=0.008)

    Example command: servoj([0.0,1.57,-1.57,0,0,3.14], 0, 0, 0.008, 0.1, 300)
    Example Parameters:
        • q = [0.0,1.57,-1.57,0,0,3.14] joint angles in radians representing rotations of base,
        shoulder, elbow, wrist1, wrist2 and wrist3
        • a = 0 → not used in current version
        • v = 0 → not used in current version
        • t = 0.008 time where the command is controlling the robot. The function is blocking for
        time t [S].
        • lookahead time = .1 time [S], range [0.03,0.2] smoothens the trajectory with this lookahead time
        • gain = 300 proportional gain for following target position, range [100,2000]

    Parameters
    ----------
    q : _type_
        _description_
    a : _type_
        _description_
    v : _type_
        _description_
    t : float, optional
        _description_, by default 0.008
    lookahead_time : float, optional
        _description_, by default 0.1
    gain : int, optional
        _description_, by default 300
    """
    prog = f"servoj([{q[0]},{q[1]},{q[2]},{q[3]},{q[4]},{q[5]}], {a}, {v}, {t}, {lookahead_time}, {gain})"
    # prog = f"servoj([{q[0]},{q[1]},{q[2]},{q[3]},{q[4]},{q[5]}])"
    rob.send_program(prog)


def send_movej(q, a, v, t, r):
    """
    Move to position (linear in joint-space)
    When using this command, the robot must be at a standstill or come from a movej or movel with a
    blend. The speed and acceleration parameters control the trapezoid speed profile of the move.
    Alternatively, the t parameter can be used to set the time for this move. Time setting has priority over
    speed and acceleration settings.

    If a blend radius is set, the robot arm trajectory will be modified to avoid the robot stopping at the point.
    However, if the blend region of this move overlaps with the blend radius of previous or following
    waypoints, this move will be skipped, and an ``Overlapping Blends`` warning message will be generated.

    Example command: movej([0,1.57,-1.57,3.14,-1.57,1.57], a=1.4, v=1.05, t=0, r=0)
    Example Parameters:
        • q = [0,1.57,-1.57,3.14,-1.57,1.57] base is at 0 deg rotation, shoulder is at 90 deg rotation,
        elbow is at -90 deg rotation, wrist 1 is at 180 deg rotation, wrist 2 is at -90 deg
        rotation, wrist 3 is at 90 deg rotation. Note: joint positions (q can also be specified as a
        pose, then inverse kinematics is used to calculate the corresponding joint positions)
        • a = 1.4 → acceleration is 1.4 rad/s/s
        • v = 1.05 → velocity is 1.05 rad/s
        • t = 0 the time (seconds) to make move is not specified. If it were specified the command would ignore the a and v values.
        • r = 0 → the blend radius is zero meters.

    Parameters
    ----------
    q : _type_
        joint positions (q can also be specified as a pose, then inverse kinematics is used to calculate the corresponding joint positions)
    a : _type_
        joint acceleration of leading axis [rad/s^2]
    v : _type_
        joint speed of leading axis [rad/s]
    t : _type_
        time [S]
    r : _type_
        blend radius [m]
    """
    prog = f"movej([{q[0]},{q[1]},{q[2]},{q[3]},{q[4]},{q[5]}], {a}, {v}, {t}, {r})"
    rob.send_program(prog)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    time.sleep(1)
    t = 0.0
    freq = np.pi / 1.4
    tt = []
    qc = []
    qd = []
    try:
        # a = rob.get_pos()
        # print(f"> a: {a}")
        # b = rob.get_pose()
        # print(f"> b: {b}")
        # c = rob.getj()
        # print(f"> c: {c}")

        # while t < 10.0:
        #     # get current
        #     qci = rob.getj()
        #     qc.append(qci)

        #     # get desired
        #     pos = np.sin(freq*t)
        #     qdi = [pos, -1.5708, 0.0, 0.0, 1.5708, 0.0]
        #     qd.append(qdi)

        #     send_servoj(qdi)

        #     t += 0.008
        #     tt.append(t)

        #     time.sleep(0.008)

        # qqq = 3.0
        # while qqq > 0.0:
        #     jnow = rob.getj()

        #     s = time.perf_counter()

        #     qdi = [0.0, -1.5708, 0.0, 0.0, 1.5708, 0.0]
        #     send_servoj(qdi, 0.0, 0.0, lookahead_time=0.0, gain=0)

        #     e = time.perf_counter()
        #     time.sleep(e - s)
        #     # print(f"elap : {e - s}")

        #     jj = rob.getj()
        #     qqq = jj[0]
        #     print(f"> qqq: {qqq}")


        jnow = rob.getj()
        print(f"> jnow: {jnow}")

        s = time.perf_counter()

        qdi = [1.55, -1.5708, 0.0, 0.0, 1.5708, 0.0]
        send_servoj(qdi, 0.0, 0.0, lookahead_time=0.0, gain=0)

        e = time.perf_counter()
        time.sleep(e - s)
        # print(f"elap : {e - s}")

        jj = rob.getj()
        print(f"> jj: {jj}")

        # rob.stopj()

    finally:
        rob.close()

    # print(qc)
    # print(qd)
    # print(tt)
    # plt.plot(tt, qc, label="qc")
    # plt.plot(tt, qd, label="qd")
    # plt.legend()
    # plt.show()