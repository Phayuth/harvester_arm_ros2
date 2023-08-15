import time
import robotiq_85_gripper

#0 is fully open
#255 is fully closed

g = robotiq_85_gripper.Robotiq85Gripper()

def startup_routine():
    time.sleep(1)
    print("deactivated")
    g.deactivate_gripper()
    g.process_act_cmd()
    time.sleep(1)

    print("activated")
    g.activate_gripper()
    g.process_act_cmd()
    time.sleep(2)


    print("deactivated")
    g.deactivate_gripper()
    g.process_act_cmd()
    time.sleep(2)


    print("go to close")
    g.goto(pos=255)
    g.process_act_cmd()

    time.sleep(5)

    print("go to open")
    g.goto(pos=0)
    g.process_act_cmd()
    time.sleep(5)

    print("The Gripper is Ready to Recieved Command")

if __name__ == "__main__":
    # run after the power-on 1 TIME ONLY.
    # if run multiple time, the register will get cluter with incorrect bit.
    # and I don't know how to fix it since I don't understand any of this.

    # startup_routine()

    g.goto(pos=255)
    g.process_act_cmd()
    time.sleep(2)
    print("1")

    g.goto(pos=0)
    g.process_act_cmd()
    time.sleep(2)
    print("1")

    g.goto(pos=255)
    g.process_act_cmd()
    time.sleep(2)
    print("1")

    g.goto(pos=0)
    g.process_act_cmd()
    time.sleep(2)
    print("1")

    g.goto(pos=255)
    g.process_act_cmd()
    time.sleep(2)
    print("1")

    g.goto(pos=0)
    g.process_act_cmd()
    time.sleep(2)
    print("1")
