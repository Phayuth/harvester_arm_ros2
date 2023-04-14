import time
from driver import gg
g = gg()

def startup_routine():
    time.sleep(1)
    print("deactivated")
    g.deactivate()
    g.sendCommand()
    time.sleep(1)

    print("activated")
    g.activate()
    g.sendCommand()
    time.sleep(5)


    print("deactivated")
    g.deactivate()
    g.sendCommand()
    time.sleep(5)


    print("go to close")
    g.goto(255,0,0)
    g.sendCommand()

    time.sleep(5)

    print("go to open")
    g.goto(0,0,0)
    g.sendCommand()

    print("The Gripper is Ready to Recieved Command")

if __name__ == "__main__":
    startup_routine()

    g.goto(255,0,0)
    g.sendCommand()
    time.sleep(2)

    g.goto(0,0,0)
    g.sendCommand()
    time.sleep(2)

    g.goto(255,0,0)
    g.sendCommand()
    time.sleep(2)

    g.goto(0,0,0)
    g.sendCommand()
    time.sleep(2)

    g.goto(255,0,0)
    g.sendCommand()
    time.sleep(2)

    g.goto(0,0,0)
    g.sendCommand()
    time.sleep(2)
