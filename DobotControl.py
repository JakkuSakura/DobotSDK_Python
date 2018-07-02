import ctypes
import math
import random
import threading
import time

from typing import List

import DobotDllType as dType


class After(threading.Thread):
    def __init__(self, sec, cmd):
        super().__init__()
        self.sec = sec
        self.cmd = cmd

    def run(self):
        time.sleep(self.sec)
        self.cmd()


# Load Dll
api = dType.load()

# realtime posizition
isRunning = True

dType.Debug = False
comes = False

last_color = []
color_fix_set = {'b': [0, 0, 0], 'r': [0, 5, 0], 'g': [0, -6, 0], 'k': [0, 0, 0]}


def color_fix(cr):
    fix = color_fix_set[cr]
    print("fix result", fix, cr)
    return fix


class Timer(threading.Thread):
    def run(self):
        global comes, last_color
        print("Timer started")
        while isRunning:
            pos = dType.GetPose(api)
            # print(pos)
            nowcolor = dType.GetColorSensor(api)
            if 1 in nowcolor:
                stopMoto()
                comes = True
                last_color = nowcolor
            dType.dSleep(100)


def nowPos() -> List[float]:
    return dType.GetPose(api)


def suck():
    dType.SetEndEffectorSuctionCupEx(api, 1, 1)


def unsuck():
    dType.SetEndEffectorSuctionCupEx(api, 0, 1)


def moveTo(x, y, z):
    dType.SetPTPCmdEx(api, dType.PTPMode.PTP_MOVJ_XYZ_Mode, x, y, z, 0, 1)


def moveInc(dx=0, dy=0, dz=0):
    dType.SetPTPCmdEx(api, dType.PTPMode.PTP_MOVJ_XYZ_INC_Mode, dx, dy, dz, 0, 1)


def moveToPart(x, y, z, part: list):
    pos = nowPos()
    to = (x, y, z)
    delta = [to[i] - pos[i] for i in range(3)]
    for i in range(len(part)-1):
        l = [j * part[i] for j in delta]
        l[2] = l[2] + 30.0
        moveInc(*tuple(l))
    moveTo(x, y, z)


def reletivePose(pose, dx=0, dy=0, dz=0):
    return pose[0] + dx, pose[1] + dy, pose[2] + dz


def front(x):
    moveInc(dx=x)


def back(x):
    moveInc(dx=-x)


def right(x):
    moveInc(dy=-x)


def left(x):
    moveInc(dy=x)


def up(x):
    moveInc(dz=x)


def down(x):
    moveInc(dz=-x)


l = ['r', 'g', 'b', 'k']


def getColorName(color):
    for i in range(len(color)):
        if color[i] != 0:
            return l[i]
    else:
        return l[3]


carry_to_set = {
    'r': (65, -173, -37),
    'g': (65, -173, -37),
    'b': (65, -173, -33),
}


def carry(frm):
    global comes
    if not comes:
        After(sec=0.5, cmd=startMoto).start()
        moveToPart(*frm, [0.4, 0.6])
    else:
        moveTo(*frm)
    while not comes:
        time.sleep(0.01)
    moveInc(*reletivePose(color_fix(getColorName(last_color))))
    down(25)
    suck()
    up(30)
    moveTo(*carry_to_set[getColorName(last_color)])
    unsuck()

    comes = False


def startMoto():
    vel = float(70) * 282.94212105225836
    dType.SetEMotor(api, 1, 1, int(vel), 0)


def stopMoto():
    dType.SetEMotor(api, 1, 0, 0, 0)


base_pose = (249, -9, 25)


def getColor():
    return dType.GetColorSensor(api)


def init():
    print("Initing dobot")
    dType.ClearAllAlarmsState(api)
    dType.SetQueuedCmdClear(api)
    dType.SetQueuedCmdStartExec(api)
    dType.SetHOMEParams(api, 270, 0, 50, 0, 1)

    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, 1)
    dType.SetPTPCoordinateParams(api, 200, 200, 200, 200, 1)
    dType.SetPTPJumpParams(api, 10, 200, 1)
    dType.SetPTPCommonParams(api, 100, 100, 1)

    dType.SetColorSensor(api, 1, dType.ColorPort.PORT_GP5)
    # dType.SetHOMECmdEx(api, temp=0, isQueued=1)

    moveTo(*base_pose)

    Timer().start()
    dType.dSleep(200)
    if getColorName(getColor()) == 'k':
        startMoto()


def do():
    for x in range(24):
        print("Step", x)
        carry(reletivePose(base_pose))
    pass


def clean():
    global isRunning
    isRunning = False
    stopMoto()
    unsuck()
    dType.SetQueuedCmdStopExec(api)


if __name__ == '__main__':
    state = dType.ConnectDobot(api, "", 115200)[0]
    print("Connect status:", dType.CONNECT_RESULT[state])

    if state == dType.DobotConnect.DobotConnect_NoError:
        try:
            init()
            do()
            # dType.DobotExec(api)
        except KeyboardInterrupt:
            pass
        finally:
            clean()
            dType.DisconnectDobot(api)
