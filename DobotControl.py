import math
import random
import threading
import DobotDllType as dType


def rand():
    return random.randint(0, 100)


# Load Dll
api = dType.load()
if __name__ == '__main__':

    # Connect Dobot
    state = dType.ConnectDobot(api, "", 115200)[0]
    print("Connect status:", dType.CONNECT_RESULT[state])

    if state == dType.DobotConnect.DobotConnect_NoError:
        # Clean Command Queued
        dType.SetQueuedCmdClear(api)

        # Async Motion Params Setting
        dType.SetHOMEParams(api, 250, 0, 50, 0, isQueued=1)
        dType.SetPTPJointParams(api, 200, 200, 200, 200,
                                200, 200, 200, 200, isQueued=1)
        dType.SetPTPCommonParams(api, 100, 100, isQueued=1)

        # Async Home
        dType.SetHOMECmd(api, temp=0, isQueued=1)
        dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVLXYZMode, 0, 0, 0, 0, 1)

        moveX = 0
        moveY = 0
        moveZ = 10
        moveFlag = -1
        pos = dType.GetPose(api)
        x = pos[0]
        y = pos[1]
        z = pos[2]
        rHead = pos[3]
        while (True):
            moveFlag *= -1
            for i in range(5):
                dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVLXYZMode, x + moveX, y + moveY, z + moveZ, rHead, 1)
                moveX += 10 * moveFlag
                dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVLXYZMode, x + moveX, y + moveY, z + moveZ, rHead, 1)
                dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVLXYZMode, x + moveX, y + moveY, z, rHead, 1)

    dType.SetQueuedCmdStopExec(api)

    # Disconnect Dobot
    dType.DisconnectDobot(api)
