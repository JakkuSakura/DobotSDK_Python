import sys, threading, time
import MutiDobotDllType as dType

api = dType.load()


def start_com(dobotId):
    pos = [250, 0, 30, 0]
    flag = 1
    flagb = 1
    flagc = 1

    print("Start dobot motion:dobotId =", dobotId)
    dType.SetQueuedCmdStartExec(api, dobotId)
    while (True):
        if flag > 0:
            pos[1] = 20
        else:
            pos[1] = -20

        if flagb > 0:
            pos[2] = 20
        else:
            pos[2] = -20

        dType.SetPTPCmd(api, dobotId, 1, pos[0], pos[1], pos[2], pos[3])

        flag = flag * -1
        flagc = flagc + 1
        if flagc % 3 == 0:
            flagb = flagb * -1

        dType.dSleep(100)


maxDobotConnectCount = 20
comlist = ["COM3", "COM5"]
if __name__ == '__main__':
    threads = []
    print("Start search dobot, count:", maxDobotConnectCount)
    # for i in range(0, maxDobotConnectCount):
    #    result = dType.ConnectDobot(api, "",115200)
    for i in comlist:
        result = dType.ConnectDobot(api, i, 115200)
        if result[0] == 0:
            print("Connect success: dobotid =", result[3])
            t1 = threading.Thread(target=start_com, args=(result[3],))
            threads.append(t1)
            t1.setDaemon(True)
            t1.start()

    for t in threads:
        t.join()

    dType.DobotExec(api)
