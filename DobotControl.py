from threading import Thread

import DobotAPI
from DobotSession import DobotSession


class DobotControl(Thread):
    first_init = True
    root_session = DobotSession('', '')
    searched = []

    @staticmethod
    def search():
        if DobotControl.first_init:
            DobotControl.searched = DobotControl.root_session.SearchDobot()
            print(DobotControl.searched)
            DobotControl.first_init = False
        return DobotControl.searched

    def __init__(self, index, addr):
        super().__init__()
        self.addr = addr
        self.connect_state = None
        self.dobot = DobotSession(index)
        self.dobot.SetCmdTimeout(100)
        if addr not in DobotControl.search():
            print("Cannot find port", addr)
            return
        self.connect_state = self.dobot.ConnectDobot(addr)[0]
        print(addr, "Connect status:", DobotAPI.CONNECT_RESULT[self.connect_state])

    def user_init(self):
        pass

    def init(self, speed=400):
        print("Initing dobot", self.addr)
        DobotAPI.GetPose(self.dobot.api)
        self.dobot.GetPose()
        self.dobot.ClearAllAlarmsState()
        self.dobot.SetQueuedCmdStopExec()
        self.dobot.SetQueuedCmdClear()
        self.dobot.SetQueuedCmdStartExec()
        self.dobot.SetPTPJointParamsEx(speed, speed, speed, speed, speed, speed, speed, speed, 1)
        self.dobot.SetPTPCoordinateParams(speed, speed, speed, speed, 1)
        self.dobot.SetPTPJumpParamsEx(10, speed, 1)
        self.dobot.SetPTPCommonParamsEx(speed, speed, 1)
        self.unsuck()
        self.user_init()

    def reset_zero(self, home_pose):
        if type(home_pose) == list:
            home_pose = tuple(home_pose)
        print("Resetting position", self.addr)
        self.moveTo(*home_pose)
        self.dobot.SetHOMEParams(*home_pose, 0, 1)
        self.dobot.SetHOMECmdEx(temp=0, isQueued=1)

    def run(self):
        try:
            if not self.isOk():
                return
            self.init()
            self.work()
        finally:
            self.clean()

    def clean(self):
        """
           You must mannully call this method
           :return:
        """
        self.unsuck()
        self.dobot.SetQueuedCmdForceStopExec()
        self.dobot.DisconnectDobot()

    def suck(self):
        return self.dobot.SetEndEffectorSuctionCupEx(1, 1, 1)

    def unsuck(self):
        return self.dobot.SetEndEffectorSuctionCupEx(1, 0, 1)

    def getDobot(self):
        return self.dobot

    def moveTo(self, x=None, y=None, z=None, r=None, straight=False):
        nowPos = self.dobot.GetPose()
        x = x or nowPos[0]
        y = y or nowPos[1]
        z = z or nowPos[2]
        r = r or nowPos[3]
        moveMode = DobotAPI.PTPMode.PTP_MOVL_XYZ_Mode if straight else DobotAPI.PTPMode.PTP_MOVJ_XYZ_Mode
        print(self.addr, "move to", x, y, z, r)
        self.dobot.SetPTPCmdEx(moveMode, x, y, z, 0, 1)

    def moveInc(self, dx=0, dy=0, dz=0, dr=0, straight=False):
        nowPos = self.dobot.GetPose()
        self.moveTo(nowPos[0] + dx, nowPos[1] + dy, nowPos[2] + dz, nowPos[3] + dr, straight)

    def moveSpt(self, x, y, z, spt_times):
        now = self.dobot.GetPose()
        tup = (x, y, z)
        each_list = [(tup[x] - now[x]) / spt_times for x in range(3)]
        print(each_list)
        for i in range(spt_times):
            self.moveInc(*each_list)

    def __str__(self):
        return "Dobot: %s %s" % (self.addr, self.connect_state)

    def isOk(self):
        return self.connect_state == DobotAPI.DobotConnect.DobotConnect_Successfully

    def work(self):
        pass

    def startMoto(self, port, speed):
        vel = float(speed) * 282.94212105225836
        self.dobot.SetEMotorEx(port, 1, int(vel), 1)

    def startMotoS(self, port, distance, speed):
        vel = float(speed) * 282.94212105225836
        self.dobot.SetEMotorSEx(port, 1, int(vel), distance, 1)

    def stopMoto(self, port):
        self.dobot.SetEMotorEx(port, 0, 0, 1)

    def reset_pose(self):
        if self.dobot.SetLostStepCmd():
            self.dobot.ResetPose(0, 0, 0)


def color_exists(n):
    if type(n) == int:
        return n == 255 or n == 1
    else:
        for e in n:
            if color_exists(e):
                return True
        else:
            return False


def find_color_index(lst, default=-1):
    for i in range(len(lst)):
        if color_exists(lst[i]):
            return i
    else:
        return default
