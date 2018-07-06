from threading import Thread

import DobotAPI


class DobotControl(Thread):
    first_init = True
    root_session = DobotAPI.Session('', '')
    searched = []

    @staticmethod
    def search():
        if DobotControl.first_init:
            DobotControl.searched = DobotControl.root_session.SearchDobot()
            print(DobotControl.searched)
            DobotControl.first_init = False
        return DobotControl.searched

    def __init__(self, index, COM):
        super().__init__()
        self.addr = COM
        self.connect_state = None
        self.dobot = DobotAPI.Session(index)
        self.dobot.SetCmdTimeout(100)
        if COM not in DobotControl.search():
            print("Cannot find port", COM)
            return
        self.connect_state = self.dobot.ConnectDobot(COM)[0]
        print(COM, "Connect status:", DobotAPI.CONNECT_RESULT[self.connect_state])

    def init(self):
        print("Initing dobot", self.addr)
        self.dobot.ClearAllAlarmsState()
        self.dobot.SetQueuedCmdStopExec()
        self.dobot.SetQueuedCmdClear()
        self.dobot.SetQueuedCmdStartExec()
        self.dobot.SetPTPJointParams(200, 200, 200, 200, 200, 200, 200, 200, 1)
        self.dobot.SetPTPCoordinateParams(200, 200, 200, 200, 1)
        self.dobot.SetPTPJumpParams(10, 200, 1)
        self.dobot.SetPTPCommonParams(100, 100, 1)
        self.unsuck()

    def home(self, home_pose):
        print("Homing", self.addr)
        self.dobot.SetHOMEParams(*home_pose, 0, 1)
        self.dobot.SetHOMECmdEx(temp=0, isQueued=1)

    def run(self):
        if not self.isOk():
            return
        self.init()
        self.work()

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

    def moveTo(self, x, y, z):
        self.dobot.SetPTPCmdEx(DobotAPI.PTPMode.PTP_MOVJ_XYZ_Mode, x, y, z, 0, 1)

    def moveInc(self, dx=0, dy=0, dz=0):
        self.dobot.SetPTPCmdEx(DobotAPI.PTPMode.PTP_MOVJ_XYZ_INC_Mode, dx, dy, dz, 0, 1)

    def __str__(self):
        return "Dobot: %s %s" % (self.addr, self.connect_state)

    def isOk(self):
        return self.connect_state == DobotAPI.DobotConnect.DobotConnect_Successfully

    def work(self):
        pass
