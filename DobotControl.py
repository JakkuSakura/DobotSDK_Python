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

    def home(self, home_pose):
        print("Homing", self.addr)
        self.moveTo(*home_pose)
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

    def moveTo(self, x=None, y=None, z=None):
        nowPos = self.dobot.GetPose()
        x = x or nowPos[0]
        y = y or nowPos[1]
        z = z or nowPos[2]
        self.dobot.SetPTPCmdEx(DobotAPI.PTPMode.PTP_MOVJ_XYZ_Mode, x, y, z, 0, 1)

    def moveInc(self, dx=0, dy=0, dz=0):
        self.dobot.SetPTPCmdEx(DobotAPI.PTPMode.PTP_MOVJ_XYZ_INC_Mode, dx, dy, dz, 0, 1)

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
