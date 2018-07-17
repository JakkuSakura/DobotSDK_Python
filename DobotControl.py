import sys
from threading import Thread

import DobotAPI
import DobotTypes
import getDisMap
from DobotSession import DobotSession


class DobotControl(Thread):
    first_init = True
    root_session = DobotSession('', '')
    _searched = []

    @staticmethod
    def search():
        if DobotControl.first_init:
            DobotControl._searched = DobotControl.root_session.SearchDobot()
            print(DobotControl._searched)
            DobotControl.first_init = False
        return DobotControl._searched

    def __init__(self):
        super().__init__()
        self.addr = ""

        self.connect_state = -1
        self.dobot: DobotSession = None
        self.speed = 200
        self.acc = 200
        self.connect_state = -1
        self.device_version = "UnkownVersion"

    def user_init(self):
        pass

    def setAddr(self, addr):
        if addr not in DobotControl.search():
            raise Exception("Cannot find port", addr)
        self.addr = addr

    def connect(self):
        if self.isOk():
            return
        self.dobot = DobotSession()
        self.connect_state = self.dobot.ConnectDobot(self.addr)[0]
        print(self.addr, "Connect status:", DobotTypes.CONNECT_RESULT[self.connect_state])
        self.device_version = '%d.%d.%d' % tuple(self.dobot.GetDeviceVersion())
        if self.device_version != "3.2.2" and self.device_version != "3.5.0":
            print("Device version may be not supported:", self.device_version, self, file=sys.stderr)

    def init(self):
        if not self.isOk():
            raise Exception("You should connect dobot successfully before you init it", self.addr)
        print("Initing dobot", self.addr)
        DobotAPI.GetPose(self.dobot.api)
        self.dobot.GetPose()
        self.dobot.ClearAllAlarmsState()
        self.dobot.SetQueuedCmdStopExec()
        self.dobot.SetQueuedCmdClear()
        self.dobot.SetQueuedCmdStartExec()
        self.dobot.SetPTPJointParamsEx(self.speed, self.acc, self.speed, self.acc, self.speed, self.acc, self.speed,
                                       self.acc, 1)
        self.dobot.SetPTPCoordinateParams(self.speed, self.acc, self.speed, self.acc, 1)
        self.dobot.SetPTPJumpParamsEx(10, 170, 1)
        self.dobot.SetPTPCommonParamsEx(self.speed, self.acc, 1)
        self.reset_pose()
        self.unsuck()
        self.user_init()

    def getAlarmState(self):
        return self.dobot.GetAlarmsState()

    def reset_zero(self, home_pose):
        if type(home_pose) == list:
            home_pose = tuple(home_pose)
        print("Resetting position", self.addr)
        self.moveTo(*home_pose)
        self.dobot.SetHOMEParams(*home_pose, 1)
        self.dobot.SetHOMECmdEx(temp=0, isQueued=1)

    def run(self):
        self.connect()
        if not self.isOk():
            return
        try:
            self.init()
            self.work()
        finally:
            self.clean()
        print("stopped", self.addr)

    def clean(self):
        """
           You shall mannully call this method
           :return:
        """
        print("auto cleaning", self.addr)
        self.unsuck()
        self.dobot.DisconnectDobot()

    def suck(self):
        return self.dobot.SetEndEffectorSuctionCupEx(1, 1, 1)

    def unsuck(self):
        return self.dobot.SetEndEffectorSuctionCupEx(1, 0, 1)

    def getDobot(self):
        return self.dobot

    def moveTo(self, x=None, y=None, z=None, r=None, straight=False):
        nowPos = self.dobot.GetPose()
        x = float(x) if x is not None else nowPos[0]
        y = float(y) if y is not None else nowPos[1]
        z = float(z) if z is not None else nowPos[2]
        r = float(r) if r is not None else nowPos[3]
        moveMode = DobotTypes.PTPMode.PTP_MOVL_XYZ_Mode if straight else DobotTypes.PTPMode.PTP_MOVJ_XYZ_Mode
        print(self.addr, "move to", x, y, z, r)
        self.dobot.SetPTPCmdEx(moveMode, x, y, z, r, 1)

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

    def isOk(self):
        return self.connect_state == DobotTypes.DobotConnect.DobotConnect_Successfully

    def work(self):
        pass

    def startMoto(self, port, speed):
        if speed <= 0:
            raise Exception("You should call stopMoto")
        vel = speed * 282.94212105225836
        self.dobot.SetEMotorEx(port, 1, int(vel), 1)

    def startMotoS(self, port, speed, distance_ms):
        if speed <= 0 or distance_ms <= 0:
            raise Exception("You should call stopMoto")
        vel = speed * 282.94212105225836
        distance = getDisMap.get_dis_tick(distance_ms)
        self.dobot.SetEMotorSEx(port, 1, int(vel), int(distance), 1)

    def stopMoto(self, port):
        self.dobot.SetEMotorEx(port, 0, 0, 1)

    def reset_pose(self):
        if self.dobot.SetLostStepCmd():
            self.dobot.ResetPose(0, 0, 0)

    def __str__(self):
        return "Dobot[{addr}]".format(addr=self.addr)


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
