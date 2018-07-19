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
        self.color_sensor = None
        self.pump = None

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
        self.pumpControl(1, 1)

    def blow(self):
        self.pumpControl(1, 0)

    def unsuck(self):
        self.pumpControl(0, 0)

    def pumpControl(self, enable, control):
        if self.pump:
            if self.pump[0] > 0:
                self.dobot.SetIODOEx(self.pump[0], enable, 1)

            if self.pump[1] > 0:
                self.dobot.SetIODOEx(self.pump[1], control, 1)
        else:
            self.dobot.SetEndEffectorGripperEx(enable, control, 1)

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

    def setColotSensor(self, output_port, input_left, input_right, enable=True):
        self.color_sensor = (output_port, input_left, input_right, enable)
        self.dobot.SetIOMultiplexingEx(output_port, DobotTypes.IOFunction.IOFunctionDO, 1)
        self.dobot.SetIOMultiplexingEx(input_left, DobotTypes.IOFunction.IOFunctionDI, 1)
        self.dobot.SetIOMultiplexingEx(input_right, DobotTypes.IOFunction.IOFunctionDI, 1)

        self.dobot.SetIODOEx(output_port, enable, 1)

    def getColorSensor(self):
        if self.color_sensor:
            left = self.dobot.GetIODI(self.color_sensor[1])[0]
            right = self.dobot.GetIODI(self.color_sensor[2])[0]

            if left == 0 and right == 0:
                return 1, 0, 0
            elif left == 1 and right == 0:
                return 0, 1, 0
            elif left == 0 and right == 1:
                return 0, 0, 1
            else:
                return 0, 0, 0
        else:
            return self.dobot.GetColorSensor()

    def setPump(self, power_port, control_port):
        self.pump = (power_port, control_port)
        self.dobot.SetIOMultiplexingEx(power_port, DobotTypes.IOFunction.IOFunctionDO, 1)
        self.dobot.SetIOMultiplexingEx(control_port, DobotTypes.IOFunction.IOFunctionDO, 1)

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


def ensure_color_index(tpl, default=-1):
    if type(tpl) == int:
        return tpl
    for i in range(len(tpl)):
        if color_exists(tpl[i]):
            return i
    else:
        return default


def ensure_color_tuple(index, default=(0, 0, 0)):
    if type(index) == tuple or type(index) == list:
        return tuple(index)
    if index < 0:
        return default
    lst = [0, 0, 0]
    lst[index] += 1
    return tuple(lst)
