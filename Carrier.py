import time
from threading import Thread
import DobotAPI


class Settings:
    MOTOR_DIS = 50
    DEFAULT_MOTO_SPEED = 50
    COM_LEFT = "COM6"
    COM_RIGHT = "COM5"

    HOME_BASE = (255, 0, 40)
    LEFT_GET_BASE = (122.18338012695312, 222.9822540283203, -44.83717727661133)
    LEFT_PUT_BASE = (191.9342041015625, -4.1683244705200195, 23.778854370117188)
    LEFT_CACHE_POSE = (183.4839630126953, 60.274253845214844, 29.90416717529297)

    RIGHT_GET_BASE = (255.5, -12.8, 26.85)
    RIGHT_PUT_BASE = (155, -200, -35)
    BLOCK_SIZE = 25
    DobotAPI.Debug = False


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
        self.dobot.SetCmdTimeout(300)
        self.connect_state = self.dobot.ConnectDobot(COM)[0]
        if COM not in DobotControl.search():
            print("Cannot find port", COM)
            return
        print(COM, "Connect status:", DobotAPI.CONNECT_RESULT[self.connect_state])

    def init(self):
        if self.connect_state != DobotAPI.DobotConnect.DobotConnect_Successfully:
            return
        print("Initing dobot", self.addr)
        self.dobot.ClearAllAlarmsState()
        self.dobot.SetQueuedCmdClear()
        self.dobot.SetQueuedCmdStartExec()
        self.dobot.SetPTPJointParams(200, 200, 200, 200, 200, 200, 200, 200, 1)
        self.dobot.SetPTPCoordinateParams(200, 200, 200, 200, 1)
        self.dobot.SetPTPJumpParams(10, 200, 1)
        self.dobot.SetPTPCommonParams(100, 100, 1)
        self.unsuck()
        self.moveInc(dz=50)
        self.dobot.SetHOMEParams(*Settings.HOME_BASE, 0, 1)
        # self.dobot.SetHOMECmdEx(temp=0, isQueued=1)

    def run(self):
        self.init()
        self.work()

    def clean(self):
        """
           You must mannully call this method
           :return:
        """
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


class Left(DobotControl):
    def __init__(self, index, COM, glb):
        super().__init__(index, COM)
        self.glb = glb

    def getBlockLeft(self, index):
        lst = list(Settings.LEFT_GET_BASE)
        lst[0] -= Settings.BLOCK_SIZE * (index // 4)
        lst[1] -= Settings.BLOCK_SIZE * (index % 4)
        self.moveTo(*lst)
        self.moveInc(dz=-20)
        self.suck()
        self.moveInc(dz=40)

    def moveToCache(self):
        return self.moveTo(*Settings.LEFT_CACHE_POSE)

    def startMoto(self, speed=Settings.DEFAULT_MOTO_SPEED):
        vel = float(speed) * 282.94212105225836
        self.dobot.SetEMotorEx(1, 1, int(vel), 1)

    def startMotoS(self, distance, speed=Settings.DEFAULT_MOTO_SPEED):
        vel = float(speed) * 282.94212105225836
        self.dobot.SetEMotorSEx(1, 1, int(vel), distance, 1)

    def stopMoto(self):
        self.dobot.SetEMotor(1, 0, 0, 1)

    def work(self):
        for i in range(12):
            self.getBlockLeft(i)
            if self.glb.blockComes():
                self.moveToCache()
            self.glb.waitTaken()
            self.stopMoto()
            self.realse()
            self.startMotoS(Settings.MOTOR_DIS)

    def realse(self):
        l = list(Settings.LEFT_PUT_BASE)
        l[2] += 10
        self.moveTo(*l)
        self.unsuck()
        self.moveInc(dz=20)


class Right(DobotControl):
    def __init__(self, index, COM, glb):
        super().__init__(index, COM)
        self.last_color = None
        self.dobot.SetColorSensor(1, DobotAPI.ColorPort.PORT_GP5)
        self.glb = glb
        self.counts = [0 for _ in range(3)]

    def work(self):
        for i in range(12):
            self.moveAboveRight()
            self.waitComes()
            self.glb.stopMoto()

            self.getBlockRight()
            self.moveToRight()
            self.unsuck()

    def moveAboveRight(self):
        self.moveTo(*Settings.RIGHT_GET_BASE)

    def getBlockRight(self):
        self.moveInc(dz=-20)
        self.suck()
        self.moveInc(dz=40)

    def moveToRight(self):
        lst = list(Settings.RIGHT_PUT_BASE)
        if self.last_color[0] == 1:  # red
            lst[1] -= Settings.BLOCK_SIZE * 2
        elif self.last_color[1] == 1:  # green
            lst[1] -= Settings.BLOCK_SIZE * 1
            lst[0] -= Settings.BLOCK_SIZE + 30
        elif self.last_color[2] == 1:  # blue
            lst[0] -= (Settings.BLOCK_SIZE + 30) * 2

        for x in range(3):
            if self.last_color[x]:
                lst[2] += self.counts[x] * Settings.BLOCK_SIZE
                self.counts[x] += self.last_color[x]
        print("put", self.counts)
        return self.moveTo(*lst)

    def waitComes(self):
        while not self.blockComes():
            time.sleep(0.01)
        self.last_color = self.getColor()
        print("comes", self.last_color)

    def blockComes(self):
        return 1 in self.getColor()

    def getColor(self):
        return self.dobot.GetColorSensor()


class Global:
    # global status
    isTaken = True

    def __init__(self):
        self.dobots = [Left(0, Settings.COM_LEFT, self), Right(1, Settings.COM_RIGHT, self)]

    def waitTaken(self):
        while not Global.isTaken:
            time.sleep(0.01)

    def stopMoto(self):
        if self.dobots[0].isOk():
            self.dobots[0].stopMoto()

    def blockComes(self):
        if self.dobots[1].isOk():
            return self.dobots[1].blockComes()
        return False

    def run(self):
        for e in self.dobots:
            e.start()

    def clean(self):
        for e in self.dobots:
            if e.isOk():
                e.clean()


if __name__ == '__main__':
    glb = Global()
    try:
        glb.run()
    except KeyboardInterrupt:
        glb.clean()
