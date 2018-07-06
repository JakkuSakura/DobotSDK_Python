import time

import DobotAPI
from DobotControl import DobotControl


class Settings:
    MOTOR_DIS = 50
    DEFAULT_MOTO_SPEED = 50
    COM_LEFT = "COM6"
    COM_RIGHT = "COM5"

    HOME_BASE = (255, 0, 40)
    LEFT_GET_BASE = (81.61100769042969, 274.7370300292969, -39.551631927490234)
    LEFT_PUT_BASE = (191.9342041015625, -4.1683244705200195, 23.778854370117188)
    LEFT_CACHE_POSE = (183.4839630126953, 60.274253845214844, 29.90416717529297)
    LEFT_GET_DIS = 30

    RIGHT_GET_BASE = (255.5, -12.8, 26.85)
    RIGHT_PUT_BASE = (155, -200, -35)
    BLOCK_SIZE = 25
    DobotAPI.Debug = False


class Left(DobotControl):
    def __init__(self, index, COM, glb):
        super().__init__(index, COM)
        self.glb = glb

    def getBlockLeft(self, index):
        lst = list(Settings.LEFT_GET_BASE)
        lst[0] -= Settings.LEFT_GET_DIS * (index // 4)
        lst[1] -= Settings.LEFT_GET_DIS * (index % 4)
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
        print("running left")
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
        self.glb = glb
        self.counts = [0 for _ in range(3)]

    def init(self):
        self.dobot.SetColorSensor(1, DobotAPI.ColorPort.PORT_GP5)

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
        return 1 in self.dobot.GetInfraredSensor(1)

    def getColor(self):
        return self.dobot.GetColorSensor()


class Global:
    isTaken = True

    def __init__(self):
        self.left = Left(0, Settings.COM_LEFT, self)
        self.right = Right(1, Settings.COM_RIGHT, self)
        self.dobots = [self.left, self.right]

    def waitTaken(self):
        while not Global.isTaken:
            time.sleep(0.01)

    def stopMoto(self):
        if self.left.isOk():
            self.left.stopMoto()

    def blockComes(self):
        if self.right.isOk():
            return self.right.blockComes()
        return False

    def run(self):
        for e in self.dobots:
            if e is not None and e.isOk():
                e.start()
        for e in self.dobots:
            if e is not None and e.isOk():
                e.join()

    def clean(self):
        print("cleaning")
        for e in self.dobots:
            if e.isOk():
                e.clean()


if __name__ == '__main__':
    glb = Global()
    try:
        glb.run()
    finally:
        glb.clean()
