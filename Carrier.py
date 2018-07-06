import threading
import time

import DobotAPI
from DobotControl import DobotControl


class Settings:
    INFRARED_PORT = DobotAPI.InfraredPort.PORT_GP2
    MOTOR_DIS = 15000
    DEFAULT_MOTO_SPEED = 50
    COM_LEFT = "COM6"
    COM_RIGHT = "COM5"

    HOME_BASE = (255, 0, 40)
    LEFT_GET_BASE = (89.71346282958984, 280.5124816894531, -36.28629684448242)
    LEFT_PUT_BASE = (245.9342041015625, -4.1683244705200195, 20.778854370117188)

    LEFT_GET_DIS = 30

    RIGHT_GET_BASE = (255.6810607910156, -15, 25.20525360107422)
    RIGHT_FIX_Y = 10

    RIGHT_PUT_BASE = (155, -200, -33)
    BLOCK_SIZE = 25
    DobotAPI.Debug = False

    MOTOR_PORT = DobotAPI.EMotor

    COLOR_PORT = DobotAPI.ColorPort.PORT_GP4


class Left(DobotControl):
    def __init__(self, index, COM, glb):
        super().__init__(index, COM)
        self.glb = glb

    def waitForCome(self):
        if self.glb.coming:
            while self.glb.running and not self.glb.isTaken:
                time.sleep(0.01)
            while self.glb.running and self.glb.isTaken:
                time.sleep(0.01)
            self.glb.coming = False

    def work(self):
        print("running left")
        for i in range(12):
            self.getBlockLeft(i)
            self.waitForCome()
            self.gotoPut()
            self.waitForCome()
            self.stopMoto()
            self.release()
            self.waitForCome()
            self.startMotoS(Settings.MOTOR_DIS)
            self.waitForCome()
        self.glb.sent = True

    def moveSpt(self, x, y, z, spt_times):
        now = self.dobot.GetPose()
        tup = (x, y, z)
        each_list = [(tup[x] - now[x]) / spt_times for x in range(3)]
        print(each_list)
        for i in range(spt_times):
            self.moveInc(*each_list)
            self.waitForCome()

    def getBlockLeft(self, index):
        lst = list(Settings.LEFT_GET_BASE)
        lst[0] -= Settings.LEFT_GET_DIS * (index // 4)
        lst[1] -= Settings.LEFT_GET_DIS * (index % 4)
        self.moveSpt(*lst, 2)
        self.moveInc(dz=-15)
        self.waitForCome()
        self.suck()
        time.sleep(0.1)
        self.moveInc(dz=50)
        self.waitForCome()

    def startMoto(self, speed=Settings.DEFAULT_MOTO_SPEED):
        vel = float(speed) * 282.94212105225836
        self.dobot.SetEMotorEx(Settings.MOTOR_PORT, 1, int(vel), 1)

    def startMotoS(self, distance, speed=Settings.DEFAULT_MOTO_SPEED):
        raise Exception("Do not use this")
        vel = float(speed) * 282.94212105225836
        self.dobot.SetEMotorSEx(Settings.MOTOR_PORT, 1, int(vel), distance, 1)

    def stopMoto(self):
        self.dobot.SetEMotorEx(Settings.MOTOR_PORT, 0, 0, 1)

    def gotoPut(self):
        l = list(Settings.LEFT_PUT_BASE)
        l[2] += 10
        self.moveSpt(*l, 2)

    def release(self):
        time.sleep(0.1)
        self.unsuck()
        time.sleep(0.1)
        self.moveInc(dz=20)


class Right(DobotControl):
    def __init__(self, index, COM, glb):
        super().__init__(index, COM)
        self.isWaitting = False
        self.last_color = None
        self.glb = glb
        self.counts = [0 for _ in range(3)]

    def init(self):
        super().init()
        self.dobot.SetColorSensor(1, Settings.COLOR_PORT)
        self.dobot.SetInfraredSensor(1, 1)

        threading.Thread(target=self.checkBlockComing).start()

    def work(self):
        print("running right")
        for i in range(12):
            self.moveAboveRight()
            self.waitComes()
            self.glb.isTaken = False
            self.glb.stopMoto()
            self.getBlock()
            self.glb.isTaken = True
            self.moveToRight()
            self.release()
            if self.glb.sent and not self.blockComes():
                self.glb.startMoto()
        self.glb.running = False

    def release(self):
        time.sleep(0.1)
        self.unsuck()
        time.sleep(0.1)
        self.moveInc(dz=50)

    def moveAboveRight(self):
        lst = list(Settings.RIGHT_GET_BASE)
        if self.glb.sent:
            lst[1] += Settings.RIGHT_FIX_Y
        self.moveTo(*lst)

    def getBlock(self):
        self.moveInc(dz=-20)
        self.suck()
        time.sleep(0.1)
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
        print("waiting block comes")
        self.isWaitting = True
        while self.glb.running and not self.blockComes():
            time.sleep(0.01)

        self.isWaitting = False
        print("color ", self.last_color)

    def blockComes(self):
        # return self.dobot.GetInfraredSensor(1) == 1
        return 1 in self.getColor()

    def checkBlockComing(self):
        while self.glb.running:
            coming = self.dobot.GetInfraredSensor(Settings.INFRARED_PORT)
            if coming:
                self.glb.coming = True
            time.sleep(0.01)

    def getColor(self):
        # self.moveTo(*Settings.RIGHT_COLOR_POSE)
        # while True:
        tmp = self.dobot.GetColorSensor()
        if 1 in tmp:
            self.last_color = tmp
        # if 1 in self.last_color:
        #     break
        # self.moveInc(dz=10)
        return tmp


class Global:

    def __init__(self):
        self.left = Left(0, Settings.COM_LEFT, self)
        self.right = Right(1, Settings.COM_RIGHT, self)
        self.dobots = [self.left, self.right]
        self.sent = False
        self.isTaken = True
        self.coming = False
        self.running = True

    def waitTaken(self):
        while not self.isTaken or self.blockComes():
            time.sleep(0.01)

    def stopMoto(self):
        if self.left.isOk():
            self.left.stopMoto()

    def blockComes(self):
        if self.right.isOk():
            return self.right.blockComes()
        return False

    def startMoto(self):
        if self.left.isOk():
            self.left.startMoto()

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

    def adjust(self):
        if self.right is not None:
            if self.right.isWaitting:
                self.right.moveAboveRight()


if __name__ == '__main__':
    glb = Global()
    try:
        glb.run()
    finally:
        glb.clean()
