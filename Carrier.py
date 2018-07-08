import threading
import time

import DobotAPI
from DobotControl import DobotControl


class Settings:
    INFRARED_PORT = DobotAPI.InfraredPort.PORT_GP2
    MOTOR_DIS = 10000
    DEFAULT_MOTO_SPEED = 50
    COM_LEFT = "COM5"
    COM_RIGHT = "COM6"

    HOME_BASE = (255, 0, 40)
    LEFT_GET_BASE = (89.71346282958984, 280.5124816894531, -36.28629684448242)
    LEFT_PUT_BASE = (253.9342041015625, -4.1683244705200195, 20.778854370117188)

    LEFT_GET_DIS = 30
    RIGHT_GET_DIS = 30

    RIGHT_TEMP_BASE = (147.51332092285156, 210.30560302734375, -10.84282684326172)
    RIGHT_GET_BASE = (255.6810607910156, -10, 27.20525360107422)
    RIGHT_FIX_Y = 10

    RIGHT_PUT_BASE = (155, -200, -38)
    BLOCK_SIZE = 25
    DobotAPI.Debug = False

    MOTOR_PORT = DobotAPI.EMotorPort.EMotor_1

    COLOR_PORT = DobotAPI.ColorPort.PORT_GP4


class Left(DobotControl):
    def __init__(self, index, COM, global_obj):
        super().__init__(index, COM)
        self.glb = global_obj

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
            self.gotoPut()
            self.stopMoto()
            self.release()
            self.startMotoS(Settings.MOTOR_DIS)
        self.glb.trans_moto_control = True

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
        self.moveTo(*lst)
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
        vel = float(speed) * 282.94212105225836
        self.dobot.SetEMotorSEx(Settings.MOTOR_PORT, 1, int(vel), distance, 1)
        time.sleep(1)

    def stopMoto(self):
        self.dobot.SetEMotorEx(Settings.MOTOR_PORT, 0, 0, 1)

    def gotoPut(self):
        l = list(Settings.LEFT_PUT_BASE)
        l[2] += 10
        self.moveTo(*l)
        self.moveInc(dz=-10)

    def release(self):
        time.sleep(0.1)
        self.unsuck()
        time.sleep(0.1)
        self.moveInc(dz=20)


class Right(DobotControl):
    def __init__(self, index, COM, global_obj):
        super().__init__(index, COM)
        self.isWaitting = False
        self.last_color = (0, 0, 0)
        self.glb = global_obj
        self.counts = [0 for _ in range(4)]

    def init(self):
        super().init()
        self.dobot.SetColorSensor(1, Settings.COLOR_PORT)
        self.dobot.SetInfraredSensor(1, 1)
        self.moveTo(*Settings.HOME_BASE)

    def work(self):
        print("running right")
        print("temp blocks")
        for i in range(8):
            print("temp block", i)
            lz = i // 4
            lx = i % 4 // 2
            ly = i % 2
            pose = list(Settings.RIGHT_TEMP_BASE)
            tp = [lx, ly, lz]
            for j in range(3):
                pose[j] -= Settings.RIGHT_GET_DIS * tp[j]
            self.moveTo(*pose)
            self.capture()
            self.moveAboveGetPlaceRight()
            time.sleep(0.3)

            self.last_color = (0, 0, 0)
            for _ in range(10):
                self.getColor()
                if 1 in self.last_color:
                    break

            self.moveToRight()
            self.release()

            pos = self.dobot.GetPose()
            if pos[2] <= 20:
                pos[2] = 20
            self.moveTo(*pos[:3])
            self.moveAboveGetPlaceRight()

        print("left zone blocks")
        for i in range(12):
            self.moveAboveGetPlaceRight()
            self.waitComes()
            self.glb.stopMoto()
            self.capture()
            self.moveToRight()
            self.release()
            if self.glb.trans_moto_control and not self.blockComes():
                self.glb.startMoto()
        self.glb.running = False

    def release(self):
        time.sleep(0.1)
        self.unsuck()
        time.sleep(0.1)
        self.moveInc(dz=50)

    def moveAboveGetPlaceRight(self):
        self.moveTo(*Settings.RIGHT_GET_BASE)

    def capture(self):
        self.moveInc(dz=-15)
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
        # elif self.last_color[2] == 1:  # blue
        else:
            lst[0] -= (Settings.BLOCK_SIZE + 30) * 2

        lst[2] += 60

        for x in range(3):
            if self.last_color[x]:
                lst[2] += self.counts[x] * Settings.BLOCK_SIZE
                self.counts[x] += 1
        else:
            lst[2] += self.counts[3] * Settings.BLOCK_SIZE
            self.counts[3] += 1
        print("put", self.counts)
        self.moveTo(*lst)
        self.moveInc(dz=-60)

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
        # self.left = Left(0, Settings.COM_LEFT, self)
        self.trans_moto_control = True
        self.left = None
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
            if e is not None and e.isOk():
                e.clean()

    def adjust(self):
        if self.right is not None:
            if self.right.isWaitting:
                self.right.moveAboveGetPlaceRight()


if __name__ == '__main__':
    glb = Global()
    try:
        glb.run()
    finally:
        glb.clean()
