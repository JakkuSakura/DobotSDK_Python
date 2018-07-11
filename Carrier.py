import math
import threading
import time

import DobotAPI
from DobotControl import DobotControl


class Settings:
    MOTOR_DIS = 25000
    DEFAULT_MOTO_SPEED = 28
    COM_LEFT = "COM5"
    COM_RIGHT = "COM6"

    HOME_BASE = (255, 0, 40)
    LEFT_GET_BASE = (54.873268127441406, 218.372314453125, -33.711280822753906)
    LEFT_PUT_BASE = (195.9342041015625, -4.1683244705200195, 20.778854370117188)

    LEFT_GET_DIS_Y = 28
    LEFT_GET_DIS_X = 24.5
    RIGHT_GET_DIS = 30

    RIGHT_TEMP_BASE = (152.51332092285156, 205.30560302734375, -5.84282684326172)
    RIGHT_GET_BASE = (260.6810607910156, -15, 27.20525360107422)
    RIGHT_FIX_Y = (25, 10, 10)

    RIGHT_WASTE_POSE = (173, 102, 10)
    RIGHT_PUT_BASE = (155, -190, -38)
    # BLOCK_SIZE = 25
    BLOCK_SIZE = 26
    DobotAPI.Debug = False

    MOTOR_PORT = DobotAPI.EMotorPort.EMOTOR_1
    MOVE_TIME = 1.5
    INFRARED_PORT = DobotAPI.InfraredPort.PORT_GP2
    COLOR_PORT = DobotAPI.ColorPort.PORT_GP4

    ENABLE_LEFT = True

    HOME_INIT = False


class Left(DobotControl):
    def __init__(self, index, COM, global_obj):
        super().__init__(index, COM)
        self.glb: Global = global_obj
        if Settings.HOME_INIT:
            self.home(Settings.HOME_BASE)

    def wait(self):
        if self.glb.finished_temp:
            print("wait taken")
            self.glb.waitTaken()

    def work(self):
        print("running left")
        for i in range(12):
            self.wait()
            self.getBlockLeft(i)
            self.wait()
            self.gotoPut()
            self.wait()
            self.stopMoto()
            self.release()
            self.wait()
            self.startMoto()
        self.glb.trans_moto_control = True

    def moveSpt(self, x, y, z, spt_times):
        now = self.dobot.GetPose()
        tup = (x, y, z)
        each_list = [(tup[x] - now[x]) / spt_times for x in range(3)]
        print(each_list)
        for i in range(spt_times):
            self.moveInc(*each_list)

    def getBlockLeft(self, index):
        lst = list(Settings.LEFT_GET_BASE)
        lst[0] -= Settings.LEFT_GET_DIS_X * (index // 4)
        lst[1] -= Settings.LEFT_GET_DIS_Y * (index % 4)
        self.moveTo(*lst)
        self.moveInc(dz=-15)
        self.suck()
        time.sleep(0.1)
        self.moveInc(dz=50)

    def startMoto(self, speed=Settings.DEFAULT_MOTO_SPEED):
        vel = float(speed) * 282.94212105225836
        self.dobot.SetEMotorEx(Settings.MOTOR_PORT, 1, int(vel), 1)

    def startMotoS(self, distance, speed=Settings.DEFAULT_MOTO_SPEED):
        vel = float(speed) * 282.94212105225836
        self.dobot.SetEMotorSEx(Settings.MOTOR_PORT, 1, int(vel), distance, 1)

        time.sleep(Settings.MOVE_TIME)

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

    def clean(self):
        self.stopMoto()


class Interal(threading.Thread):
    def __init__(self, right):
        super().__init__()
        self.right: Right = right

    def run(self):
        while self.right.glb.is_running:
            if self.right.blockComes():
                self.right.glb.is_taken = False
            time.sleep(0.1)


class Right(DobotControl):
    def __init__(self, index, COM, global_obj):
        super().__init__(index, COM)
        self.isWaitting = False
        self.glb: Global = global_obj
        self.counts = [0 for _ in range(4)]
        self.debug = False
        if Settings.HOME_INIT:
            self.home(Settings.HOME_BASE)

    def user_init(self):
        self.dobot.SetColorSensor(1, Settings.COLOR_PORT)
        self.dobot.SetInfraredSensor(1, Settings.INFRARED_PORT)
        # self.moveTo(*Settings.HOME_BASE)
        if self.debug:
            self.moveTo(*Settings.RIGHT_GET_BASE)
            self.moveTo(*Settings.RIGHT_TEMP_BASE)
            self.moveTo(*Settings.RIGHT_GET_BASE)
            self.moveTo(*Settings.RIGHT_PUT_BASE)
            self.moveTo(*Settings.RIGHT_GET_BASE)
            self.moveTo(*Settings.RIGHT_WASTE_POSE)
            self.moveTo(*Settings.RIGHT_GET_BASE)
            pass

    def work(self):
        print("running right")
        print("temp blocks")
        for i in range(0):
            print("temp block", i)
            self.moveToGetTempRight(i)
            self.capture()
            self.moveAboveGetPlaceRight()

            color = self.readColorForTimes(times=10)
            self.moveToPutRight(color)
            self.release(down=5, up=10)
        self.glb.finished_temp = True
        inter = Interal(self)
        inter.start()
        print("left zone blocks")
        for i in range(12):
            self.moveAboveGetPlaceRight()
            self.glb.awaitting = True
            self.waitComes()
            self.glb.stopMoto()
            color = self.readColorForTimes(10)
            index = int(math.fabs(color.find(1))) # default 1

            self.moveAboveGetPlaceRight(fix=Settings.RIGHT_FIX_Y[index])
            self.capture(up=30)
            self.glb.is_taken = True
            self.glb.awaitting = False
            self.moveToPutRight(color)
            self.release(down=5, up=30)
            if self.glb.trans_moto_control and not self.blockComes():
                self.glb.startMoto()
        self.glb.is_running = False

    def moveToGetTempRight(self, i):
        pose = self.getTempPutPose(i)
        nowPose = self.dobot.GetPose()[:3]
        #  if the dobot is in the right area
        if nowPose[1] - 30 < Settings.RIGHT_GET_BASE[1]:
            if nowPose[2] < 60:
                self.moveTo(z=60)
            self.moveTo(*pose[:2])
            self.moveTo(*pose)
        else:
            if i < 4:  # the upper pile
                pass
            else:
                self.moveTo(*pose[:2])
            self.moveTo(*pose)

    def readColorForTimes(self, times):
        color = (0, 0, 0)
        for _ in range(times):
            color = self.getColor()
            if 1 in color:
                break

        hz = 500
        for i in range(3):
            hz += 200
            if color[i]:
                break
        import winsound
        winsound.Beep(hz, 300)
        color = self.getColor()
        print("color ", color)

        return color

    @staticmethod
    def getTempPutPose(i):
        lz = i // 4
        lx = i % 4 // 2
        ly = (i + 1) % 2
        pose = list(Settings.RIGHT_TEMP_BASE)
        tp = [lx, ly, lz]
        for j in range(3):
            pose[j] -= Settings.RIGHT_GET_DIS * tp[j]
        return pose

    def release(self, down=0, up=0):
        self.moveInc(dz=-down)
        self.unsuck()
        time.sleep(0.1)
        self.moveInc(dz=up)

    def moveAboveGetPlaceRight(self, fix=0):
        pose = self.dobot.GetPose()
        if pose[2] < Settings.RIGHT_GET_BASE[2]:
            self.moveTo(z=Settings.RIGHT_GET_BASE[2])
        else:
            self.moveTo(*Settings.RIGHT_GET_BASE[:2])
        to = list(Settings.RIGHT_GET_BASE)
        to[1] += fix
        self.moveTo(*to)

    def capture(self, down=15, up=0):
        self.moveInc(dz=-down)
        self.suck()
        time.sleep(0.1)
        self.moveInc(dz=up)

    def moveToPutRight(self, color):

        pose = list(Settings.RIGHT_PUT_BASE)
        if color[0] == 1:  # red
            pose[1] -= Settings.BLOCK_SIZE * 2.2
        elif color[1] == 1:  # green
            pose[1] -= Settings.BLOCK_SIZE * 1.1
            pose[0] -= Settings.BLOCK_SIZE + 30
        elif color[2] == 1:  # blue
            pose[0] -= (Settings.BLOCK_SIZE + 30) * 2
        else:  # failed to specify color
            pose = list(Settings.RIGHT_WASTE_POSE)
            self.counts[3] += 1
            print("throw block", self.counts[3])
            self.moveTo(z=pose[2] + 40)
            self.moveTo(*pose[:2])
            return

        for x in range(3):
            if color[x]:
                pose[2] += self.counts[x] * Settings.BLOCK_SIZE
                self.counts[x] += 1
                break

        print("put", self.counts)
        if pose[2] < Settings.RIGHT_GET_BASE[2]:
            self.moveTo(*pose[:2])
        else:
            self.moveTo(z=pose[2])

        self.moveTo(*pose)

    def waitComes(self):
        print("waiting block comes")
        self.isWaitting = True
        while self.glb.is_running and not self.blockComes():
            time.sleep(0.01)
        self.isWaitting = False

    def blockComes(self):
        return 1 in self.getColor()

    def getColor(self):
        return self.dobot.GetColorSensor()


class Global:

    def __init__(self):
        self.trans_moto_control = True

        self.left = Left(0, Settings.COM_LEFT, self) if Settings.ENABLE_LEFT else None
        self.right = Right(1, Settings.COM_RIGHT, self)
        self.dobots = [self.left, self.right]
        self.sent = False
        self.is_taken = True
        self.coming = False
        self.awaitting = False
        self.is_running = True
        self.finished_temp = False

    def waitTaken(self):
        while not self.is_taken or self.awaitting:
            time.sleep(0.01)

    def stopMoto(self):
        if self.left is not None and self.left.isOk():
            self.left.stopMoto()

    def blockComes(self):
        if self.left is not None and self.right.isOk():
            return self.right.blockComes()
        return False

    def startMoto(self):
        if self.left is not None and self.left.isOk():
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
