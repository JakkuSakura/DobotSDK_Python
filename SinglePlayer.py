import math
import threading
import time

import DobotAPI
from DobotControl import DobotControl


class Settings:
    HOME_BASE = (210.15174865722656, 60.31856918334961, 20.25244903564453)

    RIGHT_GET_DIS = 26

    RIGHT_PUT_DIS_X = 45
    RIGHT_PUT_DIS_Y = 25

    RIGHT_GET_BASE = (210.15174865722656, 60.31856918334961, -40.25244903564453)
    RIGHT_COLOR_BASE = (177.43209838867188, 175.313720703125, 14.28839111328125)
    RIGHT_PUT_BASE = (100.2291030883789, 250.73562622070312, -40.88656616210938)
    RIGHT_WASTE_POSE = (173, 102, -40)

    BLOCK_SIZE = 26
    DobotAPI.Debug = False
    RIGHT_DEBUG = False
    COLOR_PORT = DobotAPI.ColorPort.PORT_GP4
    HOME_INIT = False

    COLOR_SERIES = [0, 1, 2, 0, 1, 2]


def find(lst, a, default=-1):
    for i in range(len(lst)):
        if lst[i] == a:
            return i
    else:
        return default


class Robot(DobotControl):
    def __init__(self, index, COM):
        super().__init__(index, COM)
        self.counts = [0 for _ in range(4)]
        self.debug = False
        if Settings.HOME_INIT:
            self.home(Settings.HOME_BASE)

    def init(self, speed=400):
        super().init(speed)

    def user_init(self):
        self.dobot.SetColorSensor(1, Settings.COLOR_PORT)
        self.moveTo(*Settings.HOME_BASE)
        if Settings.RIGHT_DEBUG:
            self.moveTo(*Settings.RIGHT_GET_BASE)
            self.moveTo(*Settings.RIGHT_COLOR_BASE)
            self.moveTo(*Settings.RIGHT_WASTE_POSE)
            self.moveTo(*Settings.RIGHT_PUT_BASE)
            self.moveTo(z=30)
            self.moveTo(*Settings.HOME_BASE)
            pass

    def work(self):
        print("running right")
        for i in range(10):
            print("block", i)
            self.moveToGetPlace(i)
            self.capture()
            if Settings.COLOR_SERIES is not None and i < len(Settings.COLOR_SERIES):
                color = Settings.COLOR_SERIES[i]
            else:
                self.gotoColor()
                color = self.readColorForTimes(times=10)
            self.moveToPutPlace(color)
            self.release(down=5, up=10)

    def moveToGetPlace(self, i):
        pose = self.getGetPose(i)
        nowPose = self.dobot.GetPose()[:3]
        if nowPose[2] < 10 and nowPose[0] < Settings.RIGHT_COLOR_BASE[0]:
            self.moveTo(z=30)
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

    def release(self, down=0, up=0):
        self.moveInc(dz=-down)
        self.unsuck()
        time.sleep(0.1)
        self.moveInc(dz=up)

    def gotoColor(self):
        pose = self.dobot.GetPose()
        if pose[2] < Settings.RIGHT_COLOR_BASE[2]:
            self.moveTo(z=Settings.RIGHT_COLOR_BASE[2])
        else:
            self.moveTo(*Settings.RIGHT_COLOR_BASE[:2])
        to = list(Settings.RIGHT_COLOR_BASE)
        self.moveTo(*to)

    def capture(self, down=10, up=0):
        self.moveInc(dz=-down)
        self.suck()
        time.sleep(0.1)
        self.moveInc(dz=up)

    def moveToPutPlace(self, color):
        pose = list(Settings.RIGHT_PUT_BASE)
        if type(color) == int:
            index = color
            if self.dobot.GetPose()[2] < 30:
                self.moveTo(z=30)
        else:
            index = find(color, 1, -1)

        if index >= 0:
            pose[2] += self.counts[index] * Settings.BLOCK_SIZE
            self.counts[index] += 1

            index = 3 - index
            pose[0] -= Settings.RIGHT_PUT_DIS_X * index
            pose[1] -= Settings.RIGHT_PUT_DIS_Y * index
        else:  # failed to specify color
            pose = list(Settings.RIGHT_WASTE_POSE)
            self.counts[3] += 1
            print("throw block", self.counts[3])
            self.moveTo(z=pose[2] + 40)
            self.moveTo(*pose[:2])
            return

        print("put", self.counts)
        if pose[2] < Settings.RIGHT_COLOR_BASE[2]:
            self.moveTo(*pose[:2])
        else:
            self.moveTo(z=pose[2])

        self.moveTo(*pose)

    def blockComes(self):
        return 1 in self.getColor()

    def getColor(self):
        return self.dobot.GetColorSensor()

    @staticmethod
    def getGetPose(i):
        x = i // 5
        y = i % 5

        pose = list(Settings.RIGHT_GET_BASE)
        pose[0] += x * Settings.RIGHT_GET_DIS
        pose[1] -= y * Settings.RIGHT_GET_DIS

        return pose


if __name__ == '__main__':
    right = Robot(0, Robot.search()[0])
    right.start()
    right.join()
