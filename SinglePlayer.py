import time

import DobotAPI
from DobotControl import DobotControl


class Settings:
    HOME_BASE = (210.15174865722656, 60.31856918334961, 20.25244903564453)

    RIGHT_GET_DIS = 25

    RIGHT_PUT_DIS_X = 45
    RIGHT_PUT_DIS_Y = 25

    RIGHT_GET_BASE = (210.15174865722656, 60.31856918334961, -35)
    RIGHT_COLOR_BASE = (177.43209838867188, 175.313720703125, 14.28839111328125)
    RIGHT_PUT_BASE = (100.2291030883789, 250.73562622070312, -35)
    RIGHT_WASTE_POSE = (173, 102, -40)

    BLOCK_SIZE = 26
    DobotAPI.Debug = False
    RIGHT_DEBUG = True
    COLOR_PORT = DobotAPI.ColorPort.PORT_GP4
    HOME_INIT = False

    R = 0
    G = 1
    B = 2
    color_dic = ['R', 'G', 'B']

    COLOR_SERIES = [G, B, G, R, R, R, B, B, R, B]


def find(lst, a, default=-1):
    for i in range(len(lst)):
        if lst[i] == a:
            return i
    else:
        return default


class Robot(DobotControl):
    def __init__(self, index, addr):
        super().__init__(index, addr)
        self.counts = [0 for _ in range(4)]
        self.debug = False
        if Settings.HOME_INIT:
            self.reset_zero(Settings.HOME_BASE)

    def init(self, speed=600):
        super().init(speed)

    def user_init(self):
        self.dobot.SetColorSensor(1, Settings.COLOR_PORT)
        self.moveTo(*Settings.HOME_BASE)
        if Settings.RIGHT_DEBUG:
            for i in range(10):
                self.moveToGetPlace(i)
                if Settings.COLOR_SERIES is not None and i < len(Settings.COLOR_SERIES):
                    print(Settings.color_dic[Settings.COLOR_SERIES[i]])

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
            self.moveToGetPlace(i, down=10)
            self.gotoColor()
            if Settings.COLOR_SERIES is not None and i < len(Settings.COLOR_SERIES):
                color = Settings.COLOR_SERIES[i]
            else:
                color = self.readColorForTimes(times=10)
            self.moveToPutPlace(color, down=5, up=10)

    def moveToGetPlace(self, i, down=0):
        pose = self.getGetPose(i)
        nowPose = self.dobot.GetPose()[:3]
        if nowPose[2] < 10 and nowPose[2] < Settings.RIGHT_COLOR_BASE[2]:
            self.moveTo(z=Settings.RIGHT_COLOR_BASE[2])
        self.moveTo(*pose[:2])
        pose[2] -= down
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

    def gotoColor(self):
        pose = self.dobot.GetPose()
        if pose[2] < Settings.RIGHT_COLOR_BASE[2]:
            self.moveTo(z=Settings.RIGHT_COLOR_BASE[2])
        else:
            self.moveTo(*Settings.RIGHT_COLOR_BASE[:2])
        to = list(Settings.RIGHT_COLOR_BASE)
        self.moveTo(*to)


    def moveToPutPlace(self, color, down=0, up=0):
        pose = list(Settings.RIGHT_PUT_BASE)
        if type(color) == int:
            index = color
            if self.dobot.GetPose()[2] < 30:
                self.moveTo(z=Settings.RIGHT_COLOR_BASE[2])
        else:
            index = find(color, 1, -1)

        if index >= 0:
            pose[2] += self.counts[index] * Settings.BLOCK_SIZE
            self.counts[index] += 1
            print("put", self.counts)

            index = 3 - index
            pose[0] -= Settings.RIGHT_PUT_DIS_X * index
            pose[1] -= Settings.RIGHT_PUT_DIS_Y * index
        else:  # failed to specify color
            pose = list(Settings.RIGHT_WASTE_POSE)
            self.counts[3] += 1
            print("throw block", self.counts[3])

        if pose[2] < Settings.RIGHT_COLOR_BASE[2]:
            self.moveTo(*pose[:2])
        else:
            self.moveTo(z=pose[2])

        pose[2] -= down
        self.moveTo(*pose)
        self.unsuck()
        time.sleep(0.1)
        self.moveInc(dz=up)

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
    right.run()
