import time

import DobotAPI
import DobotTypes
from DobotControl import DobotControl, find_color_index, color_exists


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
    DobotAPI.OutPutFlag = False
    RIGHT_DEBUG = True
    COLOR_PORT = DobotTypes.ColorPort.PORT_GP2
    HOME_INIT = False
    RIGHT_PUT_LIMIT = 8
    R = 0
    G = 1
    B = 2
    color_dic = ['R', 'G', 'B']

    # COLOR_SERIES = [G, B, G, R, R, R, B, B, R, B]
    COLOR_SERIES = None

    BLOCKS_ORDINARY = [11, 18] + list(range(10))


class Robot(DobotControl):
    def __init__(self):
        super().__init__()
        self.counts = [0 for _ in range(4)]
        self.debug = False
        self.speed = 800
        self.acc = 500
        if Settings.HOME_INIT:
            self.reset_zero(Settings.HOME_BASE)

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
        for i in range(12):
            print("block", i)
            self.moveToGetPlace(Settings.BLOCKS_ORDINARY[i], down=10)
            self.capture(down=0, up=15)
            self.gotoColor()
            if Settings.COLOR_SERIES is not None and i < len(Settings.COLOR_SERIES):
                color = Settings.COLOR_SERIES[i]
            else:
                color = self.readColor(times=10, default=(0, 1, 0))
            self.moveToPutPlace(color, down=5)
            self.release(down=0, up=0)

    def moveToGetPlace(self, i, down=0):
        pose = self.getGetPose(i)
        nowPose = self.dobot.GetPose()[:3]
        if nowPose[2] < 10 and nowPose[2] < Settings.RIGHT_COLOR_BASE[2]:
            self.moveTo(z=Settings.RIGHT_COLOR_BASE[2])
        self.moveTo(*pose[:2])
        pose[2] -= down
        self.moveTo(*pose)

    def readColor(self, times=1, default=(0, 0, 0)):
        for _ in range(times):
            color = self.getColor()
            if color_exists(color):
                hz = 500
                for i in range(3):
                    hz += 200
                    if color[i]:
                        break
                import winsound
                winsound.Beep(hz, 300)
                print("color ", color)
                return color
        else:
            print("color ", default)
            return default

    def capture(self, down=15, up=5):
        self.moveInc(dz=-down)
        self.suck()
        time.sleep(0.1)
        self.moveInc(dz=up)

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

    def moveToPutPlace(self, color, down):
        now_pose = self.dobot.GetPose()
        if type(color) == tuple or type(color) == list:
            index = find_color_index(color, -1)
        else:
            index = color
            if now_pose[2] < 60:
                self.moveTo(z=60)

        if index < 0:
            target_pose = list(Settings.RIGHT_WASTE_POSE)
            self.counts[3] += 1
            print("throw block", self.counts[3])
            self.moveTo(z=target_pose[2] + 80)
            self.moveTo(*target_pose[:2])
            return

        target_pose = self.getPutPose(index)

        self.counts[index] += 1

        print("put", self.counts)
        should_height = self.getShouldHeight(index, target_pose[2])
        if now_pose[2] < should_height:
            if target_pose[2] == should_height:
                self.moveTo(z=should_height)
            else:
                self.moveTo(z=should_height + Settings.BLOCK_SIZE)

        self.moveTo(*target_pose[:2], r=target_pose[3])
        target_pose[2] -= down
        self.moveTo(*target_pose)

    def getShouldHeight(self, index, target_pose_height):
        if index <= 0:
            return target_pose_height

        max_blocks = max(self.counts[:index])

        return max(target_pose_height, Settings.RIGHT_PUT_BASE[2] + max_blocks * Settings.BLOCK_SIZE)

    def getPutPose(self, index):
        target_pose = [0, 0, 0, -50]
        target_pose[0] = Settings.RIGHT_PUT_BASE[0] - Settings.RIGHT_PUT_DIS_X * index
        target_pose[1] = Settings.RIGHT_PUT_BASE[1] + Settings.RIGHT_PUT_DIS_Y * (
                self.counts[index] // Settings.RIGHT_PUT_LIMIT * 3 - 2)
        target_pose[2] = Settings.RIGHT_PUT_BASE[2] + (
                self.counts[index] % Settings.RIGHT_PUT_LIMIT + 1) * Settings.BLOCK_SIZE
        return target_pose

    def blockComes(self):
        return 1 in self.getColor()

    def getColor(self):
        return self.dobot.GetColorSensor()

    @staticmethod
    def getGetPose(i):
        x = i // 5
        y = i % 5
        z = i // 10

        pose = list(Settings.RIGHT_GET_BASE)
        pose[0] += x * Settings.RIGHT_GET_DIS
        pose[1] -= y * Settings.RIGHT_GET_DIS
        pose[2] += z * Settings.BLOCK_SIZE

        return pose


if __name__ == '__main__':
    right = Robot()
    right.connect(Robot.search()[0])
    right.run()
