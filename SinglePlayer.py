import time

import DobotAPI
import DobotTypes
from DobotControl import DobotControl, ensure_color_index, color_exists


class Settings:
    HOME_BASE = (210.15174865722656, 0, 20.25244903564453)

    RIGHT_GET_DIS_X = 25
    RIGHT_GET_DIS_Y = 25

    RIGHT_PUT_DIS_X = 55
    RIGHT_PUT_DIS_Y = 30

    RIGHT_GET_BASE = (208.95521545410156, 67.0932502746582, -40.081871032714844)
    RIGHT_COLOR_BASE = (178.8518829345703, 174.12266540527344, 7.488426208496094)
    RIGHT_PUT_BASE = (115.4960708618164, 145.10301208496094, -60.67840576171875)
    RIGHT_WASTE_POSE = (186.422119140625, 232.47254943847656, 15.287193298339844)

    BLOCK_SIZE = 28

    DobotAPI.OutPutFlag = False
    RIGHT_DEBUG = False
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

    def user_init(self):
        self.dobot.SetColorSensor(1, Settings.COLOR_PORT)
        if Settings.HOME_INIT:
            self.reset_zero(Settings.HOME_BASE)
        if Settings.RIGHT_DEBUG:
            for i in range(2):
                self.moveToGetPlace(Settings.BLOCKS_ORDINARY[i], -1)
                if Settings.COLOR_SERIES is not None and i < len(Settings.COLOR_SERIES):
                    print(Settings.color_dic[Settings.COLOR_SERIES[i]])

            self.gotoColor()
            self.moveTo(*Settings.RIGHT_WASTE_POSE)
            self.moveToPutPlace((1, 0, 0))
            self.moveToPutPlace((0, 1, 0))
            self.moveToPutPlace((0, 0, 1))
            self.counts = [0 for _ in range(4)]

            self.moveTo(z=30)
            self.moveTo(*Settings.HOME_BASE)
            pass

    def work(self):

        color = (0, 0, 0)
        print("running right")
        for i in range(0, 12):
            print("block", i)
            self.moveToGetPlace(Settings.BLOCKS_ORDINARY[i], ensure_color_index(color), down=10)
            self.suck()
            self.gotoColor()
            if Settings.COLOR_SERIES is not None and i < len(Settings.COLOR_SERIES):
                color = Settings.COLOR_SERIES[i]
            else:
                color = self.readColor(times=10, default=(0, 1, 0))
            self.moveToPutPlace(color, down=10)
            self.release(up=10)

    def moveToGetPlace(self, i, last_color_index, down=0):
        pose = self.getGetPose(i)
        nowPose = self.dobot.GetPose()[:3]
        if nowPose[1] > 100:
            should_height = max(self.getShouldHeight(last_color_index, nowPose[2]), Settings.RIGHT_COLOR_BASE[2])
            self.moveTo(z=max(nowPose[2], should_height))

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
                    if color_exists(color[i]):
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
        self.moveInc(dz=up, straight=True)

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
        self.moveTo(*Settings.RIGHT_COLOR_BASE)

    def moveToPutPlace(self, color, down=0):
        now_pose = self.dobot.GetPose()
        if type(color) == tuple or type(color) == list:
            index = ensure_color_index(color, -1)
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
        should_height = self.getShouldHeight(index, target_pose[2] - Settings.BLOCK_SIZE)

        if should_height > Settings.RIGHT_COLOR_BASE[2]:
                self.moveTo(z=max(target_pose[2], should_height + Settings.BLOCK_SIZE))

        self.moveTo(*target_pose[:2], r=target_pose[3])
        target_pose[2] -= down
        self.moveTo(*target_pose)

    def getShouldBlocksHeight(self, color_index):
        return max(self.counts[color_index + 1:])

    def getShouldHeight(self, color_index, target_pose_height):
        if color_index < 0:
            return target_pose_height

        max_blocks = self.getShouldBlocksHeight(color_index)

        return max(target_pose_height, Settings.RIGHT_PUT_BASE[2] + max_blocks * Settings.BLOCK_SIZE)

    def getPutPose(self, color_index):
        target_pose = [0, 0, 0, 0]
        target_pose[0] = Settings.RIGHT_PUT_BASE[0] - Settings.RIGHT_PUT_DIS_X * (3 - color_index)
        target_pose[1] = Settings.RIGHT_PUT_BASE[1] + Settings.RIGHT_PUT_DIS_Y * (
                3 - self.counts[color_index] // Settings.RIGHT_PUT_LIMIT * 3)
        target_pose[2] = Settings.RIGHT_PUT_BASE[2] + (
                self.counts[color_index] % Settings.RIGHT_PUT_LIMIT + 1) * Settings.BLOCK_SIZE
        return target_pose

    def blockComes(self):
        return 1 in self.getColor()

    def getColor(self):
        return self.dobot.GetColorSensor()

    @staticmethod
    def getGetPose(i):
        x = i % 10 // 5
        y = i % 5
        z = i // 10

        pose = list(Settings.RIGHT_GET_BASE)
        pose[0] += x * Settings.RIGHT_GET_DIS_X
        pose[1] -= y * Settings.RIGHT_GET_DIS_Y
        pose[2] += z * Settings.BLOCK_SIZE

        return pose


if __name__ == '__main__':
    right = Robot()
    right.setAddr(Robot.search()[0])
    right.run()
    # right.counts = [0, 2, 3]
    # print(right.getShouldBlocksHeight(0))
