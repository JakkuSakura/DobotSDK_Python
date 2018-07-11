import time

import Carrier
from DobotControl import DobotControl


class Left(DobotControl):
    def __init__(self, index, COM):
        super().__init__(index, COM)
        if Carrier.Settings.HOME_INIT:
            self.home(Carrier.Settings.HOME_BASE)

    def work(self):
        print("running left")
        for s in range(600, 800, 50):
            print("speed", s)
            self.init(s)
            for i in range(3):
                self.getBlockLeft(i % 12)
                self.gotoPut()

    def getBlockLeft(self, index):
        lst = list(Carrier.Settings.LEFT_GET_BASE)
        lst[0] -= Carrier.Settings.LEFT_GET_DIS_X * (index // 4)
        lst[1] -= Carrier.Settings.LEFT_GET_DIS_Y * (index % 4)
        self.moveTo(*lst)
        self.moveInc(dz=-15)
        self.suck()
        time.sleep(0.1)
        self.moveInc(dz=50)

    def gotoPut(self):
        l = list(Carrier.Settings.LEFT_PUT_BASE)
        l[2] += 10
        self.moveTo(*l)
        self.moveInc(dz=-10)

    def release(self):
        time.sleep(0.1)
        self.unsuck()
        time.sleep(0.1)
        self.moveInc(dz=20)


if __name__ == "__main__":
    left = Left(1, "COM5")
    left.start()
