import time

import DualCarriers
from DobotControl import DobotControl


class Left(DobotControl):
    def __init__(self, index, addr):
        super().__init__(index, addr)
        if DualCarriers.Settings.HOME_INIT:
            self.reset_zero(DualCarriers.Settings.HOME_BASE)

    def work(self):
        print("running left")
        for s in range(600, 800, 50):
            print("speed", s)
            self.init(s)
            for i in range(3):
                self.getBlockLeft(i % 12)
                self.gotoPut()

    def getBlockLeft(self, index):
        lst = list(DualCarriers.Settings.LEFT_GET_BASE)
        lst[0] -= DualCarriers.Settings.LEFT_GET_DIS_X * (index // 4)
        lst[1] -= DualCarriers.Settings.LEFT_GET_DIS_Y * (index % 4)
        self.moveTo(*lst)
        self.moveInc(dz=-15)
        self.suck()
        time.sleep(0.1)
        self.moveInc(dz=50)

    def gotoPut(self):
        l = list(DualCarriers.Settings.LEFT_PUT_BASE)
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
