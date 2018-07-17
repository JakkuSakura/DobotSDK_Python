import time

import DobotTypes
import DualCarriers
import DobotAPI
import DualCarriers
import winsound

from DobotControl import DobotControl


class Right(DobotControl):
    def __init__(self):
        super().__init__()

    def user_init(self):
        self.dobot.SetColorSensor(1, DobotTypes.ColorPort.PORT_GP4)

    def work(self):

        while True:
            # print(self.addr, self.dobot.GetPose())
            if 1 in self.dobot.GetColorSensor():
                hz = 500
                for i in range(3):
                    hz += 200
                    if self.dobot.GetColorSensorEx(i):
                        break
                winsound.Beep(hz, 100)
                print(self.addr, self.dobot.GetColorSensor())
            else:
                time.sleep(0.01)


class Left(DobotControl):
    def __init__(self, ):
        super().__init__()
        if DualCarriers.Settings.HOME_INIT:
            self.reset_zero(DualCarriers.Settings.HOME_BASE)

    def work(self):
        print("running left")
        for i in range(0):
            self.getBlockLeft(i)
            self.gotoPut()
            self.stopMoto(DobotTypes.EMotorPort.EMOTOR_1)
            self.release()
            self.startMoto(DobotTypes.EMotorPort.EMOTOR_1)
        self.startMoto(DobotTypes.EMotorPort.EMOTOR_1)

    def moveSpt(self, x, y, z, spt_times):
        now = self.dobot.GetPose()
        tup = (x, y, z)
        each_list = [(tup[x] - now[x]) / spt_times for x in range(3)]
        print(each_list)
        for i in range(spt_times):
            self.moveInc(*each_list)

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

    def clean(self):
        self.stopMoto(DobotTypes.EMotorPort.EMOTOR_1)


if __name__ == "__main__":
    rt = Right()
    rt.connect(DobotControl.search()[0])
    rt.start()

    # left = Left(1, "COM5")
    # left.start()
