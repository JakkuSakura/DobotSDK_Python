import time

import DobotAPI
import DualCarriers
import winsound

from DobotControl import color_exists


class Dobot(DualCarriers.DobotControl):
    def __init__(self, index, COM):
        super().__init__(index, COM)

    def user_init(self):
        self.dobot.SetColorSensor(1, DobotAPI.ColorPort.PORT_GP2)

        self.dobot.SetEMotorEx(DualCarriers.Settings.MOTOR_PORT, 0, 0, 1)

    def work(self):
        self.unsuck()
        self.moveTo(220, 0, 80)
        # self.home((220, 0, 80))

        while True:
            print(self.addr, self.dobot.GetPose())
            print(self.addr, self.dobot.GetColorSensor())
            if color_exists(self.dobot.GetColorSensor()):
                hz = 500
                for i in range(3):
                    hz += 200
                    if self.dobot.GetColorSensorEx(i):
                        break
                winsound.Beep(hz, 300)
            else:
                time.sleep(0.2)


if __name__ == "__main__":
    sr = Dobot.search()
    dl = [Dobot(i, sr[i]) for i in range(len(sr))]
    for e in dl:
        e.start()
