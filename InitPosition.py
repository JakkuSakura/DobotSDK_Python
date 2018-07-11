import time

import DualCarriers
import DobotAPI
import DualCarriers
import winsound


class Dobot(DualCarriers.DobotControl):
    def __init__(self, index, COM):
        super().__init__(index, COM)

    def user_init(self):
        self.dobot.SetColorSensor(0, DobotAPI.ColorPort.PORT_GP4)

        self.dobot.SetEMotorEx(DualCarriers.Settings.MOTOR_PORT, 0, 0, 1)

    def work(self):
        self.unsuck()
        self.moveTo(220, 0, 80)
        self.home((220, 0, 80))

        while True:
            print(self.addr, self.dobot.GetPose())
            if 1 in self.dobot.GetColorSensor():
                hz = 500
                for i in range(3):
                    hz += 200
                    if self.dobot.GetColorSensorEx(i):
                        break
                winsound.Beep(hz, 300)
                print(self.addr, self.dobot.GetColorSensor())
            else:
                time.sleep(0.01)


if __name__ == "__main__":
    sr = Dobot.search()
    dl = [Dobot(i, sr[i]) for i in range(len(sr))]
    for e in dl:
        e.start()
