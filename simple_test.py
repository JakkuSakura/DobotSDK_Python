
import time

import DualCarriers
from DobotControl import DobotControl


class Dbt(DobotControl):
    def __init__(self, index, COM):
        super().__init__(index, COM)
        if DualCarriers.Settings.HOME_INIT:
            self.home(DualCarriers.Settings.HOME_BASE)

    def work(self):
        while True:
            print("Gripper")
            self.dobot.SetEndEffectorGripperEx(1, 1, 1)
            time.sleep(5)
            print("Sucker")
            self.suck()
            time.sleep(5)


if __name__ == "__main__":
    db = Dbt(1, "COM6")
    db.start()
    db.join()
