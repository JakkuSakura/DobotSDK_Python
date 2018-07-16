import time

import DobotTypes
import DualCarriers
from DobotControl import DobotControl


class Dbt(DobotControl):
    def __init__(self):
        super().__init__()
        if DualCarriers.Settings.HOME_INIT:
            self.reset_zero(DualCarriers.Settings.HOME_BASE)

    def work(self):
        for e in range(30, 301, 10):
            print("now speed", e)
            self.startMoto(DobotTypes.EMotorPort.EMOTOR_1, e)
            input("next speed?")


if __name__ == "__main__":
    db = Dbt()
    db.connect("COM6")
    db.setDaemon(True)
    db.start()
    db.join()
