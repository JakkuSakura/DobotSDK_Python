
import time

import DualCarriers
from DobotControl import DobotControl


class Dbt(DobotControl):
    def __init__(self, index, addr):
        super().__init__(index, addr)
        if DualCarriers.Settings.HOME_INIT:
            self.reset_zero(DualCarriers.Settings.HOME_BASE)

    def work(self):
        self.moveTo(y=-1, r=100)
        self.moveTo(y=1, r=-100)

if __name__ == "__main__":
    db = Dbt(1, "COM5")
    db.setDaemon(True)
    db.start()
    db.join()
