import time

from DobotControl import DobotControl


class Dbt(DobotControl):
    def work(self):
        # self.reset_zero((240, 0, 80))
        self.moveTo(90.63232421875, -150.4977569580078, -30.65545654296875)


dbt = Dbt()
dbt.connect(DobotControl.searched)