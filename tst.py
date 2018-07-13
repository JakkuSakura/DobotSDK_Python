import time

from DobotControl import DobotControl


class Dbt(DobotControl):
    def work(self):
        self.reset_zero((220, 0, 30))


Dbt(0, Dbt.search()[0]).run()
