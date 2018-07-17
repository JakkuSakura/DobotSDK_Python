import DobotTypes
from DobotControl import DobotControl


class Dbt(DobotControl):
    def __init__(self):
        super().__init__()

    def work(self):
        for distance in range(520, 540, 5):
            print("now distance", distance)
            self.startMotoS(DobotTypes.EMotorPort.EMOTOR_1, 50, distance)
            input("next distance?")


if __name__ == "__main__":
    db = Dbt()
    db.setAddr(DobotControl.search()[0])
    db.setDaemon(True)
    db.start()
    db.join()
