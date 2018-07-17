import threading
import time
from typing import List

import DualCarriers
from DobotControl import *

mutex = threading.Lock()

UPPER = 0
LOWER = 4

sr = ["10.3.21.125"]


class Dobot(DobotControl):
    def __init__(self):
        super().__init__()
        self.home_pose = (219.97760009765625, -4.5157277781981975e-05, 30.15045928955078)
        self.running = False
        self.command = ""

    def user_init(self):

        # self.dobot.SetWIFIConfigMode(True)
        # self.dobot.SetWIFISSID("teacher")
        # self.dobot.SetWIFIPassword("yuanding")
        # self.dobot.SetWIFIIPAddress(1, 0, 0, 0, 0)

        print(self.dobot.GetWIFISSID(), self.dobot.GetWIFIPassword(), self.dobot.GetWIFIConnectStatus(), self.dobot.GetWIFIIPAddress())
        self.dobot.SetColorSensor(1, DobotTypes.ColorPort.PORT_GP2)

        self.dobot.SetEMotorEx(DualCarriers.Settings.MOTOR_PORT, 0, 0, 1)
        self.running = True

    def work(self):
        while self.running:
            global UPPER, LOWER
            while LOWER < len(sr):
                time.sleep(0.1)
            self.moveInc(dz=30)
            mutex.acquire()
            UPPER += 1
            LOWER -= 1
            mutex.release()
            while UPPER < len(sr):
                time.sleep(0.1)
            self.moveInc(dz=-30)
            mutex.acquire()
            LOWER += 1
            UPPER -= 1
            mutex.release()

        print("exit", self.addr)

    def exec(self, cmd_):
        self.command = cmd_


class Command:
    def __init__(self):
        self.dobots: List[Dobot] = []

    def add(self, dobot: Dobot):
        self.dobots.append(dobot)

    def work(self):
        for e in self.dobots:
            e.running = True
            e.setDaemon(True)
            e.start()
        try:

            input("Quit?")
        finally:
            for e in self.dobots:
                e.running = False


if __name__ == "__main__":
    cmd = Command()
    for i in range(len(sr)):
        dbt = Dobot()
        dbt.connect(sr[i])
        cmd.add(dbt)
    cmd.work()
