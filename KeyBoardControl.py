import sys
import time
import uuid

from typing import List

import DobotAPI
import DobotTypes
import DualCarriers


# import winsound


class Dobot(DualCarriers.DobotControl):
    def __init__(self):
        super().__init__()
        self.home_pose = (219.97760009765625, -4.5157277781981975e-05, 60.15045928955078)
        self.running = False
        self.command = ""

    def user_init(self):
        self.dobot.SetColorSensor(1, DobotTypes.ColorPort.PORT_GP2)

        self.dobot.SetEMotorEx(DualCarriers.Settings.MOTOR_PORT, 0, 0, 1)
        self.running = True

    def work(self):
        while self.running:
            while self.command == '':
                time.sleep(0.1)
            try:
                print(self.addr, self.command)
                if self.command == "s":
                    print(self.addr, tuple(self.dobot.GetPose()))
                elif self.command == "c":
                    print(self.addr, self.dobot.GetColorSensor())
                elif self.command.startswith("m"):
                    spt = self.command[1:].split()
                    if spt[0] == 'u':
                        self.moveInc(dz=float(spt[1]))
                    elif spt[0] == 'd':
                        self.moveInc(dz=-float(spt[1]))
                    elif spt[0] == 'l':
                        self.moveInc(dy=float(spt[1]))
                    elif spt[0] == 'r':
                        self.moveInc(dy=-float(spt[1]))
                    elif spt[0] == 'f':
                        self.moveInc(dx=float(spt[1]))
                    elif spt[0] == 'b':
                        self.moveInc(dx=-float(spt[1]))
                    elif spt[0] == 'rh':
                        self.moveInc(dr=float(spt[1]))
                elif self.command == "hm":
                    print("goto home")
                    self.moveTo(*self.home_pose)
                elif self.command == "us":
                    self.unsuck()
                elif self.command == "sk":
                    self.suck()
                elif self.command == "cl":
                    self.reset_zero(self.home_pose)
                elif self.command == "sr":
                    self.home_pose = self.dobot.GetPose()[:3]
                elif self.command.startswith("to"):
                    self.command = self.command[2:]
                    spt = self.command.split()
                    self.moveTo(spt[0], spt[1], spt[2], spt[3])
                elif self.command == 'pt':
                    print(self.addr)
                else:
                    raise Exception("Unknown command")
            except Exception as e:
                print("Input error, try again", self.addr, e)
            finally:
                self.command = ''

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
            while True:
                time.sleep(1)
                cmd_ = input(">> ")
                try:
                    if not cmd_:
                        continue
                    if cmd_ == 'q':
                        return
                    id = int(cmd_[0])
                    cmd_ = cmd_[1:]
                    if cmd_[0] == 'w':
                        while True:
                            self.dobots[id].exec(cmd_[1:])
                            time.sleep(0.5)

                    else:
                        self.dobots[id].exec(cmd_)
                except KeyboardInterrupt:
                    raise
                except Exception as e:
                    print("Input error")
        finally:
            for e in self.dobots:
                e.running = False


if __name__ == "__main__":
    sr = Dobot.search()
    cmd = Command()
    for i in range(len(sr)):
        dbt = Dobot()
        dbt.setAddr(sr[i])
        cmd.add(dbt)
    cmd.work()
