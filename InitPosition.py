import sys
import time

from typing import List

import DobotAPI
import DualCarriers


# import winsound


class Dobot(DualCarriers.DobotControl):
    def __init__(self, index, addr):
        super().__init__(index, addr)
        self.home_pose = (219.97760009765625, -4.5157277781981975e-05, 80.15045928955078)
        self.running = False
        self.command = ""

    def user_init(self):
        self.dobot.SetColorSensor(1, DobotAPI.ColorPort.PORT_GP2)

        self.dobot.SetEMotorEx(DualCarriers.Settings.MOTOR_PORT, 0, 0, 1)
        self.running = True

    def work(self):
        while self.running:
            while self.command == '':
                time.sleep(0.1)
            try:
                cmd = self.command
                print(self.addr, cmd)
                if cmd == "s":
                    print(self.addr, tuple(self.dobot.GetPose()))
                elif cmd == "c":
                    print(self.addr, self.dobot.GetColorSensor())
                elif cmd.startswith("m"):
                    spt = cmd[1:].split()
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
                elif cmd == "hm":
                    print("goto home")
                    self.moveTo(*self.home_pose)
                elif cmd == "us":
                    self.unsuck()
                elif cmd == "sk":
                    self.suck()
                elif cmd == "cl":
                    self.reset_zero(self.home_pose)
                elif cmd == "sr":
                    self.home_pose = self.dobot.GetPose()[:3]
                else:
                    raise Exception("Unknown command")
            except Exception as e:
                print("Input error, try again", self.addr, e)
            finally:
                self.command = ''

        print("exit", self.addr)

    def exec(self, cmd):
        self.command = cmd


class Command:
    def __init__(self):
        self.dobots: List[Dobot] = []

    def add(self, dobot: Dobot):
        self.dobots.append(dobot)

    def work(self):
        for e in self.dobots:
            e.start()
        try:
            while True:
                time.sleep(0.1)
                cmd = input(">> ")
                try:
                    id = int(cmd[0])
                    cmd = cmd[1:]
                    if cmd[0] == 'w':
                        while True:
                            self.dobots[id].exec(cmd[1:])
                            time.sleep(0.5)
                    elif cmd[0] == 'p':
                        print(id, self.dobots[id].addr)
                    else:
                        self.dobots[id].exec(cmd)
                except KeyboardInterrupt:
                    raise
                except Exception as e:
                    print("Input error")
        finally:
            for e in self.dobots:
                e.running = False


if __name__ == "__main__":
    sr = Dobot.search()
    dl = [Dobot(i, sr[i]) for i in range(len(sr))]
    cmd = Command()
    for e in dl:
        cmd.add(e)
    cmd.work()
