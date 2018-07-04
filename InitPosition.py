import time

import Carrier


class Dobot(Carrier.DobotControl):
    def __init__(self, index, COM):
        super().__init__(index, COM)

    def work(self):
        self.dobot.SetHOMECmdEx(temp=0, isQueued=1)
        while True:
            print(self.addr, self.dobot.GetPose())
            time.sleep(0.5)


if __name__ == "__main__":
    sr = Dobot.search()
    dl = [Dobot(i, sr[i]) for i in range(len(sr))]
    for e in dl:
        e.start()
