import time
import winsound

import DobotAPI
import DobotTypes
from DobotControl import DobotControl, color_exists, ensure_color_index


class Right(DobotControl):
    color_dic = {
        -1: "Black",
        0: "Red",
        1: "Green",
        2: "Blue"
    }
    DobotAPI.OutPutFlag = False

    def __init__(self):
        super().__init__()

    def user_init(self):
        # self.setColotSensor(output_port=13, input_left=12, input_right=15, enable=True)
        self.setPump(17, 11)
    def work(self):
        while True:
            self.suck()
            time.sleep(1)
            self.unsuck()
            time.sleep(1)

            # color = self.getColorSensor()
            # if color_exists(color):
            #     hz = 300
            #     for i in range(3):
            #         hz += 300
            #         if color_exists(color[i]):
            #             break
            #     winsound.Beep(hz, 500)
            # else:
            #     time.sleep(0.5)
            # print(self.addr, color,
            #       Right.color_dic[find_color_index(color)])


if __name__ == "__main__":
    rt = Right()
    rt.setAddr(DobotControl.search()[0])
    rt.start()
