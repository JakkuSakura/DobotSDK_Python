import threading
import time
from typing import List

import DobotAPI
import DobotTypes
from DobotControl import DobotControl, ensure_color_index, color_exists, ensure_color_tuple


class Settings:
    COLOR_SERIES = [1, 1, 2, 2]
    MOTOR_DIS = 542 / 6
    DEFAULT_MOTO_SPEED = 80
    COM_LEFT = "COM6"
    COM_RIGHT = "COM5"

    HOME_BASE = (230, 0, 80, 30)

    RIGHT_TEMP_GET_DIS = 30

    RIGHT_TEMP_BASE = (149.23934936523438, 195.394775390625, -40.386314392089844)
    RIGHT_GET_BASE = (249.8165740966797, 0, 20.960845947265625)
    RIGHT_PUT_BASE = (150.06248474121094, -191.3921356201172, -66.0019302368164)
    RIGHT_WASTE_POSE = (173, 102, 10)

    RIGHT_PUT_DIS_X = 65
    RIGHT_PUT_DIS_Y = 27

    RIGHT_PUT_LIMIT = 7
    # it shouldnt be 8

    BLOCK_SIZE = 25.3
    DobotAPI.OutPutFlag = False

    LEFT_GET_BASE = (79.82850646972656, 192.2878875732422, -32.388450622558594)
    LEFT_PUT_BASE = (255.84046936035156, 0, 20.267623901367188)

    LEFT_GET_DIS_Y = 30
    LEFT_GET_DIS_X = 30
    MOTOR_PORT = DobotTypes.EMotorPort.EMOTOR_1
    COLOR_PORT = DobotTypes.ColorPort.PORT_GP2

    ENABLE_LEFT = True
    ENABLE_RIGHT = True

    RIGHT_DEBUG = False
    LEFT_DEBUG = False

    HOME_INIT = False


class Interal(threading.Thread):
    def __init__(self, delay_s, func):
        super().__init__()
        self.delay = delay_s
        self.func = func

    def run(self):
        time.sleep(self.delay)
        self.func()


class ColorInteral(threading.Thread):
    def __init__(self, right):
        super().__init__()
        self.right: Right = right
        self.hold = False

    def run(self):
        while self.right.glb.is_running:
            if color_exists(self.right.readColor(blank_times=30)):
                self.right.glb.is_taken = False
                self.right.glb.is_first_block_arrived = True
            else:
                # self.right.glb.is_taken = True
                pass
            time.sleep(0.1)
            if self.hold:
                time.sleep(1.2)
                self.hold = False


class Right(DobotControl):
    def __init__(self, global_obj, addr):
        super().__init__()
        self.isWaitting = False
        self.glb: Global = global_obj
        self.counts = [0 for _ in range(4)]
        self.speed = 1200
        self.acc = 620
        self.setAddr(addr)
        self.interal = ColorInteral(self)

    def user_init(self):
        self.dobot.SetColorSensor(1, Settings.COLOR_PORT)
        self.startMoto(Settings.MOTOR_PORT, Settings.DEFAULT_MOTO_SPEED)
        self.interal.setDaemon(True)
        self.interal.start()
        if Settings.RIGHT_DEBUG:
            self.moveAboveGetPlace()

            self.moveToGetTemp(0)

            self.moveToPutPlace(0)
            self.counts = [0, 0, 0, 0]
            self.moveAboveGetPlace()

        while Settings.LEFT_DEBUG or Settings.RIGHT_DEBUG:
            time.sleep(0.1)

    def work(self):
        print("running right")
        print("temp blocks")
        for i in range(4):
            if not self.glb.is_running:
                return
            print("temp block", i)
            self.moveToGetTemp(i, down=8)
            self.capture(down=0, up=0)
            color = Settings.COLOR_SERIES[i]
            self.moveToPutPlace(color, down=5)
            self.release(up=0)
        self.glb.finished_temp = True
        self.moveAboveGetPlace()
        while self.glb.is_running and not self.glb.is_first_block_arrived:
            time.sleep(0.01)

        color = (0, 0, 0)
        for i in range(12):
            if not self.glb.is_running:
                return
            self.moveAboveGetPlace(last_color_index=ensure_color_index(color), down=15)
            # if not self.glb.trans_moto_control:
            # self.waitComes()
            color = self.readColor(blank_times=15, default=(0, 1, 0))
            self.interal.hold = True
            self.capture(down=0, up=0)
            self.glb.is_taken = True
            if self.glb.trans_moto_control:
                Interal(0.4, self.glb.startMotoS).start()
            self.moveToPutPlace(color, down=5)
            self.release(down=0, up=10)

    def moveToGetTemp(self, i, down=0):
        pose = self.calcTempGetPose(i)
        nowPose = self.dobot.GetPose()[:3]
        #  if the dobot is in the right area
        if nowPose[1] - 30 < Settings.RIGHT_GET_BASE[1]:
            if nowPose[2] < 60:
                self.moveTo(z=60)

        self.moveTo(*pose[:2], r=Settings.HOME_BASE[3])
        pose[2] -= down
        self.moveTo(*pose)

    def readColor(self, blank_times=1, color_times=1, default=(0, 0, 0), output=True):
        for _ in range(blank_times):
            color = self.getColor()
            if color_exists(color):
                cl = []
                for i in range(color_times):
                    cl.append(ensure_color_index(self.getColor()))
                cx = [0, 0, 0]
                for i in range(3):
                    if color_exists(cl[i]):
                        cx[i] += 1

                max_index = -1
                for i in range(3):
                    if max_index >= 0 and cx[i] > cx[max_index]:
                        max_index = i
                color = ensure_color_tuple(max_index, default)
                break
        else:
            color = default
        if output:
            print("color ", default)
        return color

    @staticmethod
    def calcTempGetPose(i):
        lz = i // 4
        lx = i % 4 // 2
        ly = (i + 1) % 2
        pose = list(Settings.RIGHT_TEMP_BASE)
        tp = [lx, ly, lz]
        for j in range(3):
            pose[j] -= Settings.RIGHT_TEMP_GET_DIS * tp[j]
        return pose

    def release(self, down=0, up=0):
        self.moveInc(dz=-down)
        self.unsuck()
        time.sleep(0.1)
        self.moveInc(dz=up)

    def moveAboveGetPlace(self, fix=0, last_color_index=-1, down=0):
        now_pose = self.dobot.GetPose()
        target_pose = list(Settings.RIGHT_GET_BASE)
        target_pose[1] += fix

        if now_pose[1] < -50:
            should_height = self.calcShouldHeight(last_color_index, target_pose[2], with_block=False)
            self.moveTo(z=max(should_height, now_pose[2]))

        self.moveTo(*target_pose[:2])
        # while self.glb.is_taken:
        #     time.sleep(0.01)
        target_pose[2] -= down
        self.moveTo(*target_pose)

    def capture(self, down=15, up=5):
        self.moveInc(dz=-down)
        self.suck()
        time.sleep(0.1)
        self.moveInc(dz=up)

    def moveToPutPlace(self, color, down=0):
        if type(color) == tuple or type(color) == list:
            index = ensure_color_index(color, -1)
        else:
            index = color
        now_pose = self.dobot.GetPose()

        if index < 0:
            target_pose = list(Settings.RIGHT_WASTE_POSE)
            self.counts[3] += 1
            print("throw block", self.counts[3])
            self.moveTo(z=target_pose[2] + 80)
            self.moveTo(*target_pose[:2])
            return

        target_pose = self.calcPutPose(index)

        self.counts[index] += 1

        print("put", self.counts)
        should_height = self.calcShouldHeight(index, target_pose[2], with_block=True)

        self.moveTo(z=max(now_pose[2], should_height, Settings.RIGHT_GET_BASE[2] + Settings.BLOCK_SIZE * 2))

        self.moveTo(*target_pose[:2])
        target_pose[2] -= down
        self.moveTo(*target_pose)

    def calcPutPose(self, index):
        target_pose = [0, 0, 0]
        target_pose[0] = Settings.RIGHT_PUT_BASE[0] - Settings.RIGHT_PUT_DIS_X * index
        target_pose[1] = Settings.RIGHT_PUT_BASE[1] + Settings.RIGHT_PUT_DIS_Y * (
                self.counts[index] // Settings.RIGHT_PUT_LIMIT * 3 - 2)
        target_pose[2] = Settings.RIGHT_PUT_BASE[2] + (
                self.counts[index] % Settings.RIGHT_PUT_LIMIT + 1) * Settings.BLOCK_SIZE
        return target_pose

    def calcShouldHeight(self, color_index, target_pose_height, with_block):
        """
        :param color_index:
        :param target_pose_height: the target position in z-axis without block
        :param with_block:
        :return: where the machine should go in z-axis
        """
        if color_index <= 0:
            return target_pose_height
        if not with_block:
            max_blocks = max(self.counts[:color_index])
        else:
            max_blocks = max(self.counts[:color_index]) + 1
        height = Settings.RIGHT_PUT_BASE[2] + max_blocks * Settings.BLOCK_SIZE
        return max(height, target_pose_height)

    def waitComes(self):
        print("waiting block comes")
        self.isWaitting = True
        while self.glb.is_running and not self.blockComes():
            time.sleep(0.01)
        self.isWaitting = False

    def blockComes(self):
        return color_exists(self.getColor())

    def getColor(self):
        return self.dobot.GetColorSensor()

    def clean(self):
        print("cleaning right")
        self.stopMoto(Settings.MOTOR_PORT)
        self.dobot.SetColorSensor(0, Settings.COLOR_PORT)
        self.glb.stop()

    def moveToTransferer(self):
        self.moveTo(x=Settings.RIGHT_GET_BASE[0] + 10, y=Settings.RIGHT_TEMP_BASE[1] - 60,
                    z=Settings.RIGHT_GET_BASE[2] + 20)


class Left(DobotControl):
    def __init__(self, global_obj, addr):
        super().__init__()
        self.glb: Global = global_obj
        self.left_putting = False
        self.speed = 1200
        self.acc = 680
        self.setAddr(addr)

    def user_init(self):
        self.unsuck()
        self.stopMoto()

        if Settings.LEFT_DEBUG:
            self.gotoPut()
            for i in range(12):
                self.getBlockLeft(i)
                time.sleep(3)
            self.gotoPut()

        while Settings.LEFT_DEBUG or Settings.RIGHT_DEBUG:
            time.sleep(0.1)

    def waitTaken(self):
        while self.glb.is_running and not self.glb.is_taken:
            time.sleep(0.01)

    def work(self):
        print("running left")
        for i in range(12):
            if not self.glb.is_running:
                return
            self.getBlockLeft(i)
            self.capture(down=15, up=50)
            self.gotoPut()
            self.left_putting = True
            self.release()
            self.left_putting = False
            self.waitTaken()
            if self.glb.is_first_block_arrived:
                time.sleep(0.4)
            self.startMotoS()
        print("left ok")
        self.glb.trans_moto_control = True
        while self.glb.is_running:
            time.sleep(1)

    def getBlockLeft(self, index):
        lst = list(Settings.LEFT_GET_BASE)
        lst[0] -= Settings.LEFT_GET_DIS_X * (index // 4)
        lst[1] += Settings.LEFT_GET_DIS_Y * (index % 4)
        self.moveTo(*lst)

    def startMoto(self, port=Settings.MOTOR_PORT, speed=Settings.DEFAULT_MOTO_SPEED):
        super().startMoto(port, speed)

    def startMotoS(self, port=Settings.MOTOR_PORT, speed=Settings.DEFAULT_MOTO_SPEED, distance=Settings.MOTOR_DIS):
        super().startMotoS(port, speed, distance)

    def stopMoto(self, port=Settings.MOTOR_PORT):
        self.dobot.SetEMotorEx(port, 0, 0, 1)

    def gotoPut(self):
        l = list(Settings.LEFT_PUT_BASE)
        l[2] += 10
        self.moveTo(*l)
        self.moveInc(dz=-10)

    def release(self):
        time.sleep(0.1)
        self.unsuck()
        time.sleep(0.1)
        self.moveInc(dz=20)

    def clean(self):
        print("cleaning left")
        # self.stopMoto()
        self.unsuck()

    def capture(self, down=15, up=5):
        self.moveInc(dz=-down)
        self.suck()
        time.sleep(0.1)
        self.moveInc(dz=up)


class Global:

    def __init__(self):
        self.trans_moto_control = not Settings.ENABLE_LEFT

        self.left = Left(self, Settings.COM_LEFT) if Settings.ENABLE_LEFT else None
        self.right = Right(self, Settings.COM_RIGHT) if Settings.ENABLE_RIGHT else None
        self.dobots: List[DobotControl] = [self.left, self.right]
        self.sent = False
        self.is_taken = True
        self.is_running = True
        self.finished_temp = False
        self.left_put = False
        self.is_first_block_arrived = False

    def stopMoto(self):
        if self.left is not None and self.left.isOk():
            self.left.stopMoto()

    def blockComes(self):
        if self.right is not None and self.right.isOk():
            return self.right.blockComes()
        return False

    def startMoto(self):
        if self.left is not None and self.left.isOk():
            self.left.startMoto()

    def run(self):
        while True:
            ipt = input("Go?[G/r/h]")
            if ipt == 'r' or ipt == 'h':
                if self.right is not None:
                    self.right.connect()
                    if ipt == 'h':
                        self.right.reset_zero(Settings.HOME_BASE)
                    if ipt == 'r':
                        self.right.moveToGetTemp(0, down=-10)
                if self.left is not None:
                    self.left.connect()
                    if ipt == 'h':
                        self.left.reset_zero(Settings.HOME_BASE)
                    if ipt == 'r':
                        self.left.moveTo(*Settings.LEFT_GET_BASE[:2], z=Settings.LEFT_GET_BASE[2] + 10)
            else:
                break
        for e in self.dobots:
            if e is not None:
                e.setDaemon(True)
                e.start()
        if Settings.LEFT_DEBUG or Settings.RIGHT_DEBUG:
            input("Continue?")
        Settings.LEFT_DEBUG = Settings.RIGHT_DEBUG = False

        begin_time = time.time()
        while self.is_running:
            time.sleep(0.1)
            # inp = input()
            # if inp == 'q':
            #     self.is_running = False
            #     break

        end_time = time.time()

        print("used time:", end_time - begin_time)

        for e in self.dobots:
            if e is not None:
                e.join()

    def stop(self):
        print("stop")
        self.is_running = False

    def startMotoS(self):
        if self.left is not None and self.left.isOk():
            self.left.startMotoS()


if __name__ == '__main__':
    glb = Global()
    glb.run()
