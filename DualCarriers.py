import threading
import time
from typing import List

import DobotAPI
import DobotTypes
from DobotControl import DobotControl, find_color_index, color_exists


class Settings:
    COLOR_SERIES = [1, 1, 2, 2]
    MOTOR_DIS = 542 / 6
    DEFAULT_MOTO_SPEED = 50
    COM_LEFT = "COM6"
    COM_RIGHT = "COM5"

    HOME_BASE = (230, 0, 80, 30)

    RIGHT_TEMP_GET_DIS = 30

    RIGHT_TEMP_BASE = (147.23934936523438, 195.394775390625, -40.386314392089844)
    RIGHT_GET_BASE = (245.8165740966797, 0, 20.960845947265625)
    RIGHT_FIX_Y = (11, 7, 7)
    RIGHT_PUT_BASE = (150.06248474121094, -191.3921356201172, -66.0019302368164)
    RIGHT_WASTE_POSE = (173, 102, 10)
    RIGHT_PUT_DIS_X = 65
    RIGHT_PUT_DIS_Y = 27

    RIGHT_PUT_LIMIT = 7
    # it shouldnt be 8

    BLOCK_SIZE = 26
    DobotAPI.OutPutFlag = False

    LEFT_GET_BASE = (85.82850646972656, 192.2878875732422, -32.388450622558594)
    LEFT_PUT_BASE = (252.84046936035156, 0, 20.267623901367188)

    LEFT_GET_DIS_Y = 30
    LEFT_GET_DIS_X = 30
    MOTOR_PORT = DobotTypes.EMotorPort.EMOTOR_1
    # MOVE_TIME = 1.5
    # INFRARED_PORT = DobotTypes.InfraredPort.PORT_GP2
    COLOR_PORT = DobotTypes.ColorPort.PORT_GP2

    ENABLE_LEFT = True
    ENABLE_RIGHT = True

    RIGHT_DEBUG = False
    LEFT_DEBUG = False

    HOME_INIT = False


class Interal(threading.Thread):
    def __init__(self, right):
        super().__init__()
        self.right: Right = right
        self.hold = False

    def run(self):
        while self.right.glb.is_running:
            if color_exists(self.right.readColor(times=30)):
                self.right.glb.is_taken = False
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
        self.speed = 1000
        self.acc = 600
        self.setAddr(addr)
        self.interal = Interal(self)

    def user_init(self):
        if Settings.HOME_INIT:
            self.reset_zero(Settings.HOME_BASE)
        self.dobot.SetColorSensor(1, Settings.COLOR_PORT)
        self.startMoto(Settings.MOTOR_PORT, Settings.DEFAULT_MOTO_SPEED)
        # self.dobot.SetInfraredSensor(1, Settings.INFRARED_PORT)
        self.moveTo(r=Settings.HOME_BASE[3])
        self.interal.setDaemon(True)
        self.interal.start()
        if Settings.RIGHT_DEBUG:
            self.moveAboveGetPlace()

            self.moveToGetTemp(0)

            self.moveToPutPlace(0)
            self.counts = [0, 0, 0, 0]
            self.moveAboveGetPlace()

            while Settings.RIGHT_DEBUG:
                time.sleep(0.1)

    def work(self):
        print("running right")
        print("temp blocks")
        for i in range(4):
            if not self.glb.is_running:
                return
            print("temp block", i)
            self.moveToGetTemp(i)
            self.capture(down=10, up=0)
            color = Settings.COLOR_SERIES[i]
            self.moveToPutPlace(color)
            self.release(down=5, up=10)
        self.glb.finished_temp = True

        print("left zone blocks")
        while self.glb.is_running and self.glb.left is not None and self.glb.left_put:
            time.sleep(0.1)

        color = (0, 0, 0)
        for i in range(12):
            if not self.glb.is_running:
                return
            self.moveAboveGetPlace(last_color_index=find_color_index(color))
            self.waitComes()
            color = self.readColor(times=15)
            self.interal.hold = True
            self.capture(down=10, up=15)
            self.glb.is_taken = True
            self.moveToPutPlace(color)
            self.release(down=5, up=10)

    def moveToGetTemp(self, i):
        pose = self.getTempGetPose(i)
        nowPose = self.dobot.GetPose()[:3]
        #  if the dobot is in the right area
        if nowPose[1] - 30 < Settings.RIGHT_GET_BASE[1]:
            if nowPose[2] < 60:
                self.moveTo(z=60)

        self.moveTo(*pose[:2])
        self.moveTo(*pose)

    def readColor(self, times=1, default=(0, 0, 0)):
        for _ in range(times):
            color = self.getColor()
            if color_exists(color):
                hz = 500
                for i in range(3):
                    hz += 200
                    if color[i]:
                        break
                import winsound
                winsound.Beep(hz, 300)
                print("color ", color)
                return color
        else:
            print("color ", default)
            return default

    @staticmethod
    def getTempGetPose(i):
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

    def moveAboveGetPlace(self, fix=0, last_color_index=-1):
        now_pose = self.dobot.GetPose()
        target_pose = list(Settings.RIGHT_GET_BASE)
        target_pose[1] += fix

        if now_pose[1] < -50:
            should_height = max(self.getShouldHeight(last_color_index, now_pose[2]), Settings.RIGHT_GET_BASE[2])
            self.moveTo(z=max(now_pose[2], should_height))

        self.moveTo(*target_pose[:2])
        self.moveTo(*target_pose)

    def capture(self, down=15, up=5):
        self.moveInc(dz=-down)
        self.suck()
        time.sleep(0.1)
        self.moveInc(dz=up)

    def moveToPutPlace(self, color):
        if type(color) == tuple or type(color) == list:
            index = find_color_index(color, -1)
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

        target_pose = self.getPutPose(index)

        self.counts[index] += 1

        print("put", self.counts)
        should_height = self.getShouldHeight(index, target_pose[2] - Settings.BLOCK_SIZE)

        self.moveTo(z=max(now_pose[2], target_pose[2], should_height + Settings.BLOCK_SIZE,
                          Settings.RIGHT_GET_BASE[2] + Settings.BLOCK_SIZE * 2))

        self.moveTo(*target_pose[:2])

        self.moveTo(*target_pose)

    def getPutPose(self, index):
        target_pose = [0, 0, 0]
        target_pose[0] = Settings.RIGHT_PUT_BASE[0] - Settings.RIGHT_PUT_DIS_X * index
        target_pose[1] = Settings.RIGHT_PUT_BASE[1] + Settings.RIGHT_PUT_DIS_Y * (
                self.counts[index] // Settings.RIGHT_PUT_LIMIT * 3 - 2)
        target_pose[2] = Settings.RIGHT_PUT_BASE[2] + (
                self.counts[index] % Settings.RIGHT_PUT_LIMIT + 1) * Settings.BLOCK_SIZE
        return target_pose

    def getShouldHeight(self, index, target_pose_height):
        if index <= 0:
            return target_pose_height

        max_blocks = max(self.counts[:index])

        return max(target_pose_height, Settings.RIGHT_PUT_BASE[2] + max_blocks * Settings.BLOCK_SIZE)

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
        self.speed = 1000
        self.acc = 600
        self.setAddr(addr)

    def user_init(self):
        if Settings.HOME_INIT:
            self.reset_zero(Settings.HOME_BASE)
        self.unsuck()
        self.stopMoto()

        if Settings.LEFT_DEBUG:
            self.gotoPut()
            for i in range(12):
                self.getBlockLeft(i)
            self.gotoPut()

            while Settings.LEFT_DEBUG:
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
            self.startMotoS()
        # something strange happened to the last 6th block
        self.glb.is_taken = False
        while self.glb.is_running:
            self.waitTaken()
            time.sleep(2)
            self.startMotoS()
            time.sleep(2)
        print("left ok")
        self.glb.trans_moto_control = True

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

    def stopMoto(self):
        if self.left is not None and self.left.isOk():
            self.left.stopMoto(Settings.MOTOR_PORT)

    def blockComes(self):
        if self.right is not None and self.right.isOk():
            return self.right.blockComes()
        return False

    def startMoto(self):
        if self.left is not None and self.left.isOk():
            self.left.startMoto(Settings.MOTOR_PORT, Settings.DEFAULT_MOTO_SPEED)

    def run(self):
        ipt = input("Go?[G/r]")
        if ipt == 'r':
            exit(0)
        for e in self.dobots:
            if e is not None:
                e.setDaemon(True)
                e.start()
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


if __name__ == '__main__':
    glb = Global()
    glb.run()
