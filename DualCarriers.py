import threading
import time

from typing import List

import DobotAPI
from DobotSession import DobotSession
import DobotTypes
from DobotControl import DobotControl, find_color_index, color_exists


class Settings:
    COLOR_SERIES = None
    MOTOR_DIS = 25000
    DEFAULT_MOTO_SPEED = 80

    SN_LEFT = ""
    SN_RIGHT = ""

    HOME_BASE = (230, 0, 80, 90)

    RIGHT_TEMP_GET_DIS = 28
    RIGHT_TEMP_BASE = (153.78334045410156, 201.6071319580078, -41.10369110107422)
    RIGHT_GET_BASE = (253.4164581298828, -8.221147537231445, 22.742271423339844)
    RIGHT_FIX_Y = (11, 7, 7)
    RIGHT_PUT_BASE = (145.63232421875, -200.4977569580078, -66.65545654296875)
    RIGHT_WASTE_POSE = (173, 102, 10)
    RIGHT_PUT_DIS_X = 65
    RIGHT_PUT_DIS_Y = 27

    RIGHT_PUT_LIMIT = 7
    # it shouldnt be 8

    BLOCK_SIZE = 26
    DobotAPI.OutPutFlag = False

    LEFT_GET_BASE = (68.59613037109375, 279.8572692871094, -43.00749588012695)
    LEFT_PUT_BASE = (247.058349609375, 11.079216003417969, 18.330650329589844)

    LEFT_GET_DIS_Y = 30
    LEFT_GET_DIS_X = 30
    MOTOR_PORT = DobotTypes.EMotorPort.EMOTOR_1
    # MOVE_TIME = 1.5
    # INFRARED_PORT = DobotTypes.InfraredPort.PORT_GP2
    COLOR_PORT = DobotTypes.ColorPort.PORT_GP2

    ENABLE_LEFT = False

    RIGHT_DEBUG = False
    LEFT_DEBUG = False

    HOME_INIT = False


class Interal(threading.Thread):
    def __init__(self, right):
        super().__init__()
        self.right: Right = right

    def run(self):
        while self.right.glb.is_running:
            if self.right.blockComes():
                self.right.glb.is_taken = False
            time.sleep(0.1)


class Right(DobotControl):
    def __init__(self, global_obj):
        super().__init__()
        self.isWaitting = False
        self.glb: Global = global_obj
        self.counts = [0 for _ in range(4)]
        self.speed = 600
        self.acc = 600

    def user_init(self):
        if Settings.HOME_INIT:
            self.reset_zero(Settings.HOME_BASE)
        self.dobot.SetColorSensor(1, Settings.COLOR_PORT)
        self.startMoto(Settings.MOTOR_PORT, Settings.DEFAULT_MOTO_SPEED)
        # self.dobot.SetInfraredSensor(1, Settings.INFRARED_PORT)
        self.moveTo(*Settings.HOME_BASE)
        if Settings.RIGHT_DEBUG:
            self.moveAboveGetPlace()

            self.moveToGetTemp(0)

            self.moveToPutPlace(0)

            self.moveAboveGetPlace()

            while Settings.RIGHT_DEBUG:
                time.sleep(0.1)

    def work(self):
        """
        this method was used for default satuation.
        :return: 
        """
        print("running right")
        print("temp blocks")
        for i in range(4):
            if not self.glb.is_running:
                return
            print("temp block", i)
            self.moveToGetTemp(i)
            self.capture(up=30)
            if Settings.COLOR_SERIES is not None and i < len(Settings.COLOR_SERIES):
                color = Settings.COLOR_SERIES[i]
            else:
                self.moveAboveGetPlace()
                color = self.readColor(times=10, default=(0, 1, 0))
            self.moveToPutPlace(color)
            self.release(down=5, up=10)
        self.glb.finished_temp = True
        inter = Interal(self)
        inter.start()
        print("left zone blocks")
        for i in range(12):
            if not self.glb.is_running:
                return
            while self.glb.is_running and self.glb.left is not None and self.glb.left.left_putting:
                time.sleep(0.01)
            self.glb.startMoto()
            self.reset_pose()
            self.moveAboveGetPlace()
            self.glb.awaitting = True
            self.waitComes()
            self.glb.stopMoto()
            color = self.readColor(times=15)
            index = find_color_index(color, 1)

            self.moveAboveGetPlace(fix=Settings.RIGHT_FIX_Y[index])
            self.capture(up=15)
            self.glb.is_taken = True
            self.glb.awaitting = False
            self.moveToPutPlace(color)
            self.release(down=5, up=30)

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

    def moveAboveGetPlace(self, fix=0, last_color=-1):
        now_pose = self.dobot.GetPose()
        target_pose = list(Settings.RIGHT_GET_BASE)
        target_pose[1] += fix
        if now_pose[1] - 30 > target_pose[1]:
            print("moved specially from temp zone")
            self.moveTo(x=target_pose[0], y=(target_pose[1] + now_pose[1]) / 3, z=target_pose[2] + 20, r=90)
        else:
            should_height = self.getShouldHeight(last_color, target_pose[2])
            if now_pose[2] < should_height:
                self.moveTo(z=target_pose[2])
            self.moveTo(*target_pose[:2])

        self.moveTo(*target_pose)

    def capture(self, down=15, up=5):
        self.moveInc(dz=-down)
        self.suck()
        time.sleep(0.1)
        self.moveInc(dz=up)

    def moveToPutPlace(self, color):
        now_pose = self.dobot.GetPose()
        if type(color) == tuple or type(color) == list:
            index = find_color_index(color, -1)
        else:
            index = color
            if now_pose[2] < 60:
                self.moveTo(z=60)

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
        should_height = self.getShouldHeight(index, target_pose[2])
        if now_pose[2] < should_height:
            if target_pose[2] == should_height:
                self.moveTo(z=should_height)
            else:
                self.moveTo(z=should_height + Settings.BLOCK_SIZE)

        self.moveTo(*target_pose[:2], r=target_pose[3])
        self.moveTo(*target_pose)

    def getPutPose(self, index):
        target_pose = [0, 0, 0, -50]
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
        # self.dobot.SetColorSensor(0, Settings.COLOR_PORT)
        self.glb.stop()

    def moveToTransferer(self):
        self.moveTo(x=Settings.RIGHT_GET_BASE[0] + 10, y=Settings.RIGHT_TEMP_BASE[1] - 60,
                    z=Settings.RIGHT_GET_BASE[2] + 20)


class Left(DobotControl):
    def __init__(self, global_obj):
        super().__init__()
        self.glb: Global = global_obj
        self.left_putting = False
        self.speed = 600
        self.acc = 600
        if Settings.HOME_INIT:
            self.reset_zero(Settings.HOME_BASE)

    def user_init(self):
        self.unsuck()
        if Settings.LEFT_DEBUG:
            for i in range(12):
                self.getBlockLeft(i)
            self.gotoPut()

            while Settings.LEFT_DEBUG:
                time.sleep(0.1)

    def wait(self):
        if self.glb.finished_temp:
            print("wait taken")
            self.glb.waitTaken()

    def work(self):
        print("running left")
        for i in range(12):
            if not self.glb.is_running:
                return
            self.getBlockLeft(i)
            self.gotoPut()
            self.release()
        print("left ok")

    def work_old(self):
        print("running left")
        for i in range(12):
            if not self.glb.is_running:
                return
            self.wait()
            self.getBlockLeft(i)
            self.wait()
            self.gotoPut()
            self.left_putting = True
            self.stopMoto()
            self.release()
            self.startMoto()
            self.left_putting = False
        print("left ok")
        self.glb.trans_moto_control = True

    def getBlockLeft(self, index):
        lst = list(Settings.LEFT_GET_BASE)
        lst[0] -= Settings.LEFT_GET_DIS_X * (index // 4)
        lst[1] -= Settings.LEFT_GET_DIS_Y * (index % 4)
        self.moveTo(*lst)
        self.moveInc(dz=-10)
        self.suck()
        time.sleep(0.2)
        self.moveInc(dz=50)

    def startMoto(self, port=Settings.MOTOR_PORT, speed=Settings.DEFAULT_MOTO_SPEED):
        super().startMoto(port, speed)

    def startMotoS(self, port=Settings.MOTOR_PORT, distance=Settings.MOTOR_DIS, speed=Settings.DEFAULT_MOTO_SPEED):
        super().startMotoS(port, distance, speed)

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
        self.stopMoto()


class Global:

    def __init__(self):
        self.trans_moto_control = not Settings.ENABLE_LEFT

        self.left = Left(self) if Settings.ENABLE_LEFT else None
        self.right = Right(self)
        self.dobots: List[DobotControl] = [self.left, self.right]
        self.sent = False
        self.is_taken = True
        self.coming = False
        self.awaitting = False
        self.is_running = True
        self.finished_temp = False

    def waitTaken(self):
        while self.is_running and not self.is_taken or self.awaitting:
            time.sleep(0.01)

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
        # todo
        addrs = DobotControl.search()
        for add_i in range(addrs):
            dobot = DobotSession()
            dobot.ConnectDobot(addrs[add_i])
            sn = dobot.GetDeviceName()

        for e in self.dobots:
            e.connect()
            if e is not None and e.isOk():
                e.setDaemon(True)
                e.start()
        if Settings.LEFT_DEBUG or Settings.RIGHT_DEBUG:
            input("Go?")
            Settings.LEFT_DEBUG = Settings.RIGHT_DEBUG = False
        while self.is_running:
            inp = input()
            if inp == 'q':
                self.is_running = False
                break
        for e in self.dobots:
            if e is not None:
                e.join()

    def stop(self):
        print("stop")
        self.is_running = False


if __name__ == '__main__':
    glb = Global()
    glb.run()
