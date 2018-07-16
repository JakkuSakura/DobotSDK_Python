import os
import platform
import shutil
import sys
import time
from ctypes import *
from typing import List

import DobotTypes


##################  API func   ##################


def get_machine():
    """Return type of machine."""
    if os.name == 'nt' and sys.version_info[:2] < (2, 7):
        return os.environ.get("PROCESSOR_ARCHITEW6432",
                              os.environ.get('PROCESSOR_ARCHITECTURE', ''))
    else:
        return platform.machine()


def os_bits():
    """Return bitness of operating system, or None if unknown."""
    machine = get_machine()
    machine2bits = {'AMD64': 64, 'x86_64': 64, 'i386': 32, 'x86': 32}
    return machine2bits.get(machine, None)


pre_path = './' + platform.system() + '/x' + str(os_bits()) + '/'


def load(dobotId=0, split="_"):
    def getRequiredPath(path):
        spt = path.split('.')
        req = pre_path + '.'.join(spt[:-1]) + split + str(dobotId) + '.' + spt[-1]
        if not os.path.exists(req):
            shutil.copyfile(pre_path + path, req)
        # print(req)
        return req

    if platform.system() == "Windows":
        require = getRequiredPath("DobotDll.dll")
        return CDLL(require, RTLD_GLOBAL)
    elif platform.system() == "Darwin":  # MacOS
        require = getRequiredPath("libDobotDll.dylib")
        return CDLL(require, RTLD_GLOBAL)
    else:  # linux
        require = getRequiredPath("libDobotDll.so")
        return CDLL(require)


def dSleep(ms):
    time.sleep(ms / 1000)


def gettime():
    return time.time()


OutPutFlag = True


def output(*args, **kargs):
    if not OutPutFlag:
        return
    print(*args, **kargs)
    sys.stdout.flush()


def SearchDobot(api, maxLen=1000):
    szPara = create_string_buffer(1000)  # ((len(str(maxLen)) + 4) * maxLen + 10)
    l = api.SearchDobot(szPara, maxLen)
    if l == 0:
        return []
    ret = szPara.value.decode("utf-8")
    return ret.split(" ")


def ConnectDobot(api, portName="", baudrate=115200):
    szPara = create_string_buffer(100)
    szPara.raw = portName.encode("utf-8")
    fwType = create_string_buffer(100)
    version = create_string_buffer(100)
    result = api.ConnectDobot(szPara, baudrate, fwType, version)
    return result, fwType.value.decode("utf-8"), version.value.decode("utf-8")


def DisconnectDobot(api):
    output("Disconnect Dobot")
    api.DisconnectDobot()


def DobotExec(api):
    return api.DobotExec()


class AutoLevelingCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("controlFlag", c_ubyte),
        ("precision", c_float)
    ]


def SetAutoLevelingCmd(api, controlFlag, precision, isQueued=0):
    cmd = AutoLevelingCmd()
    cmd.controlFlag = controlFlag
    cmd.precision = precision
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetAutoLevelingCmd(byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetAutoLevelingResult(api):
    precision = c_float(0)
    while True:
        result = api.GetAutoLevelingResult(byref(precision))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output("GetAutoLevelingResult", precision, precision.value)
    output('GetAutoLevelingResult: precision=%d' % precision.value)
    return [precision.value]


def GetQueuedCmdCurrentIndex(api):
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.GetQueuedCmdCurrentIndex(byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(2)
            continue
        break
    return [queuedCmdIndex.value]


def SetQueuedCmdStartExec(api):
    while True:
        result = api.SetQueuedCmdStartExec()
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def SetQueuedCmdStopExec(api):
    while True:
        result = api.SetQueuedCmdStopExec()
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output("Stopped queue successfully")


def SetQueuedCmdForceStopExec(api):
    while True:
        result = api.SetQueuedCmdForceStopExec()
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break

    output("Stopped queue forcely and successfully")


def SetQueuedCmdStartDownload(api, totalLoop, linePerLoop):
    while True:
        result = api.SetQueuedCmdStartDownload(totalLoop, linePerLoop)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def SetQueuedCmdStopDownload(api):
    while True:
        result = api.SetQueuedCmdStopDownload()
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def SetQueuedCmdClear(api):
    return api.SetQueuedCmdClear()


def SetDeviceSN(api, str_):
    szPara = create_string_buffer(25)
    szPara.raw = str_.encode("utf-8")
    while True:
        result = api.SetDeviceSN(szPara)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetDeviceSN(api):
    szPara = create_string_buffer(25)
    while True:
        result = api.GetDeviceSN(szPara, 25)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    ret = szPara.value.decode("utf-8")
    output('GetDeviceSN: ' + ret)
    return ret


def SetDeviceName(api, str_):
    szPara = create_string_buffer(66)
    szPara.raw = str_.encode("utf-8")
    while True:
        result = api.SetDeviceName(szPara)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetDeviceName(api):
    szPara = create_string_buffer(66)
    while True:
        result = api.GetDeviceName(szPara, 100)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    ret = szPara.value.decode("utf-8")
    output('GetDeviceName: ' + ret)
    return ret


def GetDeviceVersion(api) -> List[int]:
    majorVersion = c_byte(0)
    minorVersion = c_byte(0)
    revision = c_byte(0)
    while True:
        result = api.GetDeviceVersion(byref(majorVersion), byref(minorVersion), byref(revision))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetDeviceVersion: V%d.%d.%d' % (majorVersion.value, minorVersion.value, revision.value))
    return [majorVersion.value, minorVersion.value, revision.value]


def SetDeviceWithL(api, isWithL):
    cIsWithL = c_bool(isWithL)
    while True:
        result = api.SetDeviceWithL(cIsWithL)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetDeviceWithL(api):
    isWithL = c_bool(False)
    while True:
        result = api.GetDeviceWithL(byref(isWithL))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return isWithL.value


def ResetPose(api, manual, rearArmAngle, frontArmAngle):
    c_rearArmAngle = c_float(rearArmAngle)
    c_frontArmAngle = c_float(frontArmAngle)
    while True:
        result = api.ResetPose(manual, c_rearArmAngle, c_frontArmAngle)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetPose(api) -> List[float]:
    pose = DobotTypes.Pose()
    while True:
        result = api.GetPose(byref(pose))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    # output('GetPose: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f' % (
    #     pose.x, pose.y, pose.z, pose.rHead, pose.joint1Angle, pose.joint2Angle, pose.joint3Angle, pose.joint4Angle))
    return [pose.x, pose.y, pose.z, pose.rHead, pose.joint1Angle, pose.joint2Angle, pose.joint3Angle, pose.joint4Angle]


def GetPoseL(api):
    l = c_float(0)
    while True:
        result = api.GetPoseL(byref(l))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [l.value]


def GetKinematics(api):
    kinematics = DobotTypes.Kinematics()
    while True:
        result = api.GetKinematics(byref(kinematics))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetKinematics: velocity=%.4f acceleration=%.4f' % (kinematics.velocity, kinematics.acceleration))
    return [kinematics.velocity, kinematics.acceleration]


def GetAlarmsState(api, maxLen=1000):
    alarmsState = create_string_buffer(maxLen)
    # alarmsState = c_byte(0)
    length = c_int(0)
    while True:
        result = api.GetAlarmsState(alarmsState, byref(length), maxLen)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    # output('GetAlarmsState: alarmsState=%.4f len=%.4f' %(alarmsState.value, len.value))
    return [alarmsState.raw, length.value]


def ClearAllAlarmsState(api):
    while True:
        result = api.ClearAllAlarmsState()
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetUserParams(api):
    param = DobotTypes.UserParams()
    while True:
        result = api.GetUserParams(byref(param))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetUserParams: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f' % (
        param.params1, param.params2, param.params3, param.params4, param.params5, param.params6, param.params7,
        param.params8))
    return [param.params1, param.params2, param.params3, param.params4, param.params5, param.params6, param.params7,
            param.params8]


def SetHOMEParams(api, x, y, z, r, isQueued=0):
    param = DobotTypes.HOMEParams()
    param.x = x
    param.y = y
    param.z = z
    param.r = r
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetHOMEParams(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetHOMEParams(api):
    param = DobotTypes.HOMEParams()
    while True:
        result = api.GetHOMEParams(byref(param))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetUserParams: %.4f' % param.temp)
    return [param.temp]


def SetHOMECmd(api, temp, isQueued=0):
    cmd = DobotTypes.HOMECmd()
    cmd.temp = temp
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetHOMECmd(byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def SetArmOrientation(api, armOrientation, isQueued=0):
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetArmOrientation(armOrientation, isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetArmOrientation(api):
    armOrientation = c_int32(0)
    while True:
        result = api.GetArmOrientation(byref(armOrientation))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetArmOrientation: armOrientation=%d' % armOrientation.value)
    return [armOrientation.value]


def SetHHTTrigMode(api, hhtTrigMode):
    while True:
        result = api.SetHHTTrigMode(hhtTrigMode)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetHHTTrigMode(api):
    hhtTrigMode = c_int(0)
    while True:
        result = api.GetHHTTrigMode(byref(hhtTrigMode))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [hhtTrigMode.value]


def SetHHTTrigOutputEnabled(api, isEnabled):
    while True:
        result = api.SetHHTTrigOutputEnabled(isEnabled)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetHHTTrigOutputEnabled(api):
    isEnabled = c_int32(0)
    while True:
        result = api.GetHHTTrigOutputEnabled(byref(isEnabled))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [isEnabled.value]


def GetHHTTrigOutput(api):
    isAvailable = c_int32(0)
    result = api.GetHHTTrigOutput(byref(isAvailable))
    if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully or isAvailable.value == 0:
        return False
    return True


def SetEndEffectorParams(api, xBias, yBias, zBias, isQueued=0):
    param = DobotTypes.EndTypeParams()
    param.xBias = xBias
    param.yBias = yBias
    param.zBias = zBias
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetEndEffectorParams(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetEndEffectorParams(api):
    param = DobotTypes.EndTypeParams()
    while True:
        result = api.GetEndEffectorParams(byref(param))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetEndEffectorParams: xBias=%.4f yBias=%.4f zBias=%.4f' % (param.xBias, param.yBias, param.zBias))
    return [param.xBias, param.yBias, param.zBias]


def SetEndEffectorLaser(api, enableCtrl, on, isQueued=0):
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetEndEffectorLaser(enableCtrl, on, isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetEndEffectorLaser(api):
    isCtrlEnabled = c_int(0)
    isOn = c_int(0)
    while True:
        result = api.GetEndEffectorLaser(byref(isCtrlEnabled), byref(isOn))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetEndEffectorLaser: isCtrlEnabled=%d, isOn=%4f' % (isCtrlEnabled.value, isOn.value))
    return [isCtrlEnabled.value, isOn.value]


def SetEndEffectorSuctionCup(api, enableCtrl, mode, isQueued=0):
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetEndEffectorSuctionCup(enableCtrl, mode, isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetEndEffectorSuctionCup(api):
    enableCtrl = c_int(0)
    isOn = c_int(0)
    while True:
        result = api.GetEndEffectorSuctionCup(byref(enableCtrl), byref(isOn))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetEndEffectorSuctionCup: isOn=%.4f' % isOn.value)
    return [isOn.value]


def SetEndEffectorGripper(api, enableCtrl, on, isQueued=0):
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetEndEffectorGripper(enableCtrl, on, isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetEndEffectorGripper(api):
    enableCtrl = c_int(0)
    isOn = c_int(0)
    while True:
        result = api.GetEndEffectorGripper(byref(enableCtrl), byref(isOn))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetEndEffectorGripper: isOn=%.4f' % isOn.value)
    return [isOn.value]


def SetJOGJointParams(api, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity, j3Acceleration,
                      j4Velocity, j4Acceleration, isQueued=0):
    jogParam = DobotTypes.JOGJointParams()
    jogParam.joint1Velocity = j1Velocity
    jogParam.joint1Acceleration = j1Acceleration
    jogParam.joint2Velocity = j2Velocity
    jogParam.joint2Acceleration = j2Acceleration
    jogParam.joint3Velocity = j3Velocity
    jogParam.joint3Acceleration = j3Acceleration
    jogParam.joint4Velocity = j4Velocity
    jogParam.joint4Acceleration = j4Acceleration
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetJOGJointParams(byref(jogParam), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetJOGJointParams(api):
    param = DobotTypes.JOGJointParams()
    while True:
        result = api.GetJOGJointParams(byref(param))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetJOGJointParams: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f' % (
        param.joint1Velocity, param.joint1Acceleration, param.joint2Velocity, param.joint2Acceleration,
        param.joint3Velocity, param.joint3Acceleration, param.joint4Velocity, param.joint4Acceleration))
    return [param.joint1Velocity, param.joint1Acceleration, param.joint2Velocity, param.joint2Acceleration,
            param.joint3Velocity, param.joint3Acceleration, param.joint4Velocity, param.joint4Acceleration]


def SetJOGCoordinateParams(api, xVelocity, xAcceleration, yVelocity, yAcceleration, zVelocity, zAcceleration, rVelocity,
                           rAcceleration, isQueued=0):
    param = DobotTypes.JOGCoordinateParams()
    param.xVelocity = xVelocity
    param.xAcceleration = xAcceleration
    param.yVelocity = yVelocity
    param.yAcceleration = yAcceleration
    param.zVelocity = zVelocity
    param.zAcceleration = zAcceleration
    param.rVelocity = rVelocity
    param.rAcceleration = rAcceleration
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetJOGCoordinateParams(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetJOGCoordinateParams(api):
    param = DobotTypes.JOGCoordinateParams()
    while True:
        result = api.GetJOGCoordinateParams(byref(param))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetJOGCoordinateParams: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f' % (
        param.xVelocity, param.xAcceleration, param.yVelocity, param.yVelocity, param.zVelocity, param.zAcceleration,
        param.rVelocity, param.rAcceleration))
    return [param.xVelocity, param.xAcceleration, param.yVelocity, param.yVelocity, param.zVelocity,
            param.zAcceleration, param.rVelocity, param.rAcceleration]


def SetJOGLParams(api, velocity, acceleration, isQueued=0):
    param = DobotTypes.JOGLParams()
    param.velocity = velocity
    param.acceleration = acceleration
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetJOGLParams(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetJOGLParams(api):
    param = DobotTypes.JOGLParams()
    while True:
        result = api.GetJOGLParams(byref(param))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [param.velocity, param.acceleration]


def SetJOGCommonParams(api, value_velocityratio, value_accelerationratio, isQueued=0):
    param = DobotTypes.JOGCommonParams()
    param.velocityRatio = value_velocityratio
    param.accelerationRatio = value_accelerationratio
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetJOGCommonParams(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetJOGCommonParams(api):
    param = DobotTypes.JOGCommonParams()
    while True:
        result = api.GetJOGCommonParams(byref(param))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetJOGCommonParams: velocityRatio=%.4f accelerationRatio=%.4f' % (
        param.velocityRatio, param.accelerationRatio))
    return [param.velocityRatio, param.accelerationRatio]


def SetJOGCmd(api, isJoint, cmd, isQueued=0):
    cmdParam = DobotTypes.JOGCmd()
    cmdParam.isJoint = isJoint
    cmdParam.cmd = cmd
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetJOGCmd(byref(cmdParam), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def SetPTPJointParams(api, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity, j3Acceleration,
                      j4Velocity, j4Acceleration, isQueued=0):
    pbParam = DobotTypes.PTPJointParams()
    pbParam.joint1Velocity = j1Velocity
    pbParam.joint1Acceleration = j1Acceleration
    pbParam.joint2Velocity = j2Velocity
    pbParam.joint2Acceleration = j2Acceleration
    pbParam.joint3Velocity = j3Velocity
    pbParam.joint3Acceleration = j3Acceleration
    pbParam.joint4Velocity = j4Velocity
    pbParam.joint4Acceleration = j4Acceleration
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetPTPJointParams(byref(pbParam), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetPTPJointParams(api):
    pbParam = DobotTypes.PTPJointParams()
    while True:
        result = api.GetPTPJointParams(byref(pbParam))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetPTPJointParams: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f' % (
        pbParam.joint1Velocity, pbParam.joint1Acceleration, pbParam.joint2Velocity, pbParam.joint2Acceleration,
        pbParam.joint3Velocity, pbParam.joint3Acceleration, pbParam.joint4Velocity, pbParam.joint4Acceleration))
    return [pbParam.joint1Velocity, pbParam.joint1Acceleration, pbParam.joint2Velocity, pbParam.joint2Acceleration,
            pbParam.joint3Velocity, pbParam.joint3Acceleration, pbParam.joint4Velocity, pbParam.joint4Acceleration]


def SetPTPCoordinateParams(api, xyzVelocity, xyzAcceleration, rVelocity, rAcceleration, isQueued=0):
    pbParam = DobotTypes.PTPCoordinateParams()
    pbParam.xyzVelocity = xyzVelocity
    pbParam.rVelocity = rVelocity
    pbParam.xyzAcceleration = xyzAcceleration
    pbParam.rAcceleration = rAcceleration
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetPTPCoordinateParams(byref(pbParam), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetPTPCoordinateParams(api):
    pbParam = DobotTypes.PTPCoordinateParams()
    while True:
        result = api.GetPTPCoordinateParams(byref(pbParam))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetPTPCoordinateParams: xyzVelocity=%.4f rVelocity=%.4f xyzAcceleration=%.4f rAcceleration=%.4f' % (
        pbParam.xyzVelocity, pbParam.rVelocity, pbParam.xyzAcceleration, pbParam.rAcceleration))
    return [pbParam.xyzVelocity, pbParam.rVelocity, pbParam.xyzAcceleration, pbParam.rAcceleration]


def SetPTPLParams(api, velocity, acceleration, isQueued=0):
    param = DobotTypes.PTPLParams()
    param.velocity = velocity
    param.acceleration = acceleration
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetPTPLParams(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetPTPLParams(api):
    param = DobotTypes.PTPLParams()
    while True:
        result = api.GetPTPLParams(byref(param))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [param.velocity, param.acceleration]


def SetPTPJumpParams(api, jumpHeight, zLimit, isQueued=0):
    pbParam = DobotTypes.PTPJumpParams()
    pbParam.jumpHeight = jumpHeight
    pbParam.zLimit = zLimit
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetPTPJumpParams(byref(pbParam), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetPTPJumpParams(api):
    pbParam = DobotTypes.PTPJumpParams()
    while True:
        result = api.GetPTPJumpParams(byref(pbParam))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetPTPJumpParams: jumpHeight=%.4f zLimit=%.4f' % (pbParam.jumpHeight, pbParam.zLimit))
    return [pbParam.jumpHeight, pbParam.zLimit]


def SetPTPCommonParams(api, velocityRatio, accelerationRatio, isQueued=0):
    pbParam = DobotTypes.PTPCommonParams()
    pbParam.velocityRatio = velocityRatio
    pbParam.accelerationRatio = accelerationRatio
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetPTPCommonParams(byref(pbParam), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetPTPCommonParams(api):
    pbParam = DobotTypes.PTPCommonParams()
    while True:
        result = api.GetPTPCommonParams(byref(pbParam))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetPTPCommonParams: velocityRatio=%.4f accelerationRatio=%.4f' % (
        pbParam.velocityRatio, pbParam.accelerationRatio))
    return [pbParam.velocityRatio, pbParam.accelerationRatio]


def SetPTPCmd(api, ptpMode, x, y, z, rHead, isQueued=0):
    cmd = DobotTypes.PTPCmd()
    cmd.ptpMode = ptpMode
    cmd.x = x
    cmd.y = y
    cmd.z = z
    cmd.rHead = rHead
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetPTPCmd(byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(2)
            continue
        break
    return [queuedCmdIndex.value]


def SetPTPWithLCmd(api, ptpMode, x, y, z, rHead, l, isQueued=0):
    cmd = DobotTypes.PTPWithLCmd()
    cmd.ptpMode = ptpMode
    cmd.x = x
    cmd.y = y
    cmd.z = z
    cmd.rHead = rHead
    cmd.l = l
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetPTPWithLCmd(byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(2)
            continue
        break
    return [queuedCmdIndex.value]


def SetCPParams(api, planAcc, juncitionVel, acc, realTimeTrack=0, isQueued=0):
    parm = DobotTypes.CPParams()
    parm.planAcc = planAcc
    parm.juncitionVel = juncitionVel
    parm.acc = acc
    parm.realTimeTrack = realTimeTrack
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetCPParams(byref(parm), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetCPParams(api):
    parm = DobotTypes.CPParams()
    while True:
        result = api.GetCPParams(byref(parm))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetCPParams: planAcc=%.4f juncitionVel=%.4f acc=%.4f' % (parm.planAcc, parm.juncitionVel, parm.acc))
    return [parm.planAcc, parm.juncitionVel, parm.acc]


def SetCPCmd(api, cpMode, x, y, z, velocity, isQueued=0):
    cmd = DobotTypes.CPCmd()
    cmd.cpMode = cpMode
    cmd.x = x
    cmd.y = y
    cmd.z = z
    cmd.velocity = velocity
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetCPCmd(byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(2)
            continue
        break
    return [queuedCmdIndex.value]


def SetCPLECmd(api, cpMode, x, y, z, power, isQueued=0):
    cmd = DobotTypes.CPCmd()
    cmd.cpMode = cpMode
    cmd.x = x
    cmd.y = y
    cmd.z = z
    cmd.velocity = power
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetCPLECmd(byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(2)
            continue
        break
    return [queuedCmdIndex.value]


def SetARCParams(api, xyzVelocity, rVelocity, xyzAcceleration, rAcceleration, isQueued=0):
    param = DobotTypes.ARCParams()
    param.xyzVelocity = xyzVelocity
    param.rVelocity = rVelocity
    param.xyzAcceleration = xyzAcceleration
    param.rAcceleration = rAcceleration
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetARCParams(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetARCParams(api):
    parm = DobotTypes.ARCParams()
    while True:
        result = api.GetARCParams(byref(parm))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetARCParams: xyzVelocity=%.4f,rVelocity=%.4f,xyzAcceleration=%.4f,rAcceleration=%.4f' % (
        parm.xyzVelocity, parm.rVelocity, parm.xyzAcceleration, parm.rAcceleration))
    return [parm.xyzVelocity, parm.rVelocity, parm.xyzAcceleration, parm.rAcceleration]


def SetARCCmd(api, cirPoint, toPoint, isQueued=0):
    cmd = DobotTypes.ARCCmd()
    cmd.cirPoint.x = cirPoint[0]
    cmd.cirPoint.y = cirPoint[1]
    cmd.cirPoint.z = cirPoint[2]
    cmd.cirPoint.rHead = cirPoint[3]
    cmd.toPoint.x = toPoint[0]
    cmd.toPoint.y = toPoint[1]
    cmd.toPoint.z = toPoint[2]
    cmd.toPoint.rHead = toPoint[3]
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetARCCmd(byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def SetWAITCmd(api, waitTimeMs, isQueued=1):
    param = DobotTypes.WAITCmd()
    param.waitTime = waitTimeMs
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetWAITCmd(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(10)
            continue
        break
    return [queuedCmdIndex.value]


def SetTRIGCmd(api, address, mode, condition, threshold, isQueued=0):
    param = DobotTypes.TRIGCmd()
    param.address = address
    param.mode = mode
    param.condition = condition
    param.threshold = threshold
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetTRIGCmd(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def SetIOMultiplexing(api, address, multiplex, isQueued=0):
    param = DobotTypes.IOMultiplexing()
    param.address = address
    param.multiplex = multiplex
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetIOMultiplexing(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetIOMultiplexing(api, addr):
    param = DobotTypes.IOMultiplexing()
    param.address = addr
    while True:
        result = api.GetIOMultiplexing(byref(param))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetIOMultiplexing: address=%.4f multiplex=%.4f' % (param.address, param.multiplex))
    return [param.multiplex]


def SetIODO(api, address, level, isQueued=0):
    param = DobotTypes.IODO()
    param.address = address
    param.level = level
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetIODO(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetIODO(api, addr):
    param = DobotTypes.IODO()
    param.address = addr
    while True:
        result = api.GetIODO(byref(param))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetIODO: address=%.4f level=%.4f' % (param.address, param.level))
    return [param.level]


def SetIOPWM(api, address, frequency, dutyCycle, isQueued=0):
    param = DobotTypes.IOPWM()
    param.address = address
    param.frequency = frequency
    param.dutyCycle = dutyCycle
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetIOPWM(byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetIOPWM(api, addr):
    param = DobotTypes.IOPWM()
    param.address = addr
    while True:
        result = api.GetIOPWM(byref(param))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetIOPWM: address=%.4f frequency=%.4f dutyCycle=%.4f' % (param.address, param.frequency, param.dutyCycle))
    return [param.frequency, param.dutyCycle]


def GetIODI(api, addr):
    param = DobotTypes.IODI()
    param.address = addr
    while True:
        result = api.GetIODI(byref(param))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetIODI: address=%d level=%d' % (param.address, param.level))
    return [param.level]


def SetEMotor(api, index, isEnabled, speed, isQueued=0):
    emotor = DobotTypes.EMotor()
    emotor.index = index
    emotor.isEnabled = isEnabled
    emotor.speed = speed
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetEMotor(byref(emotor), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def SetEMotorS(api, index, isEnabled, speed, distance, isQueued=0):
    emotors = DobotTypes.EMotorS()
    emotors.index = index
    emotors.isEnabled = isEnabled
    emotors.distance = distance
    emotors.speed = speed
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetEMotorS(byref(emotors), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetIOADC(api, addr):
    param = DobotTypes.IOADC()
    param.address = addr
    while True:
        result = api.GetIOADC(byref(param))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetIOADC: address=%.4f value=%.4f' % (param.address, param.value))
    return [param.value]


def SetColorSensor(api, isEnable, colorPort):
    enable = c_bool(isEnable)
    while True:
        result = api.SetColorSensor(enable, colorPort)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetColorSensor(api):
    r = c_ubyte(0)
    g = c_ubyte(0)
    b = c_ubyte(0)
    while True:
        result = api.GetColorSensor(byref(r), byref(g), byref(b))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    color = (r.value, g.value, b.value)
    output("got color sensor", color)
    return color


def SetInfraredSensor(api, isEnable, infraredPort):
    enable = c_bool(isEnable)
    while True:
        result = api.SetInfraredSensor(enable, infraredPort)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetInfraredSensor(api, infraredPort):
    val = c_ubyte(0)
    while True:
        result = api.GetInfraredSensor(infraredPort, byref(val))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return val.value


def SetAngleSensorStaticError(api, rearArmAngleError, frontArmAngleError):
    c_rearArmAngleError = c_float(rearArmAngleError)
    c_frontArmAngleError = c_float(frontArmAngleError)
    while True:
        result = api.SetAngleSensorStaticError(c_rearArmAngleError, c_frontArmAngleError)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetAngleSensorStaticError(api):
    rearArmAngleError = c_float(0)
    frontArmAngleError = c_float(0)
    while True:
        result = api.GetAngleSensorStaticError(byref(rearArmAngleError), byref(frontArmAngleError))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetAngleSensorStaticError: rearArmAngleError=%.4f,frontArmAngleError=%.4f' % (
        rearArmAngleError.value, frontArmAngleError.value))
    return [rearArmAngleError.value, frontArmAngleError.value]


def SetAngleSensorCoef(api, rearArmAngleCoef, frontArmAngleCoef):
    c_rearArmAngleCoef = c_float(rearArmAngleCoef)
    c_frontArmAngleCoef = c_float(frontArmAngleCoef)
    while True:
        result = api.SetAngleSensorCoef(c_rearArmAngleCoef, c_frontArmAngleCoef)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetAngleSensorCoef(api):
    rearArmAngleCoef = c_float(0)
    frontArmAngleCoef = c_float(0)
    while True:
        result = api.GetAngleSensorCoef(byref(rearArmAngleCoef), byref(frontArmAngleCoef))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetAngleSensorStaticCoef: rearArmAngleCoef=%.4f,frontArmAngleCoef=%.4f' % (
        rearArmAngleCoef.value, frontArmAngleCoef.value))
    return [rearArmAngleCoef.value, frontArmAngleCoef.value]


def SetBaseDecoderStaticError(api, baseDecoderError):
    c_baseDecoderError = c_float(baseDecoderError)
    while True:
        result = api.SetBaseDecoderStaticError(c_baseDecoderError)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetBaseDecoderStaticError(api):
    baseDecoderError = c_float(0)
    while True:
        result = api.GetBaseDecoderStaticError(byref(baseDecoderError))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetBaseDecoderStaticError: baseDecoderError=%.4f' % baseDecoderError.value)
    return [baseDecoderError.value]


# WIFI


def GetWIFIConnectStatus(api):
    isConnected = c_bool(0)
    while True:
        result = api.GetWIFIConnectStatus(byref(isConnected))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetWIFIConnectStatus: isConnected=%d' % isConnected.value)
    return isConnected.value


def SetWIFIConfigMode(api, enable):
    while True:
        result = api.SetWIFIConfigMode(enable)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetWIFIConfigMode(api):
    isEnabled = c_bool(0)
    while True:
        result = api.GetWIFIConfigMode(byref(isEnabled))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetWIFIConfigMode: isEnabled=%d' % isEnabled.value)
    return isEnabled.value


def SetWIFISSID(api, ssid):
    szPara = create_string_buffer(25)
    szPara.raw = ssid.encode("utf-8")
    while True:
        result = api.SetWIFISSID(szPara)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetWIFISSID(api):
    szPara = create_string_buffer(25)
    while True:
        result = api.GetWIFISSID(szPara, 25)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    ssid = szPara.value.decode("utf-8")
    output('GetWIFISSID: ssid=' + ssid)
    return ssid


def SetWIFIPassword(api, password):
    szPara = create_string_buffer(25)
    szPara.raw = password.encode("utf-8")
    while True:
        result = api.SetWIFIPassword(szPara)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetWIFIPassword(api):
    szPara = create_string_buffer(25)
    while True:
        result = api.GetWIFIPassword(szPara, 25)
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    password = szPara.value.decode("utf-8")
    output('GetWIFIPassword: password=' + password)
    return password


def SetWIFIIPAddress(api, dhcp, addr1, addr2, addr3, addr4):
    wifiIPAddress = DobotTypes.WIFIIPAddress()
    wifiIPAddress.dhcp = dhcp
    wifiIPAddress.addr1 = addr1
    wifiIPAddress.addr2 = addr2
    wifiIPAddress.addr3 = addr3
    wifiIPAddress.addr4 = addr4
    while True:
        result = api.SetWIFIIPAddress(byref(wifiIPAddress))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetWIFIIPAddress(api):
    wifiIPAddress = DobotTypes.WIFIIPAddress()
    while True:
        result = api.GetWIFIIPAddress(byref(wifiIPAddress))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetWIFIIPAddress: IPAddress=%d.%d.%d.%d' % (
        wifiIPAddress.addr1, wifiIPAddress.addr2, wifiIPAddress.addr3, wifiIPAddress.addr4))
    return [c_uint8(wifiIPAddress.dhcp).value, c_uint8(wifiIPAddress.addr1).value, c_uint8(wifiIPAddress.addr2).value,
            c_uint8(wifiIPAddress.addr3).value, c_uint8(wifiIPAddress.addr4).value]


def SetWIFINetmask(api, addr1, addr2, addr3, addr4):
    wifiNetmask = DobotTypes.WIFINetmask()
    wifiNetmask.addr1 = addr1
    wifiNetmask.addr2 = addr2
    wifiNetmask.addr3 = addr3
    wifiNetmask.addr4 = addr4
    while True:
        result = api.SetWIFINetmask(byref(wifiNetmask))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetWIFINetmask(api):
    wifiNetmask = DobotTypes.WIFINetmask()
    while True:
        result = api.GetWIFINetmask(byref(wifiNetmask))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetWIFINetmask: Netmask=%d.%d.%d.%d' % (
        wifiNetmask.addr1, wifiNetmask.addr2, wifiNetmask.addr3, wifiNetmask.addr4))
    return [c_uint8(wifiNetmask.addr1).value, c_uint8(wifiNetmask.addr2).value, c_uint8(wifiNetmask.addr3).value,
            c_uint8(wifiNetmask.addr4).value]


def SetWIFIGateway(api, addr1, addr2, addr3, addr4):
    wifiGateway = DobotTypes.WIFIGateway()
    wifiGateway.addr1 = addr1
    wifiGateway.addr2 = addr2
    wifiGateway.addr3 = addr3
    wifiGateway.addr4 = addr4
    while True:
        result = api.SetWIFIGateway(byref(wifiGateway))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetWIFIGateway(api):
    wifiGateway = DobotTypes.WIFIGateway()
    while True:
        result = api.GetWIFIGateway(byref(wifiGateway))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetWIFIGateway: wifiGateway=%d.%d.%d.%d' % (
        wifiGateway.addr1, wifiGateway.addr2, wifiGateway.addr3, wifiGateway.addr4))
    return [c_uint8(wifiGateway.addr1).value, c_uint8(wifiGateway.addr2).value, c_uint8(wifiGateway.addr3).value,
            c_uint8(wifiGateway.addr4).value]


def SetWIFIDNS(api, addr1, addr2, addr3, addr4):
    wifiDNS = DobotTypes.WIFIDNS()
    wifiDNS.addr1 = addr1
    wifiDNS.addr2 = addr2
    wifiDNS.addr3 = addr3
    wifiDNS.addr4 = addr4
    while True:
        result = api.SetWIFIDNS(byref(wifiDNS))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetWIFIDNS(api):
    wifiDNS = DobotTypes.WIFIDNS()
    while True:
        result = api.GetWIFIDNS(byref(wifiDNS))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    output('GetWIFIDNS: wifiDNS=%d.%d.%d.%d' % (wifiDNS.addr1, wifiDNS.addr2, wifiDNS.addr3, wifiDNS.addr4))
    return [c_uint8(wifiDNS.addr1).value, c_uint8(wifiDNS.addr2).value, c_uint8(wifiDNS.addr3).value,
            c_uint8(wifiDNS.addr4).value]


# FIRMWARE
def UpdateFirmware(api, firmwareParams: DobotTypes.FirmwareParams):
    while True:
        result = api.UpdateFirmware(byref(firmwareParams))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def SetFirmwareMode(api, firmwareMode):
    firmwareMode = c_uint(firmwareMode)
    while True:
        result = api.SetFirmwareMode(byref(firmwareMode))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetFirmwareMode(api):
    firmwareMode = c_uint(0)
    result = api.GetFirmwareMode(byref(firmwareMode))
    return [firmwareMode, result]


# LOSTSTEP
def SetLostStepParams(api, threshold, isQueued=0):
    threshold = c_float(threshold)
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetAutoLevelingCmd(threshold, isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def SetLostStepCmd(api, isQueued=1):
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetLostStepCmd(isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


# UART4 Peripherals
def GetUART4PeripheralsType(api, p_type):
    m_type = c_uint8(p_type)
    queuedCmdIndex = c_uint64(0)

    result = api.GetUART4PeripheralsType(byref(m_type), byref(queuedCmdIndex))

    return [m_type, result]


def SetUART4PeripheralsEnable(api, isEnable):
    enable = c_bool(isEnable)
    return api.SetUART4PeripheralsEnable(enable)


def GetUART4PeripheralsEnable(api):
    isEnable = c_bool()
    result = api.GetUART4PeripheralsEnable(byref(isEnable))
    return [isEnable, result]


# Function Pluse Mode
def SendPluse(api, pluseCmd: DobotTypes.PluseCmd, isQueued=0):
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SendPluse(byref(pluseCmd), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def SendPluseEx(api, pluseCmd):
    result = api.SendPluseEx(byref(pluseCmd))
    return result


def GetPTPTime(api, ptpCmd: DobotTypes.PTPCmd, ptpTime):
    m_ptpTime = c_uint32(ptpTime)
    while True:
        result = api.GetFirmwareMode(byref(ptpCmd), byref(m_ptpTime))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return m_ptpTime


def GetServoPIDParams(api):
    pid = DobotTypes.PID()
    while True:
        result = api.GetFirmwareMode(byref(pid))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return pid


def SetServoPIDParams(api, pid: DobotTypes.PID, isQueued=0):
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetServoPIDParams(byref(pid), isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def GetServoControlLoop(api):
    m_index = c_uint8()
    controlLoop = c_uint8()
    while True:
        result = api.GetServoControlLoop(m_index, byref(controlLoop))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break
    return [m_index, controlLoop]


def SetServoControlLoop(api, p_index, controlLoop, isQueued=0):
    m_index = c_uint8(p_index)
    controlLoop = c_uint8(controlLoop)
    queuedCmdIndex = c_uint64(0)

    while True:
        result = api.SetServoControlLoop(m_index, controlLoop, isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


def SaveServoPIDParams(api, p_index, controlLoop, isQueued=0):
    m_index = c_uint8(p_index)
    controlLoop = c_uint8(controlLoop)
    queuedCmdIndex = c_uint64(0)

    while True:
        result = api.SaveServoPIDParams(m_index, controlLoop, isQueued, byref(queuedCmdIndex))
        if result != DobotTypes.DobotCommunicate.DobotCommunicate_Successfully:
            dSleep(5)
            continue
        break


##################  Ex扩展函数，该套函数会检测每一条指令运行完毕  ##################
def GetPoseEx(api, index):
    if index == 0:
        ret = GetDeviceWithL(api)
        if not ret:
            print("Dobot is not in L model")
            return

        lr = GetPoseL(api)
        return round(lr[0], 4)

    pos = GetPose(api)
    return round(pos[index - 1], 4)


def SetHOMECmdEx(api, temp, isQueued=0):
    ret = SetHOMECmd(api, temp, isQueued)
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.GetQueuedCmdCurrentIndex(byref(queuedCmdIndex))
        if result == DobotTypes.DobotCommunicate.DobotCommunicate_Successfully and ret[0] <= queuedCmdIndex.value:
            break
        # 延时太短的话按reset键后不能断开连接
        dSleep(100)


def SetWAITCmdEx(api, waitTimeMs, isQueued=0):
    ret = SetWAITCmd(api, waitTimeMs, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break
        dSleep(waitTimeMs)


def SetEndEffectorParamsEx(api, xBias, yBias, zBias, isQueued=0):
    ret = SetEndEffectorParams(api, xBias, yBias, zBias, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break
        dSleep(5)


def SetPTPJointParamsEx(api, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity, j3Acceleration,
                        j4Velocity, j4Acceleration, isQueued=0):
    ret = SetPTPJointParams(api, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity, j3Acceleration,
                            j4Velocity, j4Acceleration, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break
        dSleep(5)


def SetPTPLParamsEx(api, lVelocity, lAcceleration, isQueued=0):
    ret = GetDeviceWithL(api)
    if not ret:
        print("Dobot is not in L model")
        return

    ret = SetPTPLParams(api, lVelocity, lAcceleration, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break
        dSleep(5)


def SetPTPCommonParamsEx(api, velocityRatio, accelerationRatio, isQueued=0):
    ret = SetPTPCommonParams(api, velocityRatio, accelerationRatio, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break
        dSleep(5)


def SetPTPJumpParamsEx(api, jumpHeight, maxJumpHeight, isQueued=0):
    ret = SetPTPJumpParams(api, jumpHeight, maxJumpHeight, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break
        dSleep(5)


def SetPTPCmdEx(api, ptpMode, x, y, z, rHead, isQueued=0):
    ret = SetPTPCmd(api, ptpMode, x, y, z, rHead, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break
        dSleep(5)


def SetIOMultiplexingEx(api, address, multiplex, isQueued=0):
    ret = SetIOMultiplexing(api, address, multiplex, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break
        dSleep(5)


def SetEndEffectorSuctionCupEx(api, enableCtrl, on, isQueued=0):
    ret = SetEndEffectorSuctionCup(api, enableCtrl, on, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break
        dSleep(5)


def SetEndEffectorGripperEx(api, enableCtrl, on, isQueued=0):
    ret = SetEndEffectorGripper(api, enableCtrl, on, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break
        dSleep(5)


def SetIODOEx(api, address, level, isQueued=0):
    ret = SetIODO(api, address, level, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break
        dSleep(5)


def SetEMotorEx(api, index, isEnabled, speed, isQueued=0):
    ret = SetEMotor(api, index, isEnabled, speed, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break
        dSleep(5)


def SetEMotorSEx(api, index, isEnabled, speed, distance, isQueued=0):
    ret = SetEMotorS(api, index, isEnabled, speed, distance, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break
        dSleep(5)


def SetIOPWMEx(api, address, frequency, dutyCycle, isQueued=0):
    while True:
        ret = SetIOPWM(api, address, frequency, dutyCycle, isQueued)
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break
        dSleep(5)


def SetPTPWithLCmdEx(api, ptpMode, x, y, z, rHead, l, isQueued=0):
    ret = GetDeviceWithL(api)
    if not ret:
        output("Dobot is not in L model")
        return

    ret = SetPTPWithLCmd(api, ptpMode, x, y, z, rHead, l, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api)[0]:
            break
        dSleep(5)


def GetColorSensorEx(api, index):
    result = GetColorSensor(api)
    return result[index]
