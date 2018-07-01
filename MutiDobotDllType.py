from DobotDllType import *


##################  api, dobotId func   ##################


def DisconnectDobot(api, dobotId):
    api.DisconnectDobot(dobotId)


def SetCmdTimeout(api, dobotId, times):
    api.SetCmdTimeout(dobotId, times)


def DobotExec(api):
    return [api.DobotExec()]


def GetQueuedCmdCurrentIndex(api, dobotId):
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.GetQueuedCmdCurrentIndex(dobotId, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(2)
            continue
        break
    return [queuedCmdIndex.value]


def SetQueuedCmdStartExec(api, dobotId):
    while True:
        result = api.SetQueuedCmdStartExec(dobotId)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def SetQueuedCmdStopExec(api, dobotId):
    while True:
        result = api.SetQueuedCmdStopExec(dobotId)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def SetQueuedCmdForceStopExec(api, dobotId):
    while True:
        result = api.SetQueuedCmdForceStopExec(dobotId)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def SetQueuedCmdStartDownload(api, dobotId, totalLoop, linePerLoop):
    while True:
        result = api.SetQueuedCmdStartDownload(dobotId, totalLoop, linePerLoop)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def SetQueuedCmdStopDownload(api, dobotId):
    while True:
        result = api.SetQueuedCmdStopDownload(dobotId)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def SetQueuedCmdClear(api, dobotId):
    return [api.SetQueuedCmdClear(dobotId)]


def SetDeviceSN(api, dobotId, str):
    szPara = create_string_buffer(25)
    szPara.raw = str.encode("utf-8")
    while True:
        result = api.SetDeviceSN(dobotId, szPara)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetDeviceSN(api, dobotId):
    szPara = create_string_buffer(25)
    while True:
        result = api.GetDeviceSN(dobotId, szPara, 25)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    ret = szPara.value.decode("utf-8")
    output('GetDeviceSN: ' + ret)
    return [ret]


def SetDeviceName(api, dobotId, str):
    szPara = create_string_buffer(len(str) * 4)
    szPara.raw = str.encode("utf-8")
    while True:
        result = api.SetDeviceName(dobotId, szPara)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def SetDeviceNumName(api, dobotId, num):
    cNum = c_int(num)
    while True:
        result = api.SetDeviceName(dobotId, cNum)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetDeviceName(api, dobotId):
    szPara = create_string_buffer(66)
    while True:
        result = api.GetDeviceName(dobotId, szPara, 100)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    ret = szPara.value.decode("utf-8")
    output('GetDeviceName: ' + ret)
    return [ret]


def GetDeviceVersion(api, dobotId):
    majorVersion = c_byte(0)
    minorVersion = c_byte(0)
    revision = c_byte(0)
    while True:
        result = api.GetDeviceVersion(dobotId, byref(majorVersion), byref(minorVersion), byref(revision))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetDeviceVersion: V%d.%d.%d' % (majorVersion.value, minorVersion.value, revision.value))
    return [majorVersion.value, minorVersion.value, revision.value]


def SetDeviceWithL(api, dobotId, isWithL):
    cIsWithL = c_bool(isWithL)
    while True:
        result = api.SetDeviceWithL(dobotId, cIsWithL)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetDeviceWithL(api, dobotId):
    isWithL = c_bool(False)
    while True:
        result = api.GetDeviceWithL(dobotId, byref(isWithL))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [isWithL.value]


def GetDeviceTime(api, dobotId):
    time = c_uint32(0)
    while True:
        result = api.GetDeviceTime(dobotId, byref(time))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [time.value]


def ResetPose(api, dobotId, manual, rearArmAngle, frontArmAngle):
    c_rearArmAngle = c_float(rearArmAngle)
    c_frontArmAngle = c_float(frontArmAngle)
    while True:
        result = api.ResetPose(dobotId, manual, c_rearArmAngle, c_frontArmAngle)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetPose(api, dobotId):
    pose = Pose()
    while True:
        result = api.GetPose(dobotId, byref(pose))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetPose: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f' % (
        pose.x, pose.y, pose.z, pose.rHead, pose.joint1Angle, pose.joint2Angle, pose.joint3Angle, pose.joint4Angle))
    return [pose.x, pose.y, pose.z, pose.rHead, pose.joint1Angle, pose.joint2Angle, pose.joint3Angle, pose.joint4Angle]


def GetPoseL(api, dobotId):
    l = c_float(0)
    while True:
        result = api.GetPoseL(dobotId, byref(l))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [l.value]


def GetKinematics(api, dobotId):
    kinematics = Kinematics()
    while True:
        result = api.GetKinematics(dobotId, byref(kinematics))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetKinematics: velocity=%.4f acceleration=%.4f' % (kinematics.velocity, kinematics.acceleration))
    return [kinematics.velocity, kinematics.acceleration]


def GetAlarmsState(api, dobotId, maxLen=1000):
    alarmsState = create_string_buffer(maxLen)
    # alarmsState = c_byte(0)
    len = c_int(0)
    while True:
        result = api.GetAlarmsState(dobotId, alarmsState, byref(len), maxLen)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    # output('GetAlarmsState: alarmsState=%.4f len=%.4f' %(alarmsState.value, len.value))
    return [alarmsState.raw, len.value]


def ClearAllAlarmsState(api, dobotId):
    while True:
        result = api.ClearAllAlarmsState(dobotId)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetUserParams(api, dobotId):
    param = UserParams()
    while True:
        result = api.GetUserParams(dobotId, byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetUserParams: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f' % (
        param.params1, param.params2, param.params3, param.params4, param.params5, param.params6, param.params7,
        param.params8))
    return [param.params1, param.params2, param.params3, param.params4, param.params5, param.params6, param.params7,
            param.params8]


def SetHOMEParams(api, dobotId, x, y, z, r, isQueued=0):
    param = HOMEParams()
    param.x = x
    param.y = y
    param.z = z
    param.r = r
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetHOMEParams(dobotId, byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetHOMEParams(api, dobotId):
    param = HOMEParams()
    while True:
        result = api.GetHOMEParams(dobotId, byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetUserParams: %.4f' % (param.temp))
    return [param.temp]


def SetHOMECmd(api, dobotId, temp, isQueued=0):
    cmd = HOMECmd()
    cmd.temp = temp
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetHOMECmd(dobotId, byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def SetAutoLevelingCmd(api, dobotId, controlFlag, precision, isQueued=0):
    cmd = AutoLevelingCmd()
    cmd.controlFlag = controlFlag
    cmd.precision = precision
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetAutoLevelingCmd(dobotId, byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetAutoLevelingResult(api, dobotId):
    precision = c_float(0)
    while True:
        result = api.GetAutoLevelingResult(dobotId, byref(precision))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    print("GetAutoLevelingResult", precision, precision.value)
    output('GetAutoLevelingResult: precision=%d' % (precision.value))
    return [precision.value]


def SetArmOrientation(api, dobotId, armOrientation, isQueued=0):
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetArmOrientation(dobotId, armOrientation, isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetArmOrientation(api, dobotId):
    armOrientation = c_int32(0)
    while True:
        result = api.GetArmOrientation(dobotId, byref(armOrientation))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetArmOrientation: armOrientation=%d' % (armOrientation.value))
    return [armOrientation.value]


def SetHHTTrigMode(api, dobotId, hhtTrigMode):
    while True:
        result = api.SetHHTTrigMode(dobotId, hhtTrigMode)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetHHTTrigMode(api, dobotId):
    hhtTrigMode = c_int(0)
    while True:
        result = api.GetHHTTrigMode(dobotId, byref(hhtTrigMode))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [hhtTrigMode.value]


def SetHHTTrigOutputEnabled(api, dobotId, isEnabled):
    while True:
        result = api.SetHHTTrigOutputEnabled(dobotId, isEnabled)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetHHTTrigOutputEnabled(api, dobotId):
    isEnabled = c_int32(0)
    while True:
        result = api.GetHHTTrigOutputEnabled(dobotId, byref(isEnabled))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [isEnabled.value]


def GetHHTTrigOutput(api, dobotId):
    isAvailable = c_int32(0)
    result = api.GetHHTTrigOutput(dobotId, byref(isAvailable))
    if result != DobotCommunicate.DobotCommunicate_NoError or isAvailable.value == 0:
        return [False]
    return [True]


def SetEndEffectorParams(api, dobotId, xBias, yBias, zBias, isQueued=0):
    param = EndTypeParams()
    param.xBias = xBias
    param.yBias = yBias
    param.zBias = zBias
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetEndEffectorParams(dobotId, byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetEndEffectorParams(api, dobotId):
    param = EndTypeParams()
    while True:
        result = api.GetEndEffectorParams(dobotId, byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetEndEffectorParams: xBias=%.4f yBias=%.4f zBias=%.4f' % (param.xBias, param.yBias, param.zBias))
    return [param.xBias, param.yBias, param.zBias]


def SetEndEffectorLaser(api, dobotId, enableCtrl, on, isQueued=0):
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetEndEffectorLaser(dobotId, enableCtrl, on, isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetEndEffectorLaser(api, dobotId):
    isCtrlEnabled = c_int(0)
    isOn = c_int(0)
    while True:
        result = api.GetEndEffectorLaser(dobotId, byref(isCtrlEnabled), byref(isOn))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetEndEffectorLaser: isCtrlEnabled=%d, isOn=%4f' % (isCtrlEnabled.value, isOn.value))
    return [isCtrlEnabled.value, isOn.value]


def SetEndEffectorSuctionCup(api, dobotId, enableCtrl, on, isQueued=0):
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetEndEffectorSuctionCup(dobotId, enableCtrl, on, isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetEndEffectorSuctionCup(api, dobotId):
    enableCtrl = c_int(0)
    isOn = c_int(0)
    while True:
        result = api.GetEndEffectorSuctionCup(dobotId, byref(enableCtrl), byref(isOn))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetEndEffectorSuctionCup: isOn=%.4f' % (isOn.value))
    return [isOn.value]


def SetEndEffectorGripper(api, dobotId, enableCtrl, on, isQueued=0):
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetEndEffectorGripper(dobotId, enableCtrl, on, isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetEndEffectorGripper(api, dobotId):
    enableCtrl = c_int(0)
    isOn = c_int(0)
    while True:
        result = api.GetEndEffectorGripper(dobotId, byref(enableCtrl), byref(isOn))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetEndEffectorGripper: isOn=%.4f' % (isOn.value))
    return [isOn.value]


def SetJOGJointParams(api, dobotId, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity, j3Acceleration,
                      j4Velocity, j4Acceleration, isQueued=0):
    jogParam = JOGJointParams()
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
        result = api.SetJOGJointParams(dobotId, byref(jogParam), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetJOGJointParams(api, dobotId):
    param = JOGJointParams()
    while True:
        result = api.GetJOGJointParams(dobotId, byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetJOGJointParams: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f' % (
        param.joint1Velocity, param.joint1Acceleration, param.joint2Velocity, param.joint2Acceleration,
        param.joint3Velocity, param.joint3Acceleration, param.joint4Velocity, param.joint4Acceleration))
    return [param.joint1Velocity, param.joint1Acceleration, param.joint2Velocity, param.joint2Acceleration,
            param.joint3Velocity, param.joint3Acceleration, param.joint4Velocity, param.joint4Acceleration]


def SetJOGCoordinateParams(api, dobotId, xVelocity, xAcceleration, yVelocity, yAcceleration, zVelocity, zAcceleration,
                           rVelocity, rAcceleration, isQueued=0):
    param = JOGCoordinateParams()
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
        result = api.SetJOGCoordinateParams(dobotId, byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetJOGCoordinateParams(api, dobotId):
    param = JOGCoordinateParams()
    while True:
        result = api.GetJOGCoordinateParams(dobotId, byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetJOGCoordinateParams: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f' % (
        param.xVelocity, param.xAcceleration, param.yVelocity, param.yVelocity, param.zVelocity, param.zAcceleration,
        param.rVelocity, param.rAcceleration))
    return [param.xVelocity, param.xAcceleration, param.yVelocity, param.yVelocity, param.zVelocity,
            param.zAcceleration, param.rVelocity, param.rAcceleration]


def SetJOGLParams(api, dobotId, velocity, acceleration, isQueued=0):
    param = JOGLParams()
    param.velocity = velocity
    param.acceleration = acceleration
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetJOGLParams(dobotId, byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetJOGLParams(api, dobotId):
    param = JOGLParams()
    while True:
        result = api.GetJOGLParams(dobotId, byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [param.velocity, param.acceleration]


def SetJOGCommonParams(api, dobotId, value_velocityratio, value_accelerationratio, isQueued=0):
    param = JOGCommonParams()
    param.velocityRatio = value_velocityratio
    param.accelerationRatio = value_accelerationratio
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetJOGCommonParams(dobotId, byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetJOGCommonParams(api, dobotId):
    param = JOGCommonParams()
    while True:
        result = api.GetJOGCommonParams(dobotId, byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetJOGCommonParams: velocityRatio=%.4f accelerationRatio=%.4f' % (
        param.velocityRatio, param.accelerationRatio))
    return [param.velocityRatio, param.accelerationRatio]


def SetJOGCmd(api, dobotId, isJoint, cmd, isQueued=0):
    cmdParam = JOGCmd()
    cmdParam.isJoint = isJoint
    cmdParam.cmd = cmd
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetJOGCmd(dobotId, byref(cmdParam), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def SetPTPJointParams(api, dobotId, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity, j3Acceleration,
                      j4Velocity, j4Acceleration, isQueued=0):
    pbParam = PTPJointParams()
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
        result = api.SetPTPJointParams(dobotId, byref(pbParam), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetPTPJointParams(api, dobotId):
    pbParam = PTPJointParams()
    while True:
        result = api.GetPTPJointParams(dobotId, byref(pbParam))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetPTPJointParams: %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f' % (
        pbParam.joint1Velocity, pbParam.joint1Acceleration, pbParam.joint2Velocity, pbParam.joint2Acceleration,
        pbParam.joint3Velocity, pbParam.joint3Acceleration, pbParam.joint4Velocity, pbParam.joint4Acceleration))
    return [pbParam.joint1Velocity, pbParam.joint1Acceleration, pbParam.joint2Velocity, pbParam.joint2Acceleration,
            pbParam.joint3Velocity, pbParam.joint3Acceleration, pbParam.joint4Velocity, pbParam.joint4Acceleration]


def SetPTPCoordinateParams(api, dobotId, xyzVelocity, xyzAcceleration, rVelocity, rAcceleration, isQueued=0):
    pbParam = PTPCoordinateParams()
    pbParam.xyzVelocity = xyzVelocity
    pbParam.rVelocity = rVelocity
    pbParam.xyzAcceleration = xyzAcceleration
    pbParam.rAcceleration = rAcceleration
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetPTPCoordinateParams(dobotId, byref(pbParam), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetPTPCoordinateParams(api, dobotId):
    pbParam = PTPCoordinateParams()
    while True:
        result = api.GetPTPCoordinateParams(dobotId, byref(pbParam))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetPTPCoordinateParams: xyzVelocity=%.4f rVelocity=%.4f xyzAcceleration=%.4f rAcceleration=%.4f' % (
        pbParam.xyzVelocity, pbParam.rVelocity, pbParam.xyzAcceleration, pbParam.rAcceleration))
    return [pbParam.xyzVelocity, pbParam.rVelocity, pbParam.xyzAcceleration, pbParam.rAcceleration]


def SetPTPLParams(api, dobotId, velocity, acceleration, isQueued=0):
    param = PTPLParams()
    param.velocity = velocity
    param.acceleration = acceleration
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetPTPLParams(dobotId, byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetPTPLParams(api, dobotId):
    param = PTPLParams()
    while True:
        result = api.GetPTPLParams(dobotId, byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [param.velocity, param.acceleration]


def SetPTPJumpParams(api, dobotId, jumpHeight, zLimit, isQueued=0):
    pbParam = PTPJumpParams()
    pbParam.jumpHeight = jumpHeight
    pbParam.zLimit = zLimit
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetPTPJumpParams(dobotId, byref(pbParam), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetPTPJumpParams(api, dobotId):
    pbParam = PTPJumpParams()
    while True:
        result = api.GetPTPJumpParams(dobotId, byref(pbParam))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetPTPJumpParams: jumpHeight=%.4f zLimit=%.4f' % (pbParam.jumpHeight, pbParam.zLimit))
    return [pbParam.jumpHeight, pbParam.zLimit]


def SetPTPCommonParams(api, dobotId, velocityRatio, accelerationRatio, isQueued=0):
    pbParam = PTPCommonParams()
    pbParam.velocityRatio = velocityRatio
    pbParam.accelerationRatio = accelerationRatio
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetPTPCommonParams(dobotId, byref(pbParam), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetPTPCommonParams(api, dobotId):
    pbParam = PTPCommonParams()
    while True:
        result = api.GetPTPCommonParams(dobotId, byref(pbParam))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetPTPCommonParams: velocityRatio=%.4f accelerationRatio=%.4f' % (
        pbParam.velocityRatio, pbParam.accelerationRatio))
    return [pbParam.velocityRatio, pbParam.accelerationRatio]


def SetPTPCmd(api, dobotId, ptpMode, x, y, z, rHead, isQueued=0):
    cmd = PTPCmd()
    cmd.ptpMode = ptpMode
    cmd.x = x
    cmd.y = y
    cmd.z = z
    cmd.rHead = rHead
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetPTPCmd(dobotId, byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(2)
            continue
        break
    return [queuedCmdIndex.value]


def SetPTPWithLCmd(api, dobotId, ptpMode, x, y, z, rHead, l, isQueued=0):
    cmd = PTPWithLCmd()
    cmd.ptpMode = ptpMode
    cmd.x = x
    cmd.y = y
    cmd.z = z
    cmd.rHead = rHead
    cmd.l = l
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetPTPWithLCmd(dobotId, byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(2)
            continue
        break
    return [queuedCmdIndex.value]


def SetCPParams(api, dobotId, planAcc, juncitionVel, acc, realTimeTrack=0, isQueued=0):
    parm = CPParams()
    parm.planAcc = planAcc
    parm.juncitionVel = juncitionVel
    parm.acc = acc
    parm.realTimeTrack = realTimeTrack
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetCPParams(dobotId, byref(parm), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetCPParams(api, dobotId):
    parm = CPParams()
    while True:
        result = api.GetCPParams(dobotId, byref(parm))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetCPParams: planAcc=%.4f juncitionVel=%.4f acc=%.4f' % (parm.planAcc, parm.juncitionVel, parm.acc))
    return [parm.planAcc, parm.juncitionVel, parm.acc]


def SetCPCmd(api, dobotId, cpMode, x, y, z, velocity, isQueued=0):
    cmd = CPCmd()
    cmd.cpMode = cpMode
    cmd.x = x
    cmd.y = y
    cmd.z = z
    cmd.velocity = velocity
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetCPCmd(dobotId, byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(2)
            continue
        break
    return [queuedCmdIndex.value]


def SetCPLECmd(api, dobotId, cpMode, x, y, z, power, isQueued=0):
    cmd = CPCmd()
    cmd.cpMode = cpMode
    cmd.x = x
    cmd.y = y
    cmd.z = z
    cmd.velocity = power
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetCPLECmd(dobotId, byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(2)
            continue
        break
    return [queuedCmdIndex.value]


def SetARCParams(api, dobotId, xyzVelocity, rVelocity, xyzAcceleration, rAcceleration, isQueued=0):
    param = ARCParams()
    param.xyzVelocity = xyzVelocity
    param.rVelocity = rVelocity
    param.xyzAcceleration = xyzAcceleration
    param.rAcceleration = rAcceleration
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetARCParams(dobotId, byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetARCParams(api, dobotId):
    parm = ARCParams()
    while True:
        result = api.GetARCParams(dobotId, byref(parm))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetARCParams: xyzVelocity=%.4f,rVelocity=%.4f,xyzAcceleration=%.4f,rAcceleration=%.4f' % (
        parm.xyzVelocity, parm.rVelocity, parm.xyzAcceleration, parm.rAcceleration))
    return [parm.xyzVelocity, parm.rVelocity, parm.xyzAcceleration, parm.rAcceleration]


def SetARCCmd(api, dobotId, cirPoint, toPoint, isQueued=0):
    cmd = ARCCmd()
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
        result = api.SetARCCmd(dobotId, byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


class CircleCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("cirPoint", ARCPoint),
        ("toPoint", ARCPoint)
    ]


def SetCircleCmd(api, dobotId, cirPoint, toPoint, isQueued=0):
    cmd = CircleCmd()
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
        result = api.SetCircleCmd(dobotId, byref(cmd), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def SetWAITCmd(api, dobotId, waitTime, isQueued=0):
    param = WAITCmd()
    param.waitTime = int(waitTime)
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetWAITCmd(dobotId, byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def SetTRIGCmd(api, dobotId, address, mode, condition, threshold, isQueued=0):
    param = TRIGCmd()
    param.address = address
    param.mode = mode
    param.condition = condition
    param.threshold = threshold
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetTRIGCmd(dobotId, byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def SetIOMultiplexing(api, dobotId, address, multiplex, isQueued=0):
    param = IOMultiplexing()
    param.address = address
    param.multiplex = multiplex
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetIOMultiplexing(dobotId, byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetIOMultiplexing(api, dobotId, addr):
    param = IOMultiplexing()
    param.address = addr
    while True:
        result = api.GetIOMultiplexing(dobotId, byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetIOMultiplexing: address=%.4f multiplex=%.4f' % (param.address, param.multiplex))
    return [param.multiplex]


def SetIODO(api, dobotId, address, level, isQueued=0):
    param = IODO()
    param.address = address
    param.level = level
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetIODO(dobotId, byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetIODO(api, dobotId, addr):
    param = IODO()
    param.address = addr
    while True:
        result = api.GetIODO(dobotId, byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetIODO: address=%.4f level=%.4f' % (param.address, param.level))
    return [param.level]


def SetIOPWM(api, dobotId, address, frequency, dutyCycle, isQueued=0):
    param = IOPWM()
    param.address = address
    param.frequency = frequency
    param.dutyCycle = dutyCycle
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetIOPWM(dobotId, byref(param), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetIOPWM(api, dobotId, addr):
    param = IOPWM()
    param.address = addr
    while True:
        result = api.GetIOPWM(dobotId, byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetIOPWM: address=%.4f frequency=%.4f dutyCycle=%.4f' % (param.address, param.frequency, param.dutyCycle))
    return [param.frequency, param.dutyCycle]


def GetIODI(api, dobotId, addr):
    param = IODI()
    param.address = addr
    while True:
        result = api.GetIODI(dobotId, byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetIODI: address=%d level=%d' % (param.address, param.level))
    return [param.level]


def SetEMotor(api, dobotId, index, isEnabled, speed, isQueued=0):
    emotor = EMotor()
    emotor.index = index
    emotor.isEnabled = isEnabled
    emotor.speed = speed
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetEMotor(dobotId, byref(emotor), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def SetEMotorS(api, dobotId, index, isEnabled, speed, distance, isQueued=0):
    emotorS = EMotorS()
    emotorS.index = index
    emotorS.isEnabled = isEnabled
    emotorS.speed = speed
    emotorS.distance = distance
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetEMotorS(dobotId, byref(emotorS), isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


def GetIOADC(api, dobotId, addr):
    param = IOADC()
    param.address = addr
    while True:
        result = api.GetIOADC(dobotId, byref(param))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetIOADC: address=%.4f value=%.4f' % (param.address, param.value))
    return [param.value]


def SetAngleSensorStaticError(api, dobotId, rearArmAngleError, frontArmAngleError):
    c_rearArmAngleError = c_float(rearArmAngleError)
    c_frontArmAngleError = c_float(frontArmAngleError)
    while True:
        result = api.SetAngleSensorStaticError(dobotId, c_rearArmAngleError, c_frontArmAngleError)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetAngleSensorStaticError(api, dobotId):
    rearArmAngleError = c_float(0)
    frontArmAngleError = c_float(0)
    while True:
        result = api.GetAngleSensorStaticError(dobotId, byref(rearArmAngleError), byref(frontArmAngleError))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetAngleSensorStaticError: rearArmAngleError=%.4f,frontArmAngleError=%.4f' % (
        rearArmAngleError.value, frontArmAngleError.value))
    return [rearArmAngleError.value, frontArmAngleError.value]


def SetAngleSensorCoef(api, dobotId, rearArmAngleCoef, frontArmAngleCoef):
    c_rearArmAngleCoef = c_float(rearArmAngleCoef)
    c_frontArmAngleCoef = c_float(frontArmAngleCoef)
    while True:
        result = api.SetAngleSensorCoef(dobotId, c_rearArmAngleCoef, c_frontArmAngleCoef)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetAngleSensorCoef(api, dobotId):
    rearArmAngleCoef = c_float(0)
    frontArmAngleCoef = c_float(0)
    while True:
        result = api.GetAngleSensorCoef(dobotId, byref(rearArmAngleCoef), byref(frontArmAngleCoef))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetAngleSensorStaticCoef: rearArmAngleCoef=%.4f,frontArmAngleCoef=%.4f' % (
        rearArmAngleCoef.value, frontArmAngleCoef.value))
    return [rearArmAngleCoef.value, frontArmAngleCoef.value]


def SetBaseDecoderStaticError(api, dobotId, baseDecoderError):
    c_baseDecoderError = c_float(baseDecoderError)
    while True:
        result = api.SetBaseDecoderStaticError(dobotId, c_baseDecoderError)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetBaseDecoderStaticError(api, dobotId):
    baseDecoderError = c_float(0)
    while True:
        result = api.GetBaseDecoderStaticError(dobotId, byref(baseDecoderError))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetBaseDecoderStaticError: baseDecoderError=%.4f' % (baseDecoderError.value))
    return [baseDecoderError.value]


"""
WIFI
JoMar 
20160906
"""


def GetWIFIConnectStatus(api, dobotId):
    isConnected = c_bool(0)
    while True:
        result = api.GetWIFIConnectStatus(dobotId, byref(isConnected))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetWIFIConnectStatus: isConnected=%d' % (isConnected.value))
    return [isConnected.value]


def SetWIFIConfigMode(api, dobotId, enable):
    while True:
        result = api.SetWIFIConfigMode(dobotId, enable)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetWIFIConfigMode(api, dobotId):
    isEnabled = c_bool(0)
    while True:
        result = api.GetWIFIConfigMode(dobotId, byref(isEnabled))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetWIFIConfigMode: isEnabled=%d' % (isEnabled.value))
    return [isEnabled.value]


def SetWIFISSID(api, dobotId, ssid):
    szPara = create_string_buffer(25)
    szPara.raw = ssid.encode("utf-8")
    while True:
        result = api.SetWIFISSID(dobotId, szPara)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetWIFISSID(api, dobotId):
    szPara = create_string_buffer(25)
    while True:
        result = api.GetWIFISSID(dobotId, szPara, 25)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    ssid = szPara.value.decode("utf-8")
    output('GetWIFISSID: ssid=' + ssid)
    return [ssid]


def SetWIFIPassword(api, dobotId, password):
    szPara = create_string_buffer(25)
    szPara.raw = password.encode("utf-8")
    while True:
        result = api.SetWIFIPassword(dobotId, szPara)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetWIFIPassword(api, dobotId):
    szPara = create_string_buffer(25)
    while True:
        result = api.GetWIFIPassword(dobotId, szPara, 25)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    password = szPara.value.decode("utf-8")
    output('GetWIFIPassword: password=' + password)
    return [password]


def SetWIFIIPAddress(api, dobotId, dhcp, addr1, addr2, addr3, addr4):
    wifiIPAddress = WIFIIPAddress()
    wifiIPAddress.dhcp = dhcp
    wifiIPAddress.addr1 = addr1
    wifiIPAddress.addr2 = addr2
    wifiIPAddress.addr3 = addr3
    wifiIPAddress.addr4 = addr4
    while True:
        result = api.SetWIFIIPAddress(dobotId, byref(wifiIPAddress))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetWIFIIPAddress(api, dobotId):
    wifiIPAddress = WIFIIPAddress()
    while True:
        result = api.GetWIFIIPAddress(dobotId, byref(wifiIPAddress))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetWIFIIPAddress: IPAddress=%d.%d.%d.%d' % (
        wifiIPAddress.addr1, wifiIPAddress.addr2, wifiIPAddress.addr3, wifiIPAddress.addr4))
    return [c_uint8(wifiIPAddress.dhcp).value, c_uint8(wifiIPAddress.addr1).value, c_uint8(wifiIPAddress.addr2).value,
            c_uint8(wifiIPAddress.addr3).value, c_uint8(wifiIPAddress.addr4).value]


def SetWIFINetmask(api, dobotId, addr1, addr2, addr3, addr4):
    wifiNetmask = WIFINetmask()
    wifiNetmask.addr1 = addr1
    wifiNetmask.addr2 = addr2
    wifiNetmask.addr3 = addr3
    wifiNetmask.addr4 = addr4
    while True:
        result = api.SetWIFINetmask(dobotId, byref(wifiNetmask))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetWIFINetmask(api, dobotId):
    wifiNetmask = WIFINetmask()
    while True:
        result = api.GetWIFINetmask(dobotId, byref(wifiNetmask))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetWIFINetmask: Netmask=%d.%d.%d.%d' % (
        wifiNetmask.addr1, wifiNetmask.addr2, wifiNetmask.addr3, wifiNetmask.addr4))
    return [c_uint8(wifiNetmask.addr1).value, c_uint8(wifiNetmask.addr2).value, c_uint8(wifiNetmask.addr3).value,
            c_uint8(wifiNetmask.addr4).value]


def SetWIFIGateway(api, dobotId, addr1, addr2, addr3, addr4):
    wifiGateway = WIFIGateway()
    wifiGateway.addr1 = addr1
    wifiGateway.addr2 = addr2
    wifiGateway.addr3 = addr3
    wifiGateway.addr4 = addr4
    while True:
        result = api.SetWIFIGateway(dobotId, byref(wifiGateway))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetWIFIGateway(api, dobotId):
    wifiGateway = WIFIGateway()
    while True:
        result = api.GetWIFIGateway(dobotId, byref(wifiGateway))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetWIFIGateway: wifiGateway=%d.%d.%d.%d' % (
        wifiGateway.addr1, wifiGateway.addr2, wifiGateway.addr3, wifiGateway.addr4))
    return [c_uint8(wifiGateway.addr1).value, c_uint8(wifiGateway.addr2).value, c_uint8(wifiGateway.addr3).value,
            c_uint8(wifiGateway.addr4).value]


def SetWIFIDNS(api, dobotId, addr1, addr2, addr3, addr4):
    wifiDNS = WIFIDNS()
    wifiDNS.addr1 = addr1
    wifiDNS.addr2 = addr2
    wifiDNS.addr3 = addr3
    wifiDNS.addr4 = addr4
    while True:
        result = api.SetWIFIDNS(dobotId, byref(wifiDNS))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetWIFIDNS(api, dobotId):
    wifiDNS = WIFIDNS()
    while True:
        result = api.GetWIFIDNS(dobotId, byref(wifiDNS))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    output('GetWIFIDNS: wifiDNS=%d.%d.%d.%d' % (wifiDNS.addr1, wifiDNS.addr2, wifiDNS.addr3, wifiDNS.addr4))
    return [c_uint8(wifiDNS.addr1).value, c_uint8(wifiDNS.addr2).value, c_uint8(wifiDNS.addr3).value,
            c_uint8(wifiDNS.addr4).value]


def SetColorSensor(api, dobotId, isEnable, colorPort):
    enable = c_bool(isEnable)
    port = c_uint8(colorPort)
    while True:
        result = api.SetColorSensor(dobotId, enable, port)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetColorSensor(api, dobotId):
    r = c_ubyte(0)
    g = c_ubyte(0)
    b = c_ubyte(0)
    while True:
        result = api.GetColorSensor(dobotId, byref(r), byref(g), byref(b))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [r.value, g.value, b.value]


def SetInfraredSensor(api, dobotId, isEnable, infraredPort):
    enable = c_bool(isEnable)
    port = c_uint8(infraredPort)
    while True:
        result = api.SetInfraredSensor(dobotId, enable, port)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def GetInfraredSensor(api, dobotId, infraredPort):
    port = c_uint8(infraredPort)
    value = c_ubyte(0)

    while True:
        result = api.GetInfraredSensor(dobotId, port, byref(value))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [value.value]


def SetLostStepParams(api, dobotId, threshold):
    t = c_float(threshold)
    while True:
        result = api.SetLostStepParams(dobotId)
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break


def SetLostStepCmd(api, dobotId, isQueued=0):
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.SetLostStepCmd(dobotId, isQueued, byref(queuedCmdIndex))
        if result != DobotCommunicate.DobotCommunicate_NoError:
            dSleep(5)
            continue
        break
    return [queuedCmdIndex.value]


##################  Ex扩展函数，该套函数会检测每一条指令运行完毕  ##################
def GetPoseEx(api, dobotId, index):
    if index == 0:
        ret = GetDeviceWithL(api, dobotId)
        if not ret:
            print("Dobot is not in L model")
            return

        lr = GetPoseL(api, dobotId)
        return round(lr[0], 4)

    pos = GetPose(api, dobotId)
    return round(pos[index - 1], 4)


def SetHOMECmdEx(api, dobotId, temp, isQueued=0):
    ret = SetHOMECmd(api, dobotId, temp, isQueued)
    queuedCmdIndex = c_uint64(0)
    while True:
        result = api.GetQueuedCmdCurrentIndex(dobotId, byref(queuedCmdIndex))
        if result == DobotCommunicate.DobotCommunicate_NoError and ret[0] <= queuedCmdIndex.value:
            break
        # 延时太短的话按reset键后不能断开连接
        dSleep(100)


def SetWAITCmdEx(api, dobotId, waitTime, isQueued=0):
    # ret = SetWAITCmd(api, dobotId, waitTime, isQueued)
    # while(True):
    #   if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
    #        break
    dSleep(waitTime * 1000)


def SetEndEffectorParamsEx(api, dobotId, xBias, yBias, zBias, isQueued=0):
    ret = SetEndEffectorParams(api, dobotId, xBias, yBias, zBias, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)


def SetPTPJointParamsEx(api, dobotId, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity,
                        j3Acceleration, j4Velocity, j4Acceleration, isQueued=0):
    ret = SetPTPJointParams(api, dobotId, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity,
                            j3Acceleration, j4Velocity, j4Acceleration, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)


def SetPTPLParamsEx(api, dobotId, lVelocity, lAcceleration, isQueued=0):
    ret = GetDeviceWithL(api, dobotId)
    if not ret:
        print("Dobot is not in L model")
        return

    ret = SetPTPLParams(api, dobotId, lVelocity, lAcceleration, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)


def SetPTPCommonParamsEx(api, dobotId, velocityRatio, accelerationRatio, isQueued=0):
    ret = SetPTPCommonParams(api, dobotId, velocityRatio, accelerationRatio, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)


def SetPTPJumpParamsEx(api, dobotId, jumpHeight, maxJumpHeight, isQueued=0):
    ret = SetPTPJumpParams(api, dobotId, jumpHeight, maxJumpHeight, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)


def SetPTPCmdEx(api, dobotId, ptpMode, x, y, z, rHead, isQueued=0):
    ret = SetPTPCmd(api, dobotId, ptpMode, x, y, z, rHead, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)


def SetIOMultiplexingEx(api, dobotId, address, multiplex, isQueued=0):
    ret = SetIOMultiplexing(api, dobotId, address, multiplex, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)


def SetEndEffectorSuctionCupEx(api, dobotId, enableCtrl, on, isQueued=0):
    ret = SetEndEffectorSuctionCup(api, dobotId, enableCtrl, on, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)


def SetEndEffectorGripperEx(api, dobotId, enableCtrl, on, isQueued=0):
    ret = SetEndEffectorGripper(api, dobotId, enableCtrl, on, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)


def SetEndEffectorLaserEx(api, dobotId, enableCtrl, power, isQueued=0):
    SetIOMultiplexingEx(api, dobotId, 2, 1, isQueued)
    SetIOMultiplexingEx(api, dobotId, 4, 2, isQueued)
    SetIODOEx(api, dobotId, 2, enableCtrl, isQueued)
    SetIOPWMEx(api, dobotId, 4, 10000, power, isQueued)


def SetIODOEx(api, dobotId, address, level, isQueued=0):
    ret = SetIODO(api, dobotId, address, level, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)


def SetEMotorEx(api, dobotId, index, isEnabled, speed, isQueued=0):
    ret = SetEMotor(api, dobotId, index, isEnabled, speed, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)


def SetEMotorSEx(api, dobotId, index, isEnabled, speed, distance, isQueued=0):
    ret = SetEMotorS(api, dobotId, index, isEnabled, speed, distance, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)


def SetIOPWMEx(api, dobotId, address, frequency, dutyCycle, isQueued=0):
    while True:
        ret = SetIOPWM(api, dobotId, address, frequency, dutyCycle, isQueued)
        if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)


def SetPTPWithLCmdEx(api, dobotId, ptpMode, x, y, z, rHead, l, isQueued=0):
    ret = GetDeviceWithL(api, dobotId)
    if not ret:
        print("Dobot is not in L model")
        return

    ret = SetPTPWithLCmd(api, dobotId, ptpMode, x, y, z, rHead, l, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)


def GetColorSensorEx(api, dobotId, index):
    result = GetColorSensor(api, dobotId)
    return result[index]


def SetAutoLevelingCmdEx(api, dobotId, controlFlag, precision, isQueued=1):
    index = SetAutoLevelingCmd(api, dobotId, controlFlag, precision, isQueued)[0]
    while True:
        if index <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)


def SetLostStepCmdEx(api, dobotId, isQueued=1):
    ret = SetLostStepCmd(api, dobotId, isQueued)
    while True:
        if ret[0] <= GetQueuedCmdCurrentIndex(api, dobotId)[0]:
            break
        dSleep(5)
