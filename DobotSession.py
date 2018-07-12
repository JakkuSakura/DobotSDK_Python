from DobotAPI import *


class DobotSession:
    def __init__(self, dobotId, split='_'):
        self.dobotId = dobotId
        self.api = load(dobotId, split)

    def SearchDobot(self, maxLen=1000):
        return SearchDobot(self.api, maxLen)

    def ConnectDobot(self, portName="", baudrate=115200):
        return ConnectDobot(self.api, portName, baudrate)

    def DisconnectDobot(self):
        return DisconnectDobot(self.api)

    def SetCmdTimeout(self, times):
        return SetCmdTimeout(self.api, times)

    def DobotExec(self):
        return DobotExec(self.api)

    def SetAutoLevelingCmd(self, controlFlag, precision, isQueued=0):
        return SetAutoLevelingCmd(self.api, controlFlag, precision, isQueued)

    def GetAutoLevelingResult(self):
        return GetAutoLevelingResult(self.api)

    def GetQueuedCmdCurrentIndex(self):
        return GetQueuedCmdCurrentIndex(self.api)

    def SetQueuedCmdStartExec(self):
        return SetQueuedCmdStartExec(self.api)

    def SetQueuedCmdStopExec(self):
        return SetQueuedCmdStopExec(self.api)

    def SetQueuedCmdForceStopExec(self):
        return SetQueuedCmdForceStopExec(self.api)

    def SetQueuedCmdStartDownload(self, totalLoop, linePerLoop):
        return SetQueuedCmdStartDownload(self.api, totalLoop, linePerLoop)

    def SetQueuedCmdStopDownload(self):
        return SetQueuedCmdStopDownload(self.api)

    def SetQueuedCmdClear(self):
        return SetQueuedCmdClear(self.api)

    def SetDeviceSN(self, str_):
        return SetDeviceSN(self.api, str_)

    def GetDeviceSN(self):
        return GetDeviceSN(self.api)

    def SetDeviceName(self, str_):
        return SetDeviceName(self.api, str_)

    def GetDeviceName(self):
        return GetDeviceName(self.api)

    def GetDeviceVersion(self):
        return GetDeviceVersion(self.api)

    def SetDeviceWithL(self, isWithL):
        return SetDeviceWithL(self.api, isWithL)

    def GetDeviceWithL(self):
        return GetDeviceWithL(self.api)

    def ResetPose(self, manual, rearArmAngle, frontArmAngle):
        return ResetPose(self.api, manual, rearArmAngle, frontArmAngle)

    def GetPose(self) -> List[float]:
        return GetPose(self.api)

    def GetPoseL(self):
        return GetPoseL(self.api)

    def GetKinematics(self):
        return GetKinematics(self.api)

    def GetAlarmsState(self, maxLen=1000):
        return GetAlarmsState(self.api, maxLen)

    def ClearAllAlarmsState(self):
        return ClearAllAlarmsState(self.api)

    def GetUserParams(self):
        return GetUserParams(self.api)

    def SetHOMEParams(self, x, y, z, r, isQueued=0):
        return SetHOMEParams(self.api, x, y, z, r, isQueued)

    def GetHOMEParams(self):
        return GetHOMEParams(self.api)

    def SetHOMECmd(self, temp, isQueued=0):
        return SetHOMECmd(self.api, temp, isQueued)

    def SetArmOrientation(self, armOrientation, isQueued=0):
        return SetArmOrientation(self.api, armOrientation, isQueued)

    def GetArmOrientation(self):
        return GetArmOrientation(self.api)

    def SetHHTTrigMode(self, hhtTrigMode):
        return SetHHTTrigMode(self.api, hhtTrigMode)

    def GetHHTTrigMode(self):
        return GetHHTTrigMode(self.api)

    def SetHHTTrigOutputEnabled(self, isEnabled):
        return SetHHTTrigOutputEnabled(self.api, isEnabled)

    def GetHHTTrigOutputEnabled(self):
        return GetHHTTrigOutputEnabled(self.api)

    def GetHHTTrigOutput(self):
        return GetHHTTrigOutput(self.api)

    def SetEndEffectorParams(self, xBias, yBias, zBias, isQueued=0):
        return SetEndEffectorParams(self.api, xBias, yBias, zBias, isQueued)

    def GetEndEffectorParams(self):
        return GetEndEffectorParams(self.api)

    def SetEndEffectorLaser(self, enableCtrl, on, isQueued=0):
        return SetEndEffectorLaser(self.api, enableCtrl, on, isQueued)

    def GetEndEffectorLaser(self):
        return GetEndEffectorLaser(self.api)

    def SetEndEffectorSuctionCup(self, enableCtrl, on, isQueued=0):
        return SetEndEffectorSuctionCup(self.api, enableCtrl, on, isQueued)

    def GetEndEffectorSuctionCup(self):
        return GetEndEffectorSuctionCup(self.api)

    def SetEndEffectorGripper(self, enableCtrl, on, isQueued=0):
        return SetEndEffectorGripper(self.api, enableCtrl, on, isQueued)

    def GetEndEffectorGripper(self):
        return GetEndEffectorGripper(self.api)

    def SetJOGJointParams(self, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity, j3Acceleration,
                          j4Velocity, j4Acceleration, isQueued=0):
        return SetJOGJointParams(self.api, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity,
                                 j3Acceleration, j4Velocity, j4Acceleration, isQueued)

    def GetJOGJointParams(self):
        return GetJOGJointParams(self.api)

    def SetJOGCoordinateParams(self, xVelocity, xAcceleration, yVelocity, yAcceleration, zVelocity, zAcceleration,
                               rVelocity, rAcceleration, isQueued=0):
        return SetJOGCoordinateParams(self.api, xVelocity, xAcceleration, yVelocity, yAcceleration, zVelocity,
                                      zAcceleration, rVelocity, rAcceleration, isQueued)

    def GetJOGCoordinateParams(self):
        return GetJOGCoordinateParams(self.api)

    def SetJOGLParams(self, velocity, acceleration, isQueued=0):
        return SetJOGLParams(self.api, velocity, acceleration, isQueued)

    def GetJOGLParams(self):
        return GetJOGLParams(self.api)

    def SetJOGCommonParams(self, value_velocityratio, value_accelerationratio, isQueued=0):
        return SetJOGCommonParams(self.api, value_velocityratio, value_accelerationratio, isQueued)

    def GetJOGCommonParams(self):
        return GetJOGCommonParams(self.api)

    def SetJOGCmd(self, isJoint, cmd, isQueued=0):
        return SetJOGCmd(self.api, isJoint, cmd, isQueued)

    def SetPTPJointParams(self, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity, j3Acceleration,
                          j4Velocity, j4Acceleration, isQueued=0):
        return SetPTPJointParams(self.api, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity,
                                 j3Acceleration, j4Velocity, j4Acceleration, isQueued)

    def GetPTPJointParams(self):
        return GetPTPJointParams(self.api)

    def SetPTPCoordinateParams(self, xyzVelocity, xyzAcceleration, rVelocity, rAcceleration, isQueued=0):
        return SetPTPCoordinateParams(self.api, xyzVelocity, xyzAcceleration, rVelocity, rAcceleration, isQueued)

    def GetPTPCoordinateParams(self):
        return GetPTPCoordinateParams(self.api)

    def SetPTPLParams(self, velocity, acceleration, isQueued=0):
        return SetPTPLParams(self.api, velocity, acceleration, isQueued)

    def GetPTPLParams(self):
        return GetPTPLParams(self.api)

    def SetPTPJumpParams(self, jumpHeight, zLimit, isQueued=0):
        return SetPTPJumpParams(self.api, jumpHeight, zLimit, isQueued)

    def GetPTPJumpParams(self):
        return GetPTPJumpParams(self.api)

    def SetPTPCommonParams(self, velocityRatio, accelerationRatio, isQueued=0):
        return SetPTPCommonParams(self.api, velocityRatio, accelerationRatio, isQueued)

    def GetPTPCommonParams(self):
        return GetPTPCommonParams(self.api)

    def SetPTPCmd(self, ptpMode, x, y, z, rHead, isQueued=0):
        return SetPTPCmd(self.api, ptpMode, x, y, z, rHead, isQueued)

    def SetPTPWithLCmd(self, ptpMode, x, y, z, rHead, l, isQueued=0):
        return SetPTPWithLCmd(self.api, ptpMode, x, y, z, rHead, l, isQueued)

    def SetCPParams(self, planAcc, juncitionVel, acc, realTimeTrack=0, isQueued=0):
        return SetCPParams(self.api, planAcc, juncitionVel, acc, realTimeTrack, isQueued)

    def GetCPParams(self):
        return GetCPParams(self.api)

    def SetCPCmd(self, cpMode, x, y, z, velocity, isQueued=0):
        return SetCPCmd(self.api, cpMode, x, y, z, velocity, isQueued)

    def SetCPLECmd(self, cpMode, x, y, z, power, isQueued=0):
        return SetCPLECmd(self.api, cpMode, x, y, z, power, isQueued)

    def SetARCParams(self, xyzVelocity, rVelocity, xyzAcceleration, rAcceleration, isQueued=0):
        return SetARCParams(self.api, xyzVelocity, rVelocity, xyzAcceleration, rAcceleration, isQueued)

    def GetARCParams(self):
        return GetARCParams(self.api)

    def SetARCCmd(self, cirPoint, toPoint, isQueued=0):
        return SetARCCmd(self.api, cirPoint, toPoint, isQueued)

    def SetWAITCmd(self, waitTimeMs, isQueued=1):
        return SetWAITCmd(self.api, waitTimeMs, isQueued)

    def SetTRIGCmd(self, address, mode, condition, threshold, isQueued=0):
        return SetTRIGCmd(self.api, address, mode, condition, threshold, isQueued)

    def SetIOMultiplexing(self, address, multiplex, isQueued=0):
        return SetIOMultiplexing(self.api, address, multiplex, isQueued)

    def GetIOMultiplexing(self, addr):
        return GetIOMultiplexing(self.api, addr)

    def SetIODO(self, address, level, isQueued=0):
        return SetIODO(self.api, address, level, isQueued)

    def GetIODO(self, addr):
        return GetIODO(self.api, addr)

    def SetIOPWM(self, address, frequency, dutyCycle, isQueued=0):
        return SetIOPWM(self.api, address, frequency, dutyCycle, isQueued)

    def GetIOPWM(self, addr):
        return GetIOPWM(self.api, addr)

    def GetIODI(self, addr):
        return GetIODI(self.api, addr)

    def SetEMotor(self, index, isEnabled, speed, isQueued=0):
        return SetEMotor(self.api, index, isEnabled, speed, isQueued)

    def SetEMotorS(self, index, isEnabled, speed, distance, isQueued=0):
        return SetEMotorS(self.api, index, isEnabled, speed, distance, isQueued)

    def GetIOADC(self, addr):
        return GetIOADC(self.api, addr)

    def SetAngleSensorStaticError(self, rearArmAngleError, frontArmAngleError):
        return SetAngleSensorStaticError(self.api, rearArmAngleError, frontArmAngleError)

    def GetAngleSensorStaticError(self):
        return GetAngleSensorStaticError(self.api)

    def SetAngleSensorCoef(self, rearArmAngleCoef, frontArmAngleCoef):
        return SetAngleSensorCoef(self.api, rearArmAngleCoef, frontArmAngleCoef)

    def GetAngleSensorCoef(self):
        return GetAngleSensorCoef(self.api)

    def SetBaseDecoderStaticError(self, baseDecoderError):
        return SetBaseDecoderStaticError(self.api, baseDecoderError)

    def GetBaseDecoderStaticError(self):
        return GetBaseDecoderStaticError(self.api)

    def GetWIFIConnectStatus(self):
        return GetWIFIConnectStatus(self.api)

    def SetWIFIConfigMode(self, enable):
        return SetWIFIConfigMode(self.api, enable)

    def GetWIFIConfigMode(self):
        return GetWIFIConfigMode(self.api)

    def SetWIFISSID(self, ssid):
        return SetWIFISSID(self.api, ssid)

    def GetWIFISSID(self):
        return GetWIFISSID(self.api)

    def SetWIFIPassword(self, password):
        return SetWIFIPassword(self.api, password)

    def GetWIFIPassword(self):
        return GetWIFIPassword(self.api)

    def SetWIFIIPAddress(self, dhcp, addr1, addr2, addr3, addr4):
        return SetWIFIIPAddress(self.api, dhcp, addr1, addr2, addr3, addr4)

    def GetWIFIIPAddress(self):
        return GetWIFIIPAddress(self.api)

    def SetWIFINetmask(self, addr1, addr2, addr3, addr4):
        return SetWIFINetmask(self.api, addr1, addr2, addr3, addr4)

    def GetWIFINetmask(self):
        return GetWIFINetmask(self.api)

    def SetWIFIGateway(self, addr1, addr2, addr3, addr4):
        return SetWIFIGateway(self.api, addr1, addr2, addr3, addr4)

    def GetWIFIGateway(self):
        return GetWIFIGateway(self.api)

    def SetWIFIDNS(self, addr1, addr2, addr3, addr4):
        return SetWIFIDNS(self.api, addr1, addr2, addr3, addr4)

    def GetWIFIDNS(self):
        return GetWIFIDNS(self.api)

    def SetColorSensor(self, isEnable, colorPort):
        return SetColorSensor(self.api, isEnable, colorPort)

    def GetColorSensor(self):
        return GetColorSensor(self.api)

    def SetInfraredSensor(self, isEnable, infraredPort):
        return SetInfraredSensor(self.api, isEnable, infraredPort)

    def GetInfraredSensor(self, infraredPort):
        return GetInfraredSensor(self.api, infraredPort)

    def GetPoseEx(self, index):
        return GetPoseEx(self.api, index)

    def SetHOMECmdEx(self, temp, isQueued=0):
        return SetHOMECmdEx(self.api, temp, isQueued)

    def SetWAITCmdEx(self, waitTimeMs, isQueued=0):
        return SetWAITCmdEx(self.api, waitTimeMs, isQueued)

    def SetEndEffectorParamsEx(self, xBias, yBias, zBias, isQueued=0):
        return SetEndEffectorParamsEx(self.api, xBias, yBias, zBias, isQueued)

    def SetPTPJointParamsEx(self, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity, j3Acceleration,
                            j4Velocity, j4Acceleration, isQueued=0):
        return SetPTPJointParamsEx(self.api, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity,
                                   j3Acceleration, j4Velocity, j4Acceleration, isQueued)

    def SetPTPLParamsEx(self, lVelocity, lAcceleration, isQueued=0):
        return SetPTPLParamsEx(self.api, lVelocity, lAcceleration, isQueued)

    def SetPTPCommonParamsEx(self, velocityRatio, accelerationRatio, isQueued=0):
        return SetPTPCommonParamsEx(self.api, velocityRatio, accelerationRatio, isQueued)

    def SetPTPJumpParamsEx(self, jumpHeight, maxJumpHeight, isQueued=0):
        return SetPTPJumpParamsEx(self.api, jumpHeight, maxJumpHeight, isQueued)

    def SetPTPCmdEx(self, ptpMode, x, y, z, rHead, isQueued=0):
        return SetPTPCmdEx(self.api, ptpMode, x, y, z, rHead, isQueued)

    def SetIOMultiplexingEx(self, address, multiplex, isQueued=0):
        return SetIOMultiplexingEx(self.api, address, multiplex, isQueued)

    def SetEndEffectorSuctionCupEx(self, enableCtrl, on, isQueued=0):
        return SetEndEffectorSuctionCupEx(self.api, enableCtrl, on, isQueued)

    def SetEndEffectorGripperEx(self, enableCtrl, on, isQueued=0):
        return SetEndEffectorGripperEx(self.api, enableCtrl, on, isQueued)

    def SetIODOEx(self, address, level, isQueued=0):
        return SetIODOEx(self.api, address, level, isQueued)

    def SetEMotorEx(self, index, isEnabled, speed, isQueued=0):
        return SetEMotorEx(self.api, index, isEnabled, speed, isQueued)

    def SetEMotorSEx(self, index, isEnabled, speed, distance, isQueued=0):
        return SetEMotorSEx(self.api, index, isEnabled, speed, distance, isQueued)

    def SetIOPWMEx(self, address, frequency, dutyCycle, isQueued=0):
        return SetIOPWMEx(self.api, address, frequency, dutyCycle, isQueued)

    def SetPTPWithLCmdEx(self, ptpMode, x, y, z, rHead, l, isQueued=0):
        return SetPTPWithLCmdEx(self.api, ptpMode, x, y, z, rHead, l, isQueued)

    def GetColorSensorEx(self, index):
        return GetColorSensorEx(self.api, index)
