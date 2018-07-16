from typing import List

import DobotAPI
import DobotTypes


class DobotSession:
    dobot_index = 0

    def __init__(self, dobotId=None, split='_'):
        if dobotId is None:
            self.dobotId = DobotSession.dobot_index
            DobotSession.dobot_index += 1
        else:
            self.dobotId = dobotId
        self.api = DobotAPI.load(self.dobotId, split)

    def SearchDobot(self, maxLen=1000):
        return DobotAPI.SearchDobot(self.api, maxLen)

    def ConnectDobot(self, portName="", baudrate=115200):
        return DobotAPI.ConnectDobot(self.api, portName, baudrate)

    def DisconnectDobot(self):
        return DobotAPI.DisconnectDobot(self.api)

    def DobotExec(self):
        return DobotAPI.DobotExec(self.api)

    def SetAutoLevelingCmd(self, controlFlag, precision, isQueued=0):
        return DobotAPI.SetAutoLevelingCmd(self.api, controlFlag, precision, isQueued)

    def GetAutoLevelingResult(self):
        return DobotAPI.GetAutoLevelingResult(self.api)

    def GetQueuedCmdCurrentIndex(self):
        return DobotAPI.GetQueuedCmdCurrentIndex(self.api)

    def SetQueuedCmdStartExec(self):
        return DobotAPI.SetQueuedCmdStartExec(self.api)

    def SetQueuedCmdStopExec(self):
        return DobotAPI.SetQueuedCmdStopExec(self.api)

    def SetQueuedCmdForceStopExec(self):
        return DobotAPI.SetQueuedCmdForceStopExec(self.api)

    def SetQueuedCmdStartDownload(self, totalLoop, linePerLoop):
        return DobotAPI.SetQueuedCmdStartDownload(self.api, totalLoop, linePerLoop)

    def SetQueuedCmdStopDownload(self):
        return DobotAPI.SetQueuedCmdStopDownload(self.api)

    def SetQueuedCmdClear(self):
        return DobotAPI.SetQueuedCmdClear(self.api)

    def SetDeviceSN(self, str_):
        return DobotAPI.SetDeviceSN(self.api, str_)

    def GetDeviceSN(self):
        return DobotAPI.GetDeviceSN(self.api)

    def SetDeviceName(self, str_):
        return DobotAPI.SetDeviceName(self.api, str_)

    def GetDeviceName(self) -> str:
        return DobotAPI.GetDeviceName(self.api)

    def GetDeviceVersion(self) -> List[int]:
        return DobotAPI.GetDeviceVersion(self.api)

    def SetDeviceWithL(self, isWithL):
        return DobotAPI.SetDeviceWithL(self.api, isWithL)

    def GetDeviceWithL(self):
        return DobotAPI.GetDeviceWithL(self.api)

    def ResetPose(self, manual, rearArmAngle, frontArmAngle):
        return DobotAPI.ResetPose(self.api, manual, rearArmAngle, frontArmAngle)

    def GetPose(self) -> List[float]:
        return DobotAPI.GetPose(self.api)

    def GetPoseL(self):
        return DobotAPI.GetPoseL(self.api)

    def GetKinematics(self):
        return DobotAPI.GetKinematics(self.api)

    def GetAlarmsState(self, maxLen=1000):
        return DobotAPI.GetAlarmsState(self.api, maxLen)

    def ClearAllAlarmsState(self):
        return DobotAPI.ClearAllAlarmsState(self.api)

    def GetUserParams(self):
        return DobotAPI.GetUserParams(self.api)

    def SetHOMEParams(self, x, y, z, r, isQueued=0):
        return DobotAPI.SetHOMEParams(self.api, x, y, z, r, isQueued)

    def GetHOMEParams(self):
        return DobotAPI.GetHOMEParams(self.api)

    def SetHOMECmd(self, temp, isQueued=0):
        return DobotAPI.SetHOMECmd(self.api, temp, isQueued)

    def SetArmOrientation(self, armOrientation, isQueued=0):
        return DobotAPI.SetArmOrientation(self.api, armOrientation, isQueued)

    def GetArmOrientation(self):
        return DobotAPI.GetArmOrientation(self.api)

    def SetHHTTrigMode(self, hhtTrigMode):
        return DobotAPI.SetHHTTrigMode(self.api, hhtTrigMode)

    def GetHHTTrigMode(self):
        return DobotAPI.GetHHTTrigMode(self.api)

    def SetHHTTrigOutputEnabled(self, isEnabled):
        return DobotAPI.SetHHTTrigOutputEnabled(self.api, isEnabled)

    def GetHHTTrigOutputEnabled(self):
        return DobotAPI.GetHHTTrigOutputEnabled(self.api)

    def GetHHTTrigOutput(self):
        return DobotAPI.GetHHTTrigOutput(self.api)

    def SetEndEffectorParams(self, xBias, yBias, zBias, isQueued=0):
        return DobotAPI.SetEndEffectorParams(self.api, xBias, yBias, zBias, isQueued)

    def GetEndEffectorParams(self):
        return DobotAPI.GetEndEffectorParams(self.api)

    def SetEndEffectorLaser(self, enableCtrl, on, isQueued=0):
        return DobotAPI.SetEndEffectorLaser(self.api, enableCtrl, on, isQueued)

    def GetEndEffectorLaser(self):
        return DobotAPI.GetEndEffectorLaser(self.api)

    def SetEndEffectorSuctionCup(self, enableCtrl, on, isQueued=0):
        return DobotAPI.SetEndEffectorSuctionCup(self.api, enableCtrl, on, isQueued)

    def GetEndEffectorSuctionCup(self):
        return DobotAPI.GetEndEffectorSuctionCup(self.api)

    def SetEndEffectorGripper(self, enableCtrl, on, isQueued=0):
        return DobotAPI.SetEndEffectorGripper(self.api, enableCtrl, on, isQueued)

    def GetEndEffectorGripper(self):
        return DobotAPI.GetEndEffectorGripper(self.api)

    def SetJOGJointParams(self, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity, j3Acceleration,
                          j4Velocity, j4Acceleration, isQueued=0):
        return DobotAPI.SetJOGJointParams(self.api, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity,
                                          j3Acceleration, j4Velocity, j4Acceleration, isQueued)

    def GetJOGJointParams(self):
        return DobotAPI.GetJOGJointParams(self.api)

    def SetJOGCoordinateParams(self, xVelocity, xAcceleration, yVelocity, yAcceleration, zVelocity, zAcceleration,
                               rVelocity, rAcceleration, isQueued=0):
        return DobotAPI.SetJOGCoordinateParams(self.api, xVelocity, xAcceleration, yVelocity, yAcceleration, zVelocity,
                                               zAcceleration, rVelocity, rAcceleration, isQueued)

    def GetJOGCoordinateParams(self):
        return DobotAPI.GetJOGCoordinateParams(self.api)

    def SetJOGLParams(self, velocity, acceleration, isQueued=0):
        return DobotAPI.SetJOGLParams(self.api, velocity, acceleration, isQueued)

    def GetJOGLParams(self):
        return DobotAPI.GetJOGLParams(self.api)

    def SetJOGCommonParams(self, value_velocityratio, value_accelerationratio, isQueued=0):
        return DobotAPI.SetJOGCommonParams(self.api, value_velocityratio, value_accelerationratio, isQueued)

    def GetJOGCommonParams(self):
        return DobotAPI.GetJOGCommonParams(self.api)

    def SetJOGCmd(self, isJoint, cmd, isQueued=0):
        return DobotAPI.SetJOGCmd(self.api, isJoint, cmd, isQueued)

    def SetPTPJointParams(self, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity, j3Acceleration,
                          j4Velocity, j4Acceleration, isQueued=0):
        return DobotAPI.SetPTPJointParams(self.api, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity,
                                          j3Acceleration, j4Velocity, j4Acceleration, isQueued)

    def GetPTPJointParams(self):
        return DobotAPI.GetPTPJointParams(self.api)

    def SetPTPCoordinateParams(self, xyzVelocity, xyzAcceleration, rVelocity, rAcceleration, isQueued=0):
        return DobotAPI.SetPTPCoordinateParams(self.api, xyzVelocity, xyzAcceleration, rVelocity, rAcceleration,
                                               isQueued)

    def GetPTPCoordinateParams(self):
        return DobotAPI.GetPTPCoordinateParams(self.api)

    def SetPTPLParams(self, velocity, acceleration, isQueued=0):
        return DobotAPI.SetPTPLParams(self.api, velocity, acceleration, isQueued)

    def GetPTPLParams(self):
        return DobotAPI.GetPTPLParams(self.api)

    def SetPTPJumpParams(self, jumpHeight, zLimit, isQueued=0):
        return DobotAPI.SetPTPJumpParams(self.api, jumpHeight, zLimit, isQueued)

    def GetPTPJumpParams(self):
        return DobotAPI.GetPTPJumpParams(self.api)

    def SetPTPCommonParams(self, velocityRatio, accelerationRatio, isQueued=0):
        return DobotAPI.SetPTPCommonParams(self.api, velocityRatio, accelerationRatio, isQueued)

    def GetPTPCommonParams(self):
        return DobotAPI.GetPTPCommonParams(self.api)

    def SetPTPCmd(self, ptpMode, x, y, z, rHead, isQueued=0):
        return DobotAPI.SetPTPCmd(self.api, ptpMode, x, y, z, rHead, isQueued)

    def SetPTPWithLCmd(self, ptpMode, x, y, z, rHead, l, isQueued=0):
        return DobotAPI.SetPTPWithLCmd(self.api, ptpMode, x, y, z, rHead, l, isQueued)

    def SetCPParams(self, planAcc, juncitionVel, acc, realTimeTrack=0, isQueued=0):
        return DobotAPI.SetCPParams(self.api, planAcc, juncitionVel, acc, realTimeTrack, isQueued)

    def GetCPParams(self):
        return DobotAPI.GetCPParams(self.api)

    def SetCPCmd(self, cpMode, x, y, z, velocity, isQueued=0):
        return DobotAPI.SetCPCmd(self.api, cpMode, x, y, z, velocity, isQueued)

    def SetCPLECmd(self, cpMode, x, y, z, power, isQueued=0):
        return DobotAPI.SetCPLECmd(self.api, cpMode, x, y, z, power, isQueued)

    def SetARCParams(self, xyzVelocity, rVelocity, xyzAcceleration, rAcceleration, isQueued=0):
        return DobotAPI.SetARCParams(self.api, xyzVelocity, rVelocity, xyzAcceleration, rAcceleration, isQueued)

    def GetARCParams(self):
        return DobotAPI.GetARCParams(self.api)

    def SetARCCmd(self, cirPoint, toPoint, isQueued=0):
        return DobotAPI.SetARCCmd(self.api, cirPoint, toPoint, isQueued)

    def SetWAITCmd(self, waitTimeMs, isQueued=1):
        return DobotAPI.SetWAITCmd(self.api, waitTimeMs, isQueued)

    def SetTRIGCmd(self, address, mode, condition, threshold, isQueued=0):
        return DobotAPI.SetTRIGCmd(self.api, address, mode, condition, threshold, isQueued)

    def SetIOMultiplexing(self, address, multiplex, isQueued=0):
        return DobotAPI.SetIOMultiplexing(self.api, address, multiplex, isQueued)

    def GetIOMultiplexing(self, addr):
        return DobotAPI.GetIOMultiplexing(self.api, addr)

    def SetIODO(self, address, level, isQueued=0):
        return DobotAPI.SetIODO(self.api, address, level, isQueued)

    def GetIODO(self, addr):
        return DobotAPI.GetIODO(self.api, addr)

    def SetIOPWM(self, address, frequency, dutyCycle, isQueued=0):
        return DobotAPI.SetIOPWM(self.api, address, frequency, dutyCycle, isQueued)

    def GetIOPWM(self, addr):
        return DobotAPI.GetIOPWM(self.api, addr)

    def GetIODI(self, addr):
        return DobotAPI.GetIODI(self.api, addr)

    def SetEMotor(self, index, isEnabled, speed, isQueued=0):
        return DobotAPI.SetEMotor(self.api, index, isEnabled, speed, isQueued)

    def SetEMotorS(self, index, isEnabled, speed, distance, isQueued=0):
        return DobotAPI.SetEMotorS(self.api, index, isEnabled, speed, distance, isQueued)

    def GetIOADC(self, addr):
        return DobotAPI.GetIOADC(self.api, addr)

    def SetAngleSensorStaticError(self, rearArmAngleError, frontArmAngleError):
        return DobotAPI.SetAngleSensorStaticError(self.api, rearArmAngleError, frontArmAngleError)

    def GetAngleSensorStaticError(self):
        return DobotAPI.GetAngleSensorStaticError(self.api)

    def SetAngleSensorCoef(self, rearArmAngleCoef, frontArmAngleCoef):
        return DobotAPI.SetAngleSensorCoef(self.api, rearArmAngleCoef, frontArmAngleCoef)

    def GetAngleSensorCoef(self):
        return DobotAPI.GetAngleSensorCoef(self.api)

    def SetBaseDecoderStaticError(self, baseDecoderError):
        return DobotAPI.SetBaseDecoderStaticError(self.api, baseDecoderError)

    def GetBaseDecoderStaticError(self):
        return DobotAPI.GetBaseDecoderStaticError(self.api)

    def GetWIFIConnectStatus(self):
        return DobotAPI.GetWIFIConnectStatus(self.api)

    def SetWIFIConfigMode(self, enable):
        return DobotAPI.SetWIFIConfigMode(self.api, enable)

    def GetWIFIConfigMode(self):
        return DobotAPI.GetWIFIConfigMode(self.api)

    def SetWIFISSID(self, ssid):
        return DobotAPI.SetWIFISSID(self.api, ssid)

    def GetWIFISSID(self):
        return DobotAPI.GetWIFISSID(self.api)

    def SetWIFIPassword(self, password):
        return DobotAPI.SetWIFIPassword(self.api, password)

    def GetWIFIPassword(self):
        return DobotAPI.GetWIFIPassword(self.api)

    def SetWIFIIPAddress(self, dhcp, addr1, addr2, addr3, addr4):
        return DobotAPI.SetWIFIIPAddress(self.api, dhcp, addr1, addr2, addr3, addr4)

    def GetWIFIIPAddress(self):
        return DobotAPI.GetWIFIIPAddress(self.api)

    def SetWIFINetmask(self, addr1, addr2, addr3, addr4):
        return DobotAPI.SetWIFINetmask(self.api, addr1, addr2, addr3, addr4)

    def GetWIFINetmask(self):
        return DobotAPI.GetWIFINetmask(self.api)

    def SetWIFIGateway(self, addr1, addr2, addr3, addr4):
        return DobotAPI.SetWIFIGateway(self.api, addr1, addr2, addr3, addr4)

    def GetWIFIGateway(self):
        return DobotAPI.GetWIFIGateway(self.api)

    def SetWIFIDNS(self, addr1, addr2, addr3, addr4):
        return DobotAPI.SetWIFIDNS(self.api, addr1, addr2, addr3, addr4)

    def GetWIFIDNS(self):
        return DobotAPI.GetWIFIDNS(self.api)

    def SetColorSensor(self, isEnable, colorPort):
        return DobotAPI.SetColorSensor(self.api, isEnable, colorPort)

    def GetColorSensor(self):
        return DobotAPI.GetColorSensor(self.api)

    def SetInfraredSensor(self, isEnable, infraredPort):
        return DobotAPI.SetInfraredSensor(self.api, isEnable, infraredPort)

    def GetInfraredSensor(self, infraredPort):
        return DobotAPI.GetInfraredSensor(self.api, infraredPort)

    # FIRMWARE
    def UpdateFirmware(self, firmwareParams: DobotTypes.FirmwareParams):
        DobotAPI.UpdateFirmware(self.api, firmwareParams)

    def SetFirmwareMode(self, firmwareMode):
        DobotAPI.SetFirmwareMode(self.api, firmwareMode)

    def GetFirmwareMode(self):
        DobotAPI.GetFirmwareMode(self.api)

    # LOSTSTEP
    def SetLostStepParams(self, threshold, isQueued=0):
        DobotAPI.SetLostStepParams(self.api, threshold, isQueued)

    def SetLostStepCmd(self, isQueued=1):
        DobotAPI.SetLostStepCmd(self.api, isQueued)

    # UART4 Peripherals
    def GetUART4PeripheralsType(self, p_type):
        DobotAPI.GetUART4PeripheralsType(self.api, p_type)

    def SetUART4PeripheralsEnable(self, isEnable):
        DobotAPI.SetUART4PeripheralsEnable(self.api, isEnable)

    # Function Pluse Mode
    def SendPluse(self, pluseCmd: DobotTypes.PluseCmd, isQueued=0):
        DobotAPI.SendPluse(self.api, pluseCmd, isQueued)

    def SendPluseEx(self, pluseCmd):
        DobotAPI.SendPluseEx(self.api, pluseCmd)

    def GetServoPIDParams(self):
        DobotAPI.GetServoPIDParams(self.api)

    def SetServoPIDParams(self, pid: DobotTypes.PID, isQueued=0):
        DobotAPI.SetServoPIDParams(self.api, pid, isQueued)

    def GetServoControlLoop(self):
        return DobotAPI.GetServoControlLoop(self.api)

    def SetServoControlLoop(self, p_index, controlLoop, isQueued=0):
        DobotAPI.SetServoControlLoop(self.api, p_index, controlLoop, isQueued)

    def SaveServoPIDParams(self, p_index, controlLoop, isQueued=0):
        DobotAPI.SaveServoPIDParams(self.api, p_index, controlLoop, isQueued)

    def GetPoseEx(self, index):
        return DobotAPI.GetPoseEx(self.api, index)

    def SetHOMECmdEx(self, temp, isQueued=0):
        return DobotAPI.SetHOMECmdEx(self.api, temp, isQueued)

    def SetWAITCmdEx(self, waitTimeMs, isQueued=0):
        return DobotAPI.SetWAITCmdEx(self.api, waitTimeMs, isQueued)

    def SetEndEffectorParamsEx(self, xBias, yBias, zBias, isQueued=0):
        return DobotAPI.SetEndEffectorParamsEx(self.api, xBias, yBias, zBias, isQueued)

    def SetPTPJointParamsEx(self, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration, j3Velocity, j3Acceleration,
                            j4Velocity, j4Acceleration, isQueued=0):
        return DobotAPI.SetPTPJointParamsEx(self.api, j1Velocity, j1Acceleration, j2Velocity, j2Acceleration,
                                            j3Velocity, j3Acceleration, j4Velocity, j4Acceleration, isQueued)

    def SetPTPLParamsEx(self, lVelocity, lAcceleration, isQueued=0):
        return DobotAPI.SetPTPLParamsEx(self.api, lVelocity, lAcceleration, isQueued)

    def SetPTPCommonParamsEx(self, velocityRatio, accelerationRatio, isQueued=0):
        return DobotAPI.SetPTPCommonParamsEx(self.api, velocityRatio, accelerationRatio, isQueued)

    def SetPTPJumpParamsEx(self, jumpHeight, maxJumpHeight, isQueued=0):
        return DobotAPI.SetPTPJumpParamsEx(self.api, jumpHeight, maxJumpHeight, isQueued)

    def SetPTPCmdEx(self, ptpMode, x, y, z, rHead, isQueued=0):
        return DobotAPI.SetPTPCmdEx(self.api, ptpMode, x, y, z, rHead, isQueued)

    def SetIOMultiplexingEx(self, address, multiplex, isQueued=0):
        return DobotAPI.SetIOMultiplexingEx(self.api, address, multiplex, isQueued)

    def SetEndEffectorSuctionCupEx(self, enableCtrl, on, isQueued=0):
        return DobotAPI.SetEndEffectorSuctionCupEx(self.api, enableCtrl, on, isQueued)

    def SetEndEffectorGripperEx(self, enableCtrl, on, isQueued=0):
        return DobotAPI.SetEndEffectorGripperEx(self.api, enableCtrl, on, isQueued)

    def SetIODOEx(self, address, level, isQueued=0):
        return DobotAPI.SetIODOEx(self.api, address, level, isQueued)

    def SetEMotorEx(self, index, isEnabled, speed, isQueued=0):
        return DobotAPI.SetEMotorEx(self.api, index, isEnabled, speed, isQueued)

    def SetEMotorSEx(self, index, isEnabled, speed, distance, isQueued=0):
        return DobotAPI.SetEMotorSEx(self.api, index, isEnabled, speed, distance, isQueued)

    def SetIOPWMEx(self, address, frequency, dutyCycle, isQueued=0):
        return DobotAPI.SetIOPWMEx(self.api, address, frequency, dutyCycle, isQueued)

    def SetPTPWithLCmdEx(self, ptpMode, x, y, z, rHead, l, isQueued=0):
        return DobotAPI.SetPTPWithLCmdEx(self.api, ptpMode, x, y, z, rHead, l, isQueued)

    def GetColorSensorEx(self, index):
        return DobotAPI.GetColorSensorEx(self.api, index)
