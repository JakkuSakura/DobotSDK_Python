from ctypes import *


class EndType:
    EndTypeCustom = 0
    EndTypeSuctionCup = 1
    EndTypeGripper = 2
    EndTypeLaser = 3
    EndTypePen = 4
    EndTypeMax = 5


# For EndTypeParams
class EndTypeParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("xBias", c_float),
        ("yBias", c_float),
        ("zBias", c_float)
    ]


class Pose(Structure):
    _pack_ = 1
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("rHead", c_float),
        ("joint1Angle", c_float),
        ("joint2Angle", c_float),
        ("joint3Angle", c_float),
        ("joint4Angle", c_float)
    ]


class Kinematics(Structure):
    _pack_ = 1
    _fields_ = [
        ("velocity", c_float),
        ("acceleration", c_float)
    ]


class AlarmsState(Structure):
    _pack_ = 1
    _fields_ = [
        ("alarmsState", c_int32)
    ]


class HOMEParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("r", c_float)
    ]


class HOMECmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("temp", c_float)
    ]


class EMotor(Structure):
    _pack_ = 1
    _fields_ = [
        ("index", c_byte),
        ("isEnabled", c_byte),
        ("speed", c_int32)
    ]


class EMotorS(Structure):
    _pack_ = 1
    _fields_ = [
        ("index", c_byte),
        ("isEnabled", c_byte),
        ("speed", c_int32),
        ("distance", c_uint32)
    ]


##################  Arm orientation定义   ##################
class ArmOrientation:
    LeftyArmOrientation = 0
    RightyArmOrientation = 1


##################  点动示教部分   ##################

class JOGJointParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("joint1Velocity", c_float),
        ("joint2Velocity", c_float),
        ("joint3Velocity", c_float),
        ("joint4Velocity", c_float),
        ("joint1Acceleration", c_float),
        ("joint2Acceleration", c_float),
        ("joint3Acceleration", c_float),
        ("joint4Acceleration", c_float)
    ]


class JOGCoordinateParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("xVelocity", c_float),
        ("yVelocity", c_float),
        ("zVelocity", c_float),
        ("rVelocity", c_float),
        ("xAcceleration", c_float),
        ("yAcceleration", c_float),
        ("zAcceleration", c_float),
        ("rAcceleration", c_float)
    ]


class JOGCommonParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("velocityRatio", c_float),
        ("accelerationRatio", c_float)
    ]


class JOGLParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("velocity", c_float),
        ("acceleration", c_float)
    ]


class JC:
    JogIdle = 0  # 空闲状态
    JogAPPressed = 1  # X + 前/ Joint1 + 左
    JogANPressed = 2  # X - 后/ Joint1 - 右
    JogBPPressed = 3  # Y + 左/ Joint2 + 前
    JogBNPressed = 4  # Y - 右/ Joint2 - 后
    JogCPPressed = 5  # Z + 上/ Joint3 + 下
    JogCNPressed = 6  # Z - 下/ Joint3 - 上
    JogDPPressed = 7  # R + Joint4 + 顺势针
    JogDNPressed = 8  # R - Joint4 - 逆时针
    JogEPPressed = 9  # L +
    JogENPressed = 10  # L -


class JOGCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("isJoint", c_byte),
        ("cmd", c_byte)
    ]


##################  再现运动部分   ##################

class PTPJointParams(Structure):
    _fields_ = [
        ("joint1Velocity", c_float),
        ("joint2Velocity", c_float),
        ("joint3Velocity", c_float),
        ("joint4Velocity", c_float),
        ("joint1Acceleration", c_float),
        ("joint2Acceleration", c_float),
        ("joint3Acceleration", c_float),
        ("joint4Acceleration", c_float)
    ]


class PTPCoordinateParams(Structure):
    _fields_ = [
        ("xyzVelocity", c_float),
        ("rVelocity", c_float),
        ("xyzAcceleration", c_float),
        ("rAcceleration", c_float)
    ]


class PTPLParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("velocity", c_float),
        ("acceleration", c_float)
    ]


class PTPJumpParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("jumpHeight", c_float),
        ("zLimit", c_float)
    ]


class PTPCommonParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("velocityRatio", c_float),
        ("accelerationRatio", c_float)
    ]


class PTPMode:
    PTP_JUMP_XYZ_Mode = 0
    PTP_MOVJ_XYZ_Mode = 1
    PTP_MOVL_XYZ_Mode = 2
    PTP_JUMP_ANGL_EMode = 3
    PTP_MOVJ_ANGL_EMode = 4
    PTP_MOVL_ANGL_EMode = 5
    PTP_MOVJ_ANGLE_INC_Mode = 6
    PTP_MOVL_XYZ_INC_Mode = 7
    PTP_MOVJ_XYZ_INC_Mode = 8
    PTP_JUMP_MOVL_XYZ_Mode = 9


class InputPin:
    InputPinNone = 0
    InputPin1 = 1
    InputPin2 = 2
    InputPin3 = 3
    InputPin4 = 4
    InputPin5 = 5
    InputPin6 = 6
    InputPin7 = 7
    InputPin8 = 8


class InputLevel:
    InputLevelBoth = 0
    IputLevelLow = 1
    InputLevelHigh = 2


class OutputPin:
    SIGNALS_O1 = 1
    SIGNALS_O2 = 2
    SIGNALS_O3 = 3
    SIGNALS_O4 = 4
    SIGNALS_O5 = 5
    SIGNALS_O6 = 6
    SIGNALS_O7 = 7
    SIGNALS_O8 = 8


class PTPCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("ptpMode", c_byte),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("rHead", c_float)
    ]


class PTPWithLCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("ptpMode", c_byte),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("rHead", c_float),
        ("l", c_float)
    ]


##################  Continuous path   ##################

class CPParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("planAcc", c_float),
        ("juncitionVel", c_float),
        ("acc", c_float),
        ("realTimeTrack", c_byte)
    ]


class ContinuousPathMode:
    CPRelativeMode = 0
    CPAbsoluteMode = 1


class CPCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("cpMode", c_byte),
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("velocity", c_float)
    ]


##################  圆弧：ARC   ##################
class ARCPoint(Structure):
    _pack_ = 1
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
        ("rHead", c_float)
    ]


class ARCParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("xyzVelocity", c_float),
        ("rVelocity", c_float),
        ("xyzAcceleration", c_float),
        ("rAcceleration", c_float)
    ]


class ARCCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("cirPoint", ARCPoint),
        ("toPoint", ARCPoint)
    ]


class CircleCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("cirPoint", ARCPoint),
        ("toPoint", ARCPoint)
    ]


##################  User parameters   ##################

class WAITParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("unitType", c_byte)
    ]


class WAITCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("waitTime", c_uint32)
    ]


class TRIGMode:
    TRIGInputIOMode = 0
    TRIGADCMode = 1


class TRIGInputIOCondition:
    TRIGInputIOEqual = 0
    TRIGInputIONotEqual = 1


class TRIGADCCondition:
    TRIGADCLT = 0
    TRIGADCLE = 1
    TRIGADCGE = 2
    TRIGADCGT = 3


class TRIGCmd(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte),
        ("mode", c_byte),
        ("condition", c_byte),
        ("threshold", c_uint16)
    ]


class GPIOType:
    GPIOTypeDO = 1
    GPIOTypePWM = 2
    GPIOTypeDI = 3
    GPIOTypeADC = 4


class IOMultiplexing(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte),
        ("multiplex", c_byte)
    ]


class IODO(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte),
        ("level", c_byte)
    ]


class IOPWM(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte),
        ("frequency", c_float),
        ("dutyCycle", c_float)
    ]


class IODI(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte),
        ("level", c_byte)
    ]


class IOADC(Structure):
    _pack_ = 1
    _fields_ = [
        ("address", c_byte),
        ("value", c_int)
    ]


class UserParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("params1", c_float),
        ("params2", c_float),
        ("params3", c_float),
        ("params4", c_float),
        ("params5", c_float),
        ("params6", c_float),
        ("params7", c_float),
        ("params8", c_float)
    ]


class ZDFCalibStatus:
    ZDFCalibNotFinished = 0
    ZDFCalibFinished = 1


# WIFI


class WIFIIPAddress(Structure):
    _pack_ = 1
    _fields_ = [
        ("dhcp", c_byte),
        ("addr1", c_byte),
        ("addr2", c_byte),
        ("addr3", c_byte),
        ("addr4", c_byte),
    ]


class WIFINetmask(Structure):
    _pack_ = 1
    _fields_ = [
        ("addr1", c_byte),
        ("addr2", c_byte),
        ("addr3", c_byte),
        ("addr4", c_byte),
    ]


class WIFIGateway(Structure):
    _pack_ = 1
    _fields_ = [
        ("addr1", c_byte),
        ("addr2", c_byte),
        ("addr3", c_byte),
        ("addr4", c_byte),
    ]


class WIFIDNS(Structure):
    _pack_ = 1
    _fields_ = [
        ("addr1", c_byte),
        ("addr2", c_byte),
        ("addr3", c_byte),
        ("addr4", c_byte),
    ]


class FirmwareSwitchMode:
    NO_SWITCH = 0,
    DOBOT_SWITCH = 1
    PRINTING_SWITCH = 2
    DRIVER1_SWITCH = 3
    DRIVER2_SWITCH = 4
    DRIVER3_SWITCH = 5
    DRIVER4_SWITCH = 6
    DRIVER5_SWITCH = 7


class FirmwareParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("mode", c_uint8)
    ]


class FirewareMode:
    INVALID_MODE = 0
    DOBOT_MODE = 1
    PRINTING_MODE = 2
    OFFLINE_MODE = 3


class UART4PeripheralsType:
    UART4PeripheralsUART = 0
    UART4PeripheralsWIFI = 1
    UART4PeripheralsBLE = 2
    UART4PeripheralsCH375 = 3


class PluseCmd:
    _pack_ = 1
    _fields_ = [
        ("j1", c_float),
        ("j2", c_float),
        ("j3", c_float),
        ("j4", c_float),
        ("e1", c_float),
        ("e2", c_float)
    ]


class PIDParams(Structure):
    _pack_ = 1
    _fields_ = [
        ("p", c_float),
        ("i", c_float),
        ("d", c_float),
        ("v", c_float),
        ("a", c_float)
    ]


class PID(Structure):
    _pack_ = 1
    _fields_ = [
        ("index", c_uint8),
        ("controlLoop", c_uint8),
        ("params", PIDParams),

    ]


# PORTS
class PortGp:
    PORT_GP1 = 0
    PORT_GP2 = 1
    PORT_GP4 = 2
    PORT_GP5 = 3


ColorPort = PortGp
InfraredPort = PortGp


class EMotorPort:
    EMOTOR_1 = 0
    EMOTOR_2 = 1


class IOFunction:
    IOFunctionDummy = 0  # 不配置功能
    IOFunctionDO = 1  # I/O 电平输出
    IOFunctionPWM = 2  # PWM 输出
    IOFunctionDI = 3  # I/O 电平输入
    IOFunctionADC = 4  # A/D 输入
    IOFunctionDIPU = 5  # 上拉输入
    IOFunctionDIPD = 6  # 下拉输入


##################  api, dobotId result   ##################

class DobotConnect:
    DobotConnect_Successfully = 0
    DobotConnect_NotFound = 1
    DobotConnect_Occupied = 2


CONNECT_RESULT = {
    DobotConnect.DobotConnect_Successfully: "DobotConnect_Successfully",
    DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}


class DobotCommunicate:
    DobotCommunicate_Successfully = 0
    DobotCommunicate_BufferFull = 1
    DobotCommunicate_Timeout = 2


Communicate_RESULT = {
    DobotCommunicate.DobotCommunicate_Successfully: "DobotCommunicate_Successfully",
    DobotCommunicate.DobotCommunicate_BufferFull: "DobotCommunicate_BufferFull",
    DobotCommunicate.DobotCommunicate_Timeout: "DobotCommunicate_Timeout"
}
