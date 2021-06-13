import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

os.sys.path.append('../dynamixel_functions_py')             # Path setting
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
portHandler = PortHandler('COM3')
packetHandler = PacketHandler(2.0)


ADDR_PRO_GOAL_CURRENT       = 102
ADDR_PRO_PRESENT_CURRENT    = 126
ADDR_PRO_HARDWARE_ERROR     = 70

def deg2pulse(deg):
    # converts from degrees of joint displacement to motor pulses
    ratio = 0.45
    return int(deg*(ratio/0.088))

def curr2Amps(current_reading):
    return float(current_reading*2.69/1000)

def torque2current(torque):
    # convert based on tau = Kt*I
    current_amps = (torque*0.95 + 0.1775)
    # convert to current units used by motor
    return int(current_amps*1000/2.69)

def current2torque(current_amps):
    return float((current_amps - 0.1775)/0.95)

# def calc_torque(DXLS, ifprint):
#     # calculate the torque based on measured current
#     dxl_present_current = [0]*max(DXLS)
#     torque = [0]*max(DXLS)
#     count = 0
#     for motor_id in DXLS:
#         # Read present current
#         dxl_present_current[motor_id-1], dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, motor_id, ADDR_PRO_PRESENT_CURRENT)
#         if dxl_comm_result != COMM_SUCCESS:
#             print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#         elif dxl_error != 0:
#             print("[ID:%-2d]: %s" % (motor_id, packetHandler.getRxPacketError(dxl_error)))
#             dxl_error_message, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, motor_id, ADDR_PRO_HARDWARE_ERROR)
#             # print(dxl_error_message)
#             # print(dxl_comm_result)
#             # print(dxl_error)
#
#         # convert current unit to amps
#         dxl_present_current[motor_id-1] = curr2Amps(dxl_present_current[motor_id-1])
#
#         if dxl_present_current[motor_id-1] > 150:
#             # when the cables are slack, the current is really large, but toque is actually zero so set to zero
#             torque[motor_id-1] = 0
#         else:
#             torque[motor_id-1] =current2torque(dxl_present_current[motor_id-1])
#         count = count+1
#     if ifprint == True:
#         measurements = ''
#         for motor_id in DXLS:
#             # measurements = measurements + ("    [ID:%02d] PresTorque:%3.2fNm, PresCurrent:%8.4fA" % (DXL_TENSIONED[i], torque[i], dxl_present_current[i]))
#             measurements = measurements + ("    [ID:%02d] Torque: %3.2fNm" % (motor_id, torque[motor_id-1]))
#         print(measurements)
#     return torque, dxl_present_current
