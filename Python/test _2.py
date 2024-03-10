import numpy as np
from dynamixel_sdk import*
import os,sys
import copy
import time
import msvcrt
import ctypes
import keyboard


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


# Control table address
ADDR_XM_TORQUE_ENABLE       = 64                          # Control table address is different in Dynamixel model
ADDR_XM_GOAL_POSITION       = 116
ADDR_XM_PRESENT_POSITION    = 132

# Protocol version
PROTOCOL_VERSION            = 2                             # See which protocol version is used in the Dynamixel

LEN_XM_GOAL_POSITION        = 4
LEN_XM_PRESENT_POSITION     = 4

# Default setting
DXL_ID                      = [1,2]                             # Dynamixel ID: 1
BAUDRATE                    = 57600
DEVICENAME                  = 'COM14'        # Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 1000                            # Dynamixel will rotate between this value
DXL_START_POS = 2048
DXL_MAXIMUM_POSITION_VALUE  = 3000                          # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                            # Dynamixel moving status threshold

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

# Units
TO_DEGREE = 0.088             # [0-4096] -> [deg]
TO_RAD = np.pi / 180.0        # [deg] -> [rad] 
TO_RPM = 60.0 / (2.0 * np.pi) # [rad/s] -> [rpm]
RPM_UNIT = 0.114              # [0-1023] -> [rpm]      # Dynamixel moving status threshold

dxl_goal_position1 = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE] 
dxl_goal_position2 = [DXL_MAXIMUM_POSITION_VALUE, DXL_MINIMUM_POSITION_VALUE]

index = 0                                                                               #Index sets the direction of tension setting 0 is in CCW, 1
# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

groupSyncWrite = GroupSyncWrite(portHandler,packetHandler,ADDR_XM_GOAL_POSITION,LEN_XM_GOAL_POSITION)
groupSyncRead  = GroupSyncRead(portHandler,packetHandler,ADDR_XM_PRESENT_POSITION,LEN_XM_PRESENT_POSITION)

# Enable Dynamixel 1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[0], ADDR_XM_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

# Enable Dynamixel 2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[1], ADDR_XM_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")


#Initialize motor to middle position (180deg in position mode)
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[0], ADDR_XM_GOAL_POSITION, DXL_START_POS)
if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
            
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[1], ADDR_XM_GOAL_POSITION, DXL_START_POS)
if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))


            
while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(ESC_ASCII_VALUE):
        break
    
    # Allocate goal position value into byte array
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position1[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position1[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position1[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position1[index]))]

    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[0], param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID[1])
        quit()

    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[1], param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID[1])
        quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()  
        
    
    
    while 1:
        
        #Group write command not sending commands but the code shows no error so its not sending the right bytes
        
        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = ctypes.c_ubyte(groupSyncRead.isAvailable(DXL_ID[0], ADDR_XM_PRESENT_POSITION, LEN_XM_PRESENT_POSITION)).value
        if dxl_getdata_result != 1:
            print("[ID:%03d] groupSyncRead getdata failed" % (DXL_ID[0]))
            quit()

        # Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = ctypes.c_ubyte(groupSyncRead.isAvailable(DXL_ID[1], ADDR_XM_PRESENT_POSITION, LEN_XM_PRESENT_POSITION)).value
        if dxl_getdata_result != 1:
            print("[ID:%03d] groupSyncRead getdata failed" % (DXL_ID[1]))
            quit()
#EDIT FROM HERE.
        # Get Dynamixel#1 present position value
        dxl1_present_position = groupSyncRead.getData(DXL_ID[0], ADDR_XM_PRESENT_POSITION, LEN_XM_PRESENT_POSITION)

        # Get Dynamixel#2 present position value
        dxl2_present_position = groupSyncRead.getData(DXL_ID[1], ADDR_XM_PRESENT_POSITION, LEN_XM_PRESENT_POSITION)

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n" % (DXL_ID[0], dxl_goal_position1[index], dxl1_present_position, DXL_ID[1], dxl_goal_position1[index], dxl2_present_position))

        if not ((abs(dxl_goal_position1[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) or (abs(dxl_goal_position1[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
            break
            
            
        while 1:
        # Syncread present position
            dxl_comm_result = groupSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

            # Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID[0], ADDR_XM_PRESENT_POSITION, LEN_XM_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] gwroupSyncRead getdata failed" % DXL_ID[0])
                quit()

            # Check if groupsyncread data of Dynamixel#2 is available
            dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID[1], ADDR_XM_PRESENT_POSITION, LEN_XM_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % DXL_ID[1])
                quit()

            # Get Dynamixel#1 present position value
            dxl1_present_position = groupSyncRead.getData(DXL_ID[0], ADDR_XM_PRESENT_POSITION, LEN_XM_PRESENT_POSITION)

            # Get Dynamixel#2 present position value
            dxl2_present_position = groupSyncRead.getData(DXL_ID[1], ADDR_XM_PRESENT_POSITION, LEN_XM_PRESENT_POSITION)

            print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID[0], dxl_goal_position1[index], dxl1_present_position, DXL_ID[1], dxl_goal_position1[index], dxl2_present_position))

            if not ((abs(dxl_goal_position1[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl_goal_position1[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
                break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0

# Clear syncread parameter storage
groupSyncRead.clearParam()