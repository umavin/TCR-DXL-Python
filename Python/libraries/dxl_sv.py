'''
Compiled initialization of multiple dynamixel motors.
Further reading and writing to dynamixel motors has also been included. 
The main goal of this class is to make it easier to read and write to multiple dynamixel motors

Currently control table is set for Dynamixel XM-540. Future work will include addresses of XM-106 or any other motor used for Twist-snake.
Changes that may need to be made :
1. For XM series there are multiple control modes that change the way variables are defined
Default library can do position,speed control
Control modes to be addressed include:
a. Extended position mode
b. Current mode
c. Current-based position control

2. groupBulkRead and groupSyncWrite for position,speed,Load initialization can be left to user in the main code. Currently all of it is being initialized together.
3. Mayyyybe defining control tables of DXLs itself can be defined as a seperate class ? 
That way while initializing main code all you have to do is type model number (NOT A PRIORITY)
4. present_positions,present_speeds,present_loads are defined as empty array which can into memory . Need to find out the exact size of message to defined their sizes perfectly
'''

import os,sys
import numpy as np

#import rospy

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

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

class dxl_comms:
    
    def __init__(self,DXL_ID):
        '''
        Control table address
        This changes for Dynamixel XM-64 and XM-106. 
        Sets the torque control mode, PID gains and whether the goal is position,speed or current(loosely put torque).
        '''
        #Control Table for XM-540
        self.ADDR_XM_TORQUE_ENABLE       = 64               # Control table address is different in Dynamixel model
        self.ADDR_XM_GOAL_POSITION       = 116
        self.ADDR_XM_GOAL_TORQUE         = 102 # Current 2 byte
        self.ADDR_XM_GOAL_VELOCITY       = 104 # 4 byte
        self.ADDR_XM_GOAL_PWM            = 100 # 2 byte
        self.ADDR_XM_PRESENT_POSITION    = 132 # 4 byte
        self.ADDR_XM_PRESENT_SPEED       = 128 # 4 byte
        self.ADDR_XM_PRESENT_LOAD        = 126 # 2 byte
        self.ADDR_XM_MOVING              = 122 # 1 byte
        self.ADDR_XM_MOVING_STATUS       = 123 # 1 byte
        
        #Gain value addresses
        self.ADDR_XM_POS_P_GAIN          = 84  # All gains are 2bytes
        self.ADDR_XM_POS_I_GAIN          = 82  
        self.ADDR_XM_POS_D_GAIN          = 80 
        self.ADDR_XM_VEL_P_GAIN          = 78
        self.ADDR_XM_VEL_I_GAIN          = 76
        self.ADDR_XM_FFD1_GAIN           = 90  #Feedforward 1st gain
        self.ADDR_XM_FFD2_GAIN           = 88  #Feedforward 2nd gain
        
        #Operating mode addresses
        self.ADDR_XM_OPERATING_MODE      = 11
        self.ADDR_XM_DRIVE_MODE          = 10
        self.ADDR_XM_PROTOCOL_TYPE       = 13
        self.ADDR_XM_PROFILE_VELOCITY    = 112
        self.ADDR_XM_PROFILE_ACCELERATION= 108
        
        # Data Byte Length
        self.LEN_XM_GOAL_POSITION        = 4
        self.LEN_XM_GOAL_SPEED           = 4
        self.LEN_XM_GOAL_TORQUE          = 2
        self.LEN_XM_PRESENT_POSITION     = 4
        self.LEN_XM_PRESENT_SPEED        = 4
        self.LEN_XM_PRESENT_LOAD         = 2
        self.LEN_XM_MOVING               = 1
        #self.LEN_XM_TORQUE_CTRL_ENABLE   = 1
        # Protocol version
        self.PROTOCOL_VERSION            = 2.0               # Set which protocol version is used in the Dynamixel
        # Default setting
        self.BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
        self.DEVICENAME                  = 'COM14'    # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.TORQUE_CTRL_ENABLE          = 1                 # Value for enabling torque control
        self.TORQUE_CTRL_DISABLE         = 0                 # Value for disabling torque control
        #self.DXL_MINIMUM_POSITION_VALUE  = 100           # Dynamixel will rotate between this value
        #self.DXL_MAXIMUM_POSITION_VALUE  = 4000            # May need this to prevent damage to experimental setup
                                                            # Dynamixel moving status threshold --> FIND OUT WHAT THIS IS FOR
        self.DXL_ID = DXL_ID                                  #2 for NL , 1 for L
        
        # Units
        self.TO_DEGREE = 0.088             # [0-4096] -> [deg]
        self.TO_RAD = np.pi / 180.0        # [deg] -> [rad] 
        self.TO_RPM = 60.0 / (2.0 * np.pi) # [rad/s] -> [rpm]
        self.RPM_UNIT = 0.114              # [0-1023] -> [rpm]      # Dynamixel moving status threshold
        
    #This function initializes the communication handlers and confirms communication with each dynamixel motor.    
    def com_initialize(self):
        #Initialize port handler and packet handler
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate")
            getch()
            quit()
            
        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
        
    # This function initializes the synchronized reading of position,speed and load parameters of each dynamixel motor.    
    def read_initialize(self):
        self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)
        self.groupBulkReadPosition = GroupBulkRead(self.portHandler, self.packetHandler)
        self.groupBulkReadSpeed    = GroupBulkRead(self.portHandler, self.packetHandler)
        self.groupBulkReadMoving   = GroupBulkRead(self.portHandler, self.packetHandler)    #Moving checks whether goal position has been reached or not
        self.groupBulkReadLoad     = GroupBulkRead(self.portHandler, self.packetHandler)

        # Enable Dynamixel Torque
        # Need to put this in a loop and extend for multiple Dynamixels.
        for id in self.DXL_ID:
    
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_XM_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully connected" % id)   
        
        for id in self.DXL_ID:
            
            dxl_addparam_result = self.groupBulkReadPosition.addParam(id,self.ADDR_XM_PRESENT_POSITION,self.LEN_XM_PRESENT_POSITION)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkReadPosition addparam failed" % id)
                quit()

            dxl_addparam_result = self.groupBulkReadSpeed.addParam(id,self.ADDR_XM_PRESENT_SPEED,self.LEN_XM_PRESENT_SPEED)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkReadSpeed addparam failed" % id)
                quit()
                
            dxl_addparam_result = self.groupBulkReadLoad.addParam(id,self.ADDR_XM_PRESENT_LOAD,self.LEN_XM_PRESENT_LOAD)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkReadLoad addparam failed" % id)
                quit()

            #Moving checks whether goal position has been reached or not.
            dxl_addparam_result = self.groupBulkReadMoving.addParam(id,self.ADDR_XM_MOVING,self.LEN_XM_MOVING)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkReadLoad addparam failed" % id)
                quit()   
    
    #Only applicable for XM-series
    def set_operating_mode(self,op_mode):
        
        for id in self.DXL_ID:
    
            dxl_comm_result = self.packetHandler.write1ByteTxOnly(self.portHandler, id, self.ADDR_XM_OPERATING_MODE, op_mode)
            print("Dynamixel#%d operating mode changed to" % id,op_mode)   
    
    def set_drive_mode(self,drv_mode):  #Default drive mode is Velocity-based profile
        for id in self.DXL_ID:
            
            dxl_comm_result = self.packetHandler.wr(self.portHandler,id,self.ADDR_XM_DRIVE_MODE,drv_mode)
            print("Dynamixel#%d drive mode changed to" % id,drv_mode)   
    
    def set_profile_velocity(self,prof_vel):
        for id in self.DXL_ID:
            
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.ADDR_XM_PROFILE_VELOCITY, prof_vel)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d profile velocity changed succesfully to " % id,prof_vel*0.229," rev/min")   
          
    def set_profile_zcceleration(self,prof_acc):
        for id in self.DXL_ID:
            
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, self.ADDR_XM_PROFILE_ACCELERATION, prof_acc)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d profile acceleration changed succesfully to " % id,prof_acc*214.577," rev/min^2")   
                      
    # Initializes the synchronized writing of position and speed. Since torque control mode has to be enabled seperately and ignoes goal position and speed it has been initialized in enable_torque_ctrl(111111111112222)            
    def write_initialize(self):
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_XM_GOAL_POSITION,self.LEN_XM_GOAL_POSITION)
        self.groupSyncWriteSpeed = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_XM_GOAL_VELOCITY,self.LEN_XM_GOAL_SPEED)
        self.groupSyncWriteTorq = GroupSyncWrite(self.portHandler,self.packetHandler,self.ADDR_XM_GOAL_TORQUE,self.LEN_XM_GOAL_TORQUE)

    #May not need this for XM-series    
    def enable_torque_ctrl(self):
        for id in self.DXL_ID:
    
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_XM_TORQUE_CTRL_ENABLE, self.TORQUE_CTRL_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d torque control mode enabled." % id)
        
        self.groupSyncWriteTorque = GroupSyncWrite(self.portHandler, self.packetHandler,self.ADDR_XM_GOAL_TORQUE,self.LEN_XM_GOAL_TORQUE)
 
    def disable_torque_ctrl(self):
        for id in self.DXL_ID:
    
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_XM_TORQUE_CTRL_ENABLE, self.TORQUE_CTRL_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d torque control mode disabled." % id)

    
    def getPos(self):
        dxl_comm_result = self.groupBulkReadPosition.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        present_positions = []
        self.groupBulkReadPosition
        for id in self.DXL_ID:
            position = self.groupBulkReadPosition.getData(id,self.ADDR_XM_PRESENT_POSITION,self.LEN_XM_PRESENT_POSITION)
            present_positions.append(position)
        
        return present_positions
 
                
    def getSpeed(self):
        dxl_comm_result = self.groupBulkReadSpeed.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.
                  packetHandler.getTxRxResult(dxl_comm_result))
        present_speed = []
        for id in self.DXL_ID:
            speed = self.groupBulkReadPosition.getData(id,self.ADDR_XM_PRESENT_SPEED,self.LEN_XM_PRESENT_SPEED)
            present_speed.append(speed)
        
        return present_speed
        
        
    def getLoad(self):
        dxl_comm_result = self.groupBulkReadLoad.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.
                  packetHandler.getTxRxResult(dxl_comm_result))
        present_loads = []
        for id in self.DXL_ID:
            loads = self.groupBulkReadLoad.getData(id,self.ADDR_XM_PRESENT_LOAD,self.LEN_XM_PRESENT_LOAD)
            present_loads.append(loads)
        
        return present_loads
   
    
    def isMoving(self):
        dxl_comm_result = self.groupBulkReadMoving.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.
                  packetHandler.getTxRxResult(dxl_comm_result))
        moving_val = []
        for id in self.DXL_ID:
            moving = self.groupBulkReadMoving.getData(id,self.ADDR_XM_MOVING,self.LEN_XM_MOVING)
            moving_val.append(moving)
        
        return moving_val
    
    
    #Below are functions for sync writing speed,motor and torque signals. Note that to write torque signals, enable_torque_ctrl() must be run first.
    
    def write_pos(self,goal_pos):
        for id, position in zip(self.DXL_ID, goal_pos):
            #print("ID: ",id,"Pos: ",position)
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(position)),DXL_HIBYTE(DXL_LOWORD(position)),DXL_LOBYTE(DXL_HIWORD(position)),DXL_HIBYTE(DXL_HIWORD(position))] # To seperate direction and position magnitude of each position command ?
            dxl_addparam_result = self.groupSyncWrite.addParam(id, param_goal_position)    #loading desired positions for respective dynamixel
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % id)
                quit()
            
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        # Clear syncWrite parameter storage after every write
        self.groupSyncWrite.clearParam()    
 
    
    
    def write_speed(self,goal_speed):
        for id, speed in zip(self.DXL_ID, goal_speed):
            param_goal_speed = [DXL_LOBYTE(DXL_LOWORD(speed)),DXL_HIBYTE(DXL_LOWORD(speed)),DXL_LOBYTE(DXL_HIWORD(speed)),DXL_HIBYTE(DXL_HIWORD(speed))]
            dxl_addparam_result = self.groupSyncWriteSpeed.addParam(id, param_goal_speed)
        
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % id)
            quit()
        dxl_comm_result = self.groupSyncWriteSpeed.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        self.groupSyncWriteSpeed.clearParam()
        # time.sleep(0.01)
   
        
    def write_Torque(self,goal_torque):
        for id, torque in zip(self.DXL_ID, goal_torque):
            param_goal_torque = [DXL_LOBYTE(DXL_LOWORD(torque)), DXL_HIBYTE(DXL_LOWORD(torque))]
            dxl_addparam_result = self.groupSyncWriteSpeed.addParam(id, param_goal_torque)
        
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % id)
            quit()
        dxl_comm_result = self.groupSyncWriteTorque.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        self.groupSyncWriteTorque.clearParam()
        # time.sleep(0.01)