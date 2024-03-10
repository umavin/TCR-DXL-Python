"""
Description:
Class for continuum robot parameter description and sending commands to dynamixel motors
to extend,retract and bend the telescopic sections of the continuum robot
Bending is based on a simple Forward kinematics model.
Inputs:
Primary rack motor DXL IDS, Secondary motor DXL IDS,

Class parameters:
1. Gear ratio
2. Keyboard commands

Class function:
1. Extend/retract
2. Bending
3. Return  to home position 
"""


import numpy as np
from dxl_sv import *
from pynput import keyboard

ESC_ASCII_VALUE = 0x1b

class cr_motion:
    
    def __init__(self,k,dxl_id):
        self.gear_ratio = k
        self.dxl_ids = dxl_id
        self.dxl = dxl_comms(dxl_id)
        self.dxl.com_initialize()
        self.dxl.set_operating_mode(4)      #Setting operation to multi-turn mode.
        self.dxl.set_profile_velocity(100)  #Currently setting the same profile velocity and accn for all motors.
        self.dxl.set_profile_acceleration(20) 
        self.dxl.read_initialize()
        self.dxl.write_initialize()
        self.n_links = len (dxl_id)
        self.step_size = 10                 #Default step _size
        self.dxl_mov_ids = [1,2,3,4,5,6]    #Default dxl ids for bending
        self.dxl_stop_ids = 0               #Deafult dxl ids for stopping ?
        self.dxl_id_prime = self.dxl_mov_ids[0:3]
        self.dxl_id_sec = self.dxl_mov_ids[3:6]
        
    #Define a function that calculates the motor angles of dxl upon initialization and stores the home position of the robot
        
    def move_check(self):
        while 1:
            move = self.dxl.isMoving()
            if all(num==0 for num in move):
                print("Motors stopped")
                break
                
        
        
    def motor_pose_reset(self):
        self.dxl.write_pos([100000,100000,100000,100000,100000,100000])
        self.move_check()
    
    def extend(self):
        step_size= np.min(self.step_size)       #Think of how to pass this off as a variable
        #Read step size from somewhere else ? But why ? Why not pass it off as an argument ?
        print('Robot extending')
        present_pos = self.dxl.getPos()
        goal_step = (step_size*1048576/(256*6*self.gear_ratio))*1    #Converting directly to dxl angles
        goal_pos = np.array(present_pos)+goal_step
        send_pos = list(goal_pos.astype(int))
        prev_pos = present_pos
        #Failsafe check for robot extending beyond last disk
        self.dxl.write_pos(send_pos)
        #write code for making python wait until motor stops
        self.move_check()
        present_pos = self.dxl.getPos()
        ext_length = (np.mean(present_pos) - np.mean(prev_pos))*256.000*6*self.gear_ratio/1048576.000
        print('Robot extended by:',ext_length)
          
    def retract(self):
        step_size = np.min(self.step_size)
        print('Robot retracting')
        present_pos = self.dxl.getPos()
        goal_step = (step_size*1048576/(256*6*self.gear_ratio))*1    #Converting directly to dxl angles
        goal_pos = np.array(present_pos)-goal_step
        send_pos = list(goal_pos.astype(int))
        prev_pos = present_pos
        #Checking code for failsafe protection against robot retracting beyond end disk
        self.dxl.write_pos(send_pos)
        #write code for making python wait until motor stops
        self.move_check()
        present_pos = self.dxl.getPos()
        ret_length = (np.mean(present_pos) - np.mean(prev_pos))*256.000*6*self.gear_ratio/1048576.000
        print('Robot retracted by:',ret_length)
    
    def ext_partial(self):   #Move single motor to bend continuum robot
        dxl_mov_ids = self.dxl_mov_ids
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] + goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        self.move_check()
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        
    def ret_partial(self):   #Move single motor to bend continuum robot
        dxl_mov_ids = self.dxl_mov_ids
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
        
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] - goal_step
    
        send_pos = goal_pos*1
        self.dxl.write_pos(send_pos)
        
        self.move_check()
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ret_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ret_length[0],"L2:",ret_length[1],"L3:",ret_length[2])
    
    def move(self,**kwargs):       #Have to give an array of dxl ids 
        self.dxl_mov_ids = kwargs['dxl_mov_ids']
        self.step_size   = kwargs['step_sizes']
        
        listener = keyboard.GlobalHotKeys({'<shift>+a':self.ext_partial,
                                '<shift>+d':self.ret_partial,
                                'w':self.extend,
                                's':self.retract})
        listener.start()
        while True:
            print("Press e to extend r to retract (or press ESC to quit!)")
            if getch() == chr(ESC_ASCII_VALUE):
                break 
        
        listener.stop()    

    def p_extend(self):
        dxl_mov_ids = self.dxl_id_prime
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] + goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check()
    
    def p_pbend1(self):
        dxl_mov_ids = self.dxl_id_prime[0]
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] + goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check()
    
    def p_pbend2(self):
        dxl_mov_ids = self.dxl_id_prime[1]
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] + goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check()
    
    def p_pbend3(self):
        dxl_mov_ids = self.dxl_id_prime[2]
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] + goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check()
        
    def p_retract(self):
        dxl_mov_ids = self.dxl_id_prime
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] - goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check()
    
    def p_nbend1(self):
        dxl_mov_ids = self.dxl_id_prime[0]
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] - goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check()
    
    def p_nbend2(self):
        dxl_mov_ids = self.dxl_id_prime[1]
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] - goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check()    
    
    def p_nbend3(self):
        dxl_mov_ids = self.dxl_id_prime[2]
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] - goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check()
    
    def s_extend(self):
        dxl_mov_ids = self.dxl_id_sec
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] + goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check()
    
    def s_pbend1(self):
        dxl_mov_ids = self.dxl_id_sec[0]
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] + goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check()
    
    def s_pbend2(self):
        dxl_mov_ids = self.dxl_id_sec[1]
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] + goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check()
    
    def s_pbend3(self):
        dxl_mov_ids = self.dxl_id_sec[2]
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] + goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check()
        
    def s_retract(self):
        dxl_mov_ids = self.dxl_id_sec
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] - goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check()
    
    def s_nbend1(self):
        dxl_mov_ids = self.dxl_id_sec[0]
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] - goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check()
    
    def s_nbend2(self):
        dxl_mov_ids = self.dxl_id_sec[1]
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] - goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check()    
    
    def s_nbend3(self):
        dxl_mov_ids = self.dxl_id_sec[2]
        
        dxl_mov_ids_index = np.array(dxl_mov_ids)-1
        present_pos = np.array(self.dxl.getPos())
        goal_pos = present_pos
        
        step_size_move = np.array(self.step_size)
      
        goal_step = (step_size_move*1048576/(256*6*self.gear_ratio))*1   #Converting directly to dxl angles
        goal_pos[dxl_mov_ids_index] = present_pos[dxl_mov_ids_index] - goal_step
        send_pos = list(goal_pos.astype(int))
        self.dxl.write_pos(send_pos)
        
        prev_pos = present_pos
        
        present_pos = self.dxl.getPos()
        ext_length = (present_pos - prev_pos)*256.000*6*self.gear_ratio/1048576.000
        print("Rack lengths changed by:","L1:",ext_length[0],"L2:",ext_length[1],"L3:",ext_length[2])
        self.move_check() 
    
    def move_sec(self,**kwargs):
        self.step_size   = kwargs['step_size']
        
        listener = keyboard.GlobalHotKeys({'w':self.p_extend,
                                's':self.p_retract,
                                'a':self.p_pbend1,
                                '<shift>+a':self.p_nbend1,
                                'd':self.p_pbend2,
                                '<shift>+d':self.p_nbend2,
                                'x':self.p_pbend3,
                                '<shift>+x':self.p_nbend3,
                                'u':self.s_extend,
                                'j':self.s_retract,
                                'h':self.s_pbend1,
                                '<shift>+h':self.s_nbend1,
                                'k':self.s_pbend2,
                                '<shift>+k':self.s_nbend2,
                                'm':self.s_pbend3,
                                '<shift>+m':self.s_nbend3})
        listener.start()
        while True:
            print("Press keyboard commands (or press ESC to quit!)")
            if getch() == chr(ESC_ASCII_VALUE):
                break 
        
        listener.stop()
    
            
    def ang2len(self,ang):   #Angles are in dxl counter
        lengths = ang*256.000*6*self.gear_ratio/1048576.000
        return lengths  #Lengths are in mm and are absolute values
    
    def len2ang(self,lengths):
        ang = (lengths*1048576/(256*6*self.gear_ratio))
        ang = np.array(list(map(int,ang)))
        return ang      #angle is in dxl counter