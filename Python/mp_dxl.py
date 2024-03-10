from cr_dxl import*
from dxl_sv import*

"""
This library is for developing level 0 motion primitives for the TCR
Motion primitives: --> Need to take into account current and whole shape of the robot ?
1. Extend 
2. Bend (in plane)
    2.a. Bend (out of plane 1)
    2.a. Bend (out of plane 2)
3. ...
4. ...
"""

"""    
All the _move commands take into account present position of the rack,
provide a relative position change. The goal is to create stackable motion primitives
that can be stacked to give a list of position changes for point-2-point motion with simple shapes
of the continuum robot.
""" 

class mprimitives:
    def __init__(self):
        self.cr_motors = cr_motion(24.000,[1,2,3,4,5,6])
        self.plin = self.cr_motors.dxl_ids[0:2]
        self.pxrot = self.cr_motors.dxl_ids[2]     #According to plate convention
        self.pxy120p = self.cr_motors.dxl_ids[0]   #Towards the operator
        self.pxy120n = self.cr_motors.dxl_ids[1]   #Away from the operator
        self.sxrot = self.cr_motors.dxl_ids[3]     #According to plate convention when
        self.sxy120p = self.cr_motors.dxl_ids[5]   #Towards the operator
        self.sxy120n = self.cr_motors.dxl_ids[4]   #Away from the operator

    def plin_move(self,step_size):
        
        
        
    def pbend_move(self,step_size,id):
        
    def slin_move(self,step_size):
        
    def sbend_move(self,step_size,id):