"""
Description :
Structural code for rack gear parameter calculations
Case 1 :
1.a.Polyacetal rack gear with gear tooth profile determined (module= 1.92 phi=20.000)
to calculate --> Desired gear offset for POM rack gear or base gear thickness 't1' 
In case of POM nitinol rod may not be present
b.TPU blended with PLA


2. Calculate desired young`s modulus for desired payload of 1-2kgs at max extension of 1000-1500mm
a. Assume case 1 parameters for gear tooth and calculate maximum bending and shear stress
b. Vary gear dimension and measure the maximum bending and shear stress
    
"""


import numpy as np
from scipy.optimize import*
import matplotlib.pyplot as plt

class rack_gear:
    
    def __init__(self,**kwargs):
    #Refer to page 134-136 of research journal
        self.d_rod = kwargs['d_inner_rod']
        self.l_rod = kwargs['l_inner_rod']
        self.Y_rod = kwargs['Y_inner_rod']
        self.Q_rod = kwargs['Shear_inner_rod']
        
        self.k = kwargs['gear_ratio']
        self.phi = kwargs['pressure_ang']
        self.m = kwargs['modulus']
        self.Y_tooth = kwargs['Y_gear_tooth']
        self.Q_tooth = kwargs['Shear_gear_tooth']
        self.t1 = kwargs['Tooth_thick1']        #Gear offset --> To be calculated
        self.t2 = kwargs['Tooth_thick2']
        self.L1 = kwargs['Tooth_total_length']
        self.L2 = kwargs['Tooth_length']
        self.L3 = kwargs['Tooth_slope_length']
        self.n = int(np.divide(self.l_rod,self.L1)) #Number of teeth
        
        self.I2x_rod = 0.000000
        self.I2y_rod = 0.000000
        
        self.I2x_gear_tooth = 0.000000
        self.I2y_gear_tooth = 0.000000
                   
        self.M = kwargs['Moment']               #Applied moment calculate based on stall torque of DXL-XM540
        
    def I2_cylinder_xy(self,r1,r2,d):
        
        A = np.pi*(r2*r2-r1*r1)
        
        I2 = 0.25*np.pi*(np.power(r2,4)-np.power(r1,4))
        
        I2_ecc = I2 + A*d*d
        
        return I2_ecc
    
    def I2_cylinder_z(self,r1,r2,d):
        
        A = np.pi*(r2*r2-r1*r1)
        
        I2 = 0.5*np.pi*(np.power(r2,4)-np.power(r1,4))
        
        I2_ecc = I2 + A*d*d
        
        return I2_ecc
        
    def I2_triangle_x(self,b,h,a,d):
        
        A = b*h
        
        I2 = (b*np.power(h,3))/12.00000
        
        I2_ecc = I2 + A*d*d
        
        return I2_ecc
    
    def I2_triangle_y(self,b,h,a,d):
        
        A = b*h
        
        I2 = (h*np.power(b,3)+np.power(b,2)*h*a+np.power(a,2)*h*b)/12.00000
        
        I2_ecc = I2 + A*d*d
        
        return I2_ecc
    
    def I2_rectangle_x(self,b,h,d):
        A = b*h
        
        I2 = (np.power(h,3)*b)/12.00000
        
        I2_ecc = I2 + A*d*d
        
        return I2_ecc
    
    def I2_rectangle_y(self,b,h,d):
        A = b*h
        
        I2 = (np.power(b,3)*h)/12.00000
        
        I2_ecc = I2 + A*d*d
        
        return I2_ecc
    
    def I2_perpendicular(self,**kwargs):
        
        Ix = kwargs["Ix"]
        Iy = kwargs["Iy"]
        
        Iz = Ix + Iy
        
        return Iz
    
    def I2_rod_x(self):
        self.I2x_rod = self.I2_rectangle_x(self.d_rod,self.l_rod,0)
        
    def I2_rod_y(self):
        self.I2y_rod = self.I2_rectangle_y(self.d_rod,self.l_rod,0.000)
    
    def I2_gear_tooth_x(self):
        #Refer to page 136 bottom
        base_length1 = self.d_rod + 2*self.t1
        height1 = self.L1
        d1 = 0.000
        I2_rect1 = self.I2_rectangle_x(base_length1,height1,d1)
        base_length2 = self.t2
        height2 = self.L2
        d2 = 0.5*self.d_rod+self.t1+0.5*self.t2
        I2_rect2 = self.I2_rectangle_x(base_length2,height2,d2)*2.00000000000
        b3 = self.t2
        h3 = self.L3
        d3x = 0.5*self.d_rod+self.t1
        d3y = 0.5*self.L2
        d3 = np.sqrt(d3x*d3x+d3y*d3y)
        I2_triangle3 = self.I2_triangle_x(b3,h3,0.00000,d3)*4.000000
        self.I2x_gear_tooth = I2_rect1 + I2_rect2 + I2_triangle3

    def I2_gear_tooth_y(self):
        #Refer to page 136 bottom
        base_length1 = self.d_rod + 2*self.t1
        height1 = self.L1
        d1 = 0.000
        I2_rect1 = self.I2_rectangle_y(base_length1,height1,d1)
        base_length2 = self.t2
        height2 = self.L2
        d2 = 0.5*self.d_rod+self.t1
        I2_rect2 = self.I2_rectangle_y(base_length2,height2,d2)*2.00000000000
        b3 = self.t2
        h3 = self.L3
        d3x = 0.5*self.d_rod+self.t1
        d3y = 0.5*self.L2
        d3 = np.sqrt(d3x*d3x+d3y*d3y)
        I2_triangle3 = self.I2_triangle_y(b3,h3,0.00000,d3)*4.000000
        self.I2y_gear_tooth = I2_rect1 + I2_rect2 + I2_triangle3
    
  #CONTINUE FROM HERE      
   # def bending_stress_calc(self):
        
                
   # def shear_stress_calc(self):