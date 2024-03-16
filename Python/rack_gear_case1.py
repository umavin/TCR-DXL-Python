from rack_gear_params import*
import numpy as np
import matplotlib.pyplot as plt

"""
Case1
"""

#Nitinol rod parameters
d_nt = 3.0
l_nt = 1000         
Y_nt = 83*1000                  #MPa
Q_nt = 28.8*1000                #MPa
k = 24.000


M_xm540 = 10.6*1000.00000

"""
#Gear-tooth parameters
Y_gt = 3000                     #MPa
F_gt = 81.4*1000                #MPa flexural strength
Q_gt = 52*1000                  #MPa 


#Wooden abacus parameters
m = 1.9200
phi = 20.000
#t1 to be tested
t1 = 4.0000
t2 = 3.890
l1 = 7.614
l2 = 1.518
l3 = 1.416
"""

""""
#PLA abacus gears testing module =4

#Gear-tooth parameters
Y_gt = 3600                     #MPa
F_gt = 81.4*1000                #MPa flexural strength
Q_gt = 52*1000                  #MPa 
m = 4
phi = 20.0000

#t1 to be tested
t1 = 6.283*6
t2 = 8.050
l1 = 15.901
l2 = 3.371
l3 = 2.93
"""
"""
#PLA abacus gears testing module =6

#Gear-tooth parameters
Y_gt = 3600                     #MPa
F_gt = 81.4*1000                #MPa flexural strength
Q_gt = 52*1000                  #MPa 
m = 4
phi = 20.0000

#t1 to be tested
t1 = 9.425*6
t2 = 12.050
l1 = 23.870
l2 = 5.052
l3 = 4.386
"""


#PLA abacus gears testing module =8

#Gear-tooth parameters
Y_gt = 3600                     #MPa
F_gt = 81.4*1000                #MPa flexural strength
Q_gt = 52*1000                  #MPa 
m = 4
phi = 20.0000

#t1 to be tested
t1 = 12.566
t2 = 16.050
l1 = 31.839
l2 = 6.743
l3 = 5.842


rack1 = rack_gear(d_inner_rod=d_nt,l_inner_rod=l_nt,Y_inner_rod=Y_nt,
                  Shear_inner_rod = Q_nt,gear_ratio=k,pressure_ang=phi,
                  modulus=m,Y_gear_tooth=Y_gt,Shear_gear_tooth=Q_gt,Tooth_thick1=t1,
                  Tooth_thick2=t2,Tooth_total_length=l1,Tooth_length=l2,Tooth_slope_length=l3,
                  Moment = M_xm540)

rack1.I2_rod_x()
rack1.I2_gear_tooth_x()

e1i1 = np.multiply(rack1.Y_rod,rack1.I2x_rod)

e2i2 = rack1.n*np.multiply(rack1.Y_tooth,rack1.I2x_gear_tooth)

ratio = e1i1/e2i2

print("E1I1:",e1i1,"E2I2:",e2i2,"Ratio:",ratio)

