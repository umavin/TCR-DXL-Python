import keyboard
import msvcrt
from cr_dxl import*
import time as t

k=24.000
dxl_primary = [1,2,3,4,5,6]
dxl_secondary = [4,5,6]
psec = cr_motion(24.000,dxl_primary)
#dsec = cr_motion(24.000,dxl_secondary)



present_pos = psec.dxl.getPos()
present_lengths = psec.ang2len(np.array(present_pos))

print("Present Rack lengths L1:",present_lengths[0],"L2:",present_lengths[1],
      "L3:",present_lengths[2])

phi = np.deg2rad(np.linspace(0.000,360.0000,num=16,endpoint=True))
d = 30.000
delL = 200.000
theta = np.deg2rad(45.000)


l4 = (theta*d*np.sin(phi))*1+theta*d*np.sin(phi+np.deg2rad(300.000))
l5 = (theta*d*np.sin(phi+np.deg2rad(240.000)))*1+theta*d*np.sin(phi+np.deg2rad(300.000))
l6 = (theta*d*np.sin(phi+np.deg2rad(120.000)))*1+theta*d*np.sin(phi+np.deg2rad(180.000))

l1 = (theta*d*np.sin(phi+np.deg2rad(60.000)))*1
l2 = (theta*d*np.sin(phi+np.deg2rad(300.000)))*1
l3 = (theta*d*np.sin(phi+np.deg2rad(180.000)))*1
step_size = 30


#Primary section bending only
l1 = np.array([-step_size,0,step_size,0,-step_size,0,step_size,0,step_size,0,-step_size,0])#,-30,-30,30,30,-30,30,-30,-30,30])
l2 = np.array([step_size,0,-step_size,0,step_size,0,-step_size,0,step_size,0,-step_size,0])#,30,30,-30,-30,30,30,-30,-30,30])
l3 = np.array([step_size,0,-step_size,0,-step_size,0,step_size,0,-step_size,0,step_size,0])#,-30,-30,30,30,-30,-30,30,30,-30])

"""
#Secondary section bending only
l4 = np.array([-step_size,0,step_size,0,-step_size,0,step_size,0,step_size,0,-step_size,0])#,-30,-30,30,30,-30,30,-30,-30,30])
l5 = np.array([step_size,0,-step_size,0,step_size,0,-step_size,0,step_size,0,-step_size,0])#,30,30,-30,-30,30,30,-30,-30,30])
l6 = np.array([step_size,0,-step_size,0,-step_size,0,step_size,0,-step_size,0,step_size,0])#,-30,-30,30,30,-30,-30,30,30,-30])
"""

"""
#Extension and retraction 
step_size = 60
l1 = np.array([1,2,3,4,5,6,7,8,9,10])*step_size
l2 = np.array([1,2,3,4,5,6,7,8,9,10])*step_size
l3 = np.array([1,2,3,4,5,6,7,8,9,10])*step_size

l4 = np.array([1,2,3,4,5,6,7,8,9,10])*step_size*0.5
l5 = np.array([1,2,3,4,5,6,7,8,9,10])*step_size*0.5
l6 = np.array([1,2,3,4,5,6,7,8,9,10])*step_size*0.5
"""

"""
#Growing and Coiling
step_size = 20

l4 = np.array([100,200,300,300,300,300,300])
l5 = np.array([100,200,350,350,350,350,350])
l6 = np.array([100,200,350,360,360,360,360])

l1 = np.array([100,200,362.20084679,342.75661709,442.75661709,592.75661709,612.75661709])
l2 = np.array([100,200,304.46581987,331.27271057,431.27271057,581.27271057,581.27271057])
l3 = np.array([100,200,333.33333333,335.97067233,435.97067233,535.97067233,535.97067233])
"""

""""
#Throwing
l1 = np.array([-1.0,0,1.5,0])*step_size
l2 = np.array([-1.0,0,1.5,0])*step_size
l3 = np.array([1,0,-2.5,0])*step_size

l4 = np.array([-2,3,2.5,0])*step_size#,2.5])*step_size
l5 = np.array([2.5,3,2,0])*step_size#,-1.5])*step_size
l6 = np.array([2.5,3,2,0])*step_size#,-1.5])*step_size

#Extending
l1 = np.array([600])#*step_size
l2 = np.array([600])#*step_size
l3 = np.array([600])#*step_size

l4 = np.array([300])#*step_size#,2.5])*step_size
l5 = np.array([300])#*step_size#,-1.5])*step_size
l6 = np.array([300])#*step_size#,-1.5])*step_size
"""

goal_ang1 = present_pos[0] + psec.len2ang(l1)
goal_ang2 = present_pos[1] + psec.len2ang(l2)
goal_ang3 = present_pos[2] + psec.len2ang(l3)

goal_ang4 = present_pos[3] + psec.len2ang(l4)
goal_ang5 = present_pos[4] + psec.len2ang(l5)
goal_ang6 = present_pos[5] + psec.len2ang(l6)






#print(goal_ang4)

ESC_ASCII_VALUE = 0x1b


i=0

while 1:
      
      print("Press any key to continue! (or press ESC to quit!)")
      if getch() == chr(ESC_ASCII_VALUE):
            break
      
      
      while 1:
            #psec.dxl.write_pos([0,0,0,goal_ang4[i],goal_ang5[i],goal_ang6[i]])
            psec.dxl.write_pos([goal_ang1[i],goal_ang2[i],goal_ang3[i],present_pos[3],present_pos[4],present_pos[5]])
            
            #psec.dxl.write_pos([goal_ang1[i],goal_ang2[i],goal_ang3[i],goal_ang4[i],goal_ang5[i],goal_ang6[i]])
            t.sleep(0.5)
            #psec.move_check()
            i=i+1
            if i == len(l4):
                  break
           
      
      i=0
      #psec.move(dxl_mov_ids = [1,2,3,4,5,6],step_sizes=[0,0,0,goal_ang4[i],goal_ang5[i],goal_ang6[i]])
      #psec.move(dxl_mov_ids = [1,2,3],step_sizes=[0,0,1])   #Default pos command - psec.dxl.write_pos([0,0,0,0,0,0]
      #psec.move(dxl_mov_ids = [1,2,3],step_sizes=[2,2,0])
      #psec.move(dxl_mov_ids = [4,5,6],step_sizes=[-10,10,10])
      #psec.move(dxl_mov_ids = [1,2,3,4,5,6],step_sizes=[-20,20,20,-20,20,20])
      #psec.move(dxl_mov_ids = [1,2,3],step_sizes=[500,500,500])
      #move = psec.dxl.isMoving()

#psec.dxl.shutdown()