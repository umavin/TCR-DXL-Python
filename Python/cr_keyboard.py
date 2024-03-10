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
#print(goal_ang4)

ESC_ASCII_VALUE = 0x1b


i=0

while 1:
      
      print("Press any key to continue! (or press ESC to quit!)")
      if getch() == chr(ESC_ASCII_VALUE):
            break
      
      psec.move_sec(step_size=5.000)

#psec.dxl.shutdown()