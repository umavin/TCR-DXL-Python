from dxl_sv import *
import keyboard
import msvcrt


dxl = dxl_comms([1,2,3,4,5,6])

dxl.com_initialize()
dxl.read_initialize()
dxl.write_initialize()

goal_pos1 = [1000,1000,1000,1000,1000,1000]
goal_pos2 = [3000,3000,3000,3000,3000,3000]
index=0
start_pos = [2048,2048,2048,2048,2048,2048]

dxl.write_pos(start_pos)
present_pos = dxl.getPos()
print("Present positions D1:",present_pos[0],"D2:",present_pos[1],
      "D3:",present_pos[2],"D4:",present_pos[3],"D5:",present_pos[4],
      "D6:",present_pos[5])


ESC_ASCII_VALUE = 0x1b

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(ESC_ASCII_VALUE):
        break
    
    if index==0:
        goal_pos = goal_pos2
        index=1
    else:
        index=0
        goal_pos = goal_pos1
    
    dxl.write_pos(goal_pos)
    
    present_pos = dxl.getPos()
    print("Present positions D1:",present_pos[0],"D2:",present_pos[1],
      "D3:",present_pos[2],"D4:",present_pos[3],"D5:",present_pos[4],
      "D6:",present_pos[5])
    

    