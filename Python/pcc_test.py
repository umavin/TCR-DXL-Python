from pcc_dxl import*
import numpy as np

#test1 = pcc2dxl(24.0000,[1,2,3,4,5,6])

#test1.pcc2len_solve([1/125.00000,np.deg2rad(45.0000),np.deg2rad(60.000)],np.deg2rad([60.000,300.00,180.000]))

import sympy as sym

sym.init_printing()
delX,delY,delL,d = sym.symbols('delX,delY,delL,d')
delDet = sym.sqrt(delX*delX+delY*delY)
cdet = sym.cos(delDet/d)
sdet = sym.sin(delDet/d)


