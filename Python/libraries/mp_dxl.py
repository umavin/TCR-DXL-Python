from cr_dxl import*
from dxl_sv import*

class mprimitives:
    def __init__(self):
        self.cr_motors = cr_motion(24.000,[1,2,3,4,5,6])
        self.plin = 