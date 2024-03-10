"""
PCC library for two segment Telescopic continuum robot. Perform 
forward and inverse kinematics based on the lengths of racks based on
the PCC assumption.
Primary section DXL IDs : [1,2,3]
Secondary section DXL IDs : [4,5,6]
Contributions:
1. Modeling geared-rod driven telescopic robot as a two-segment PCC
2. Modeling PCC in python as a q-space parametrization according to Cosimo et.al.[1] and performing inverse kinematics
3. 

Assumptions:
1. In the TCR, the base segment curvature is determined completely by the rack lengths of the secondary section
2. The bending of both PCC segments can be indpendently controlled by the 3 racks of secondary and primary respectively
3. For the ground plane coordinate systems X-axis is in the direciton of motor 4

References:
[1] "On an Improved State Parametrization for Soft Robots with Piecewise Constant Curvature and Its Use in Model Based Control" Della Santina C et.al. RA-L(2020)

"""
from dxl_sv import*
from cr_dxl import*
import numpy as np
from scipy.optimize import fsolve
from scipy.optimize import curve_fit 
import math as mh

class pcc2dxl:
    def __init__(self,k,DXL_ID):
        self.gear_ratio = k
        self.dxl_ids = DXL_ID
        self.tcr = cr_motion(k,DXL_ID)
        self.d = 60.00  #Distance of rack gear center from center of disk
        
    def len2pcc(self,lens):
        l1 = lens[0]
        l2 = lens[1]
        l3 = lens[2]
        d = self.d
        k = np.divide(np.sqrt(l1*l1+l2*l2+l3*l3-l1*l2-l2*l3-l3*l1)*2.000000000,d*(l1+l2+l3))
        theta = np.divide(np.sqrt(l1*l1+l2*l2+l3*l3-l1*l2-l2*l3-l3*l1)*2.000000000,d*3.00000000000000)
        phi = np.arctan2(np.sqrt(3)*(l2+l3-2*l1),2*(l2-l3))
        
        return (k,theta,phi)
    
    def pcc2len(self,pcc_params): #phase is the angle of the rods from ground X-axis in anti-clockwise direction
        L=np.zeros(3)
        k = pcc_params[0]
        theta = pcc_params[1]
        phi = pcc_params[2]
        delL = theta/k
        L[0] = delL - self.d*theta*mh.cos(phi+mh.radians(60))
        L[1] = delL - self.d*theta*mh.cos(phi+mh.radians(180))
        L[2] = delL - self.d*theta*mh.cos(phi+mh.radians(300))
        
        return L
          
        
    def pcc2qspace(self,pcc_params_curr):
        k = pcc_params_curr[0]
        theta = pcc_params_curr[1]
        phi = pcc_params_curr[2]
        
        #This formulation has to be modified depending on the orientation of grond coordinate system as definition of phi changes
        delX = np.multiply(theta,self.d,np.cos(phi))
        delY = np.multiply(theta,self.d,np.sin(phi))
        delL = np.divide(theta,k)
        delDet = np.sqrt(delX*delX+delY*delY)
        
        return delX,delY,delL,delDet
    
    def qspace2pcc(self,qspace_params):
        delX = qspace_params[0]
        delY = qspace_params[1]
        delL = qspace_params[2]
        delDet = qspace_params[3]
        
        pcc_params = np.zeros(3)
        
        pcc_params[1] = np.divide(delDet,self.d)
        pcc_params[2] = np.arccos(np.divide(delX,delDet))
        pcc_params[0] = np.divide(pcc_params[1],delL)
        
        return pcc_params
            
    def qspace2cart(self,qspace_params):
        
        delX = qspace_params[0]
        delY = qspace_params[1]
        delL = qspace_params[2]
        delDet = qspace_params[3]
        
        ndelX = np.divide(delX,delDet)
        ndelY = np.divide(delY,delDet)
        ndelDet = np.divide(delDet,self.d)
        #Rotation matrix with q space parametrization
        qR = np.array([1+np.multiply(np.cos(ndelDet)-1,ndelX,ndelX),np.multiply(ndelX,ndelY,(np.cos(ndelDet)-1)),np.multiply(-ndelX,np.sin(ndelDet))],
                      [np.multiply(ndelX,ndelY,(np.cos(ndelDet)-1)),1+np.multiply(np.cos(ndelDet)-1,ndelY,ndelY),np.multiply(-ndelY,np.sin(ndelDet))],
                      np.multiply(ndelX,np.sin(ndelDet)),np.multiply(ndelY,np.sin(ndelDet)),np.cos(delDet))
        #Translation matrix
        qt = np.multiply(np.divide(np.multiply(self.d,delL),np.multiply(delDet,delDet)),np.array([np.multiply(delX,(1-np.cos(ndelDet))),np.multiply(delY,(1-np.cos(ndelDet))),np.multiply(delDet,np.sin(ndelDet))]))

        
        return qR,qt
        
    #def qspace2pcc(self,qspace_params):
          
    def f_kinematics(self,goal_lengths):
        self.goal_ang = self.tcr.len2ang(goal_lengths)
        self.slens = goal_lengths[3:5]
        self.plens = goal_lengths[0:2]
        pcc_2 = self.len2pcc(self.slens)
        k2,theta2,phi2 = pcc_2
        c1 = np.array(-0.5*self.d,)
        l1s = np.divide(theta2,k2)+np.multiply(self.d,theta2,np.cos(phi2+np.deg2rad(60.000)))
        l2s = np.divide(theta2,k2)+np.multiply(self.d,theta2,np.cos(phi2+np.deg2rad(300.000)))
        l3s = np.divide(theta2,k2)+np.multiply(self.d,theta2,np.cos(phi2+np.deg2rad(180.000)))
        l1p = self.plens[0]-l1s
        l2p = self.plens[1]-l2s
        l3p = self.plens[2]-l3s
        
        pcc_1 = self.len2pcc([l1p,l2p,l3p])
        #Got pcc parameters of both sections 1 and 2 
        #Now to convert them into global cartesian coordinates
        l_all = self.tcr.ang2len(self.tcr.dxl.getPos())

    def circ_traj():
    
    
    #def inv_kinematics(self,**kwargs):  
    # #This includes the orientation x,y,z,thetax,thetay,thetaz in cartesian coordinates
    #The first solution will assume inputs are desired pcc paramters and compute the desired lengths as system of non-linear equations
    #Inputs = qspace params
    #Outputs = l1,l2,l3,l4,l5,l6
        