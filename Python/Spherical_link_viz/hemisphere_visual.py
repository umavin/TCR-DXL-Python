import numpy as np
import matplotlib.pyplot as plt
import math as mh
"""
This class is for defining sections of spheres of different radii and plotting them and their plane in 3D
The section of sphere is defined by 
r : radius
theta : angle of bending
phi : angle made by plane of bending wrt XZ plane  

"""
class h_sphere:
    
    
    def __init__(self,r,c,theta,phi,d):
        self.r = r
        self.theta = np.deg2rad(theta)
        self.phi = np.deg2rad(phi)
        self.c = c      #Centre of sphere
        self.d = d
    
    def sphere2cart(self,n):    # n is number of desired data points
        
        theta_range = np.linspace(0.000,self.theta,num=n)
        x = self.c[0] + np.multiply(self.r,(1-np.cos(theta_range)))*np.cos(self.phi)
        y = self.c[1] + np.multiply(self.r,1-np.cos(theta_range))*np.sin(self.phi)
        z = self.c[2] + self.r*np.sin(theta_range)
        
        
        return np.vstack((x,y,z))
    
    def plot_sphere_section(self,n):    #n is number of desired data points
        
        theta_range = np.linspace(self.theta,self.theta,num=n)
        
        x = self.r*np.sin(theta_range)*np.cos(self.phi)
        y = self.r*np.sin(theta_range)*np.sin(self.phi)
        z = self.r*np.cos(theta_range)
        
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(x,y,z,s=10.0,alpha=0.5)
        
        plt.show()
        
    def prack(self):    #Primary rack coordinates
        
        r_1   = self.r - self.d
        phi_1 = 0.000
        c_1   = np.array[self.d,0,0]
        
        r_2   = np.sqrt(self.r*self.r+self.d*self.d+self.r*self.d)
        phi_2 = 120.000
        c_2   = np.array[-0.5*self.d,0.5*np.sqrt(3)*self.d,0]
        
        r_3   = np.sqrt(self.r*self.r+self.d*self.d+self.r*self.d)
        phi_3 = -120.000
        c_3   = np.array[-0.5*self.d,-0.5*np.sqrt(3)*self.d,0]
        
        return r_1,phi_1,c_1,r_2,phi_2,c_2,r_3,phi_3,c_3
        
    def srack(self):    #Secondary rack coordinates
        
        r_1 =  np.sqrt(self.r*self.r+self.d*self.d-self.r*self.d)
        phi_1 = 60.000
        c_1 = np.array[0.5*self.d,0.5*np.sqrt(3)*self.d,0]
        
        r_2 =  np.sqrt(self.r*self.r+self.d*self.d-self.r*self.d)
        phi_2 = -60.000
        c_2 = np.array[0.5*self.d,-0.5*np.sqrt(3)*self.d,0]
        
        r_3   = self.r + self.d
        phi_3 = 0.000
        c_3   = np.array[self.d,0,0]
    
        return r_1,phi_1,c_1,r_2,phi_2,c_2,r_3,phi_3,c_3