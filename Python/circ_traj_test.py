import numpy as np
import matplotlib.pyplot as plt


phi = np.deg2rad(np.linspace(0.000,360.0000,num=50))
print(phi)

d = 30.000
delL = 200.000
theta = np.deg2rad(45.000)
L_1 = theta*d*np.sin(phi+np.deg2rad(60.000))
L_2 = theta*d*np.sin(phi+np.deg2rad(300.000))
L_3 = theta*d*np.sin(phi+np.deg2rad(180.000))
L_4 = theta*d*np.sin(phi)-0.5*(theta*d*np.sin(phi+np.deg2rad(60.000))+theta*d*np.sin(phi+np.deg2rad(300.000)))
L_5 = theta*d*np.sin(phi+np.deg2rad(240.000))-0.5*(theta*d*np.sin(phi+np.deg2rad(180.000))+theta*d*np.sin(phi+np.deg2rad(300.000)))
L_6 = theta*d*np.sin(phi+np.deg2rad(120.000))-0.5*(theta*d*np.sin(phi+np.deg2rad(60.000))+theta*d*np.sin(phi+np.deg2rad(180.000)))


plt.figure(1)

plt.plot(phi,L_1,alpha=0.2,label='L_1')
plt.plot(phi,L_2,alpha=0.2,label='L_2')
plt.plot(phi,L_3,alpha=0.2,label='L_3')

plt.plot(phi,L_4,alpha=0.2,label='L_4')
plt.plot(phi,L_5,alpha=0.2,label='L_5')
plt.plot(phi,L_6,alpha=0.2,label='L_6')
plt.legend()


"""
plt.plot(phi,theta*d*np.cos(phi),alpha=0.2,label='L_4')
plt.plot(phi,theta*d*np.cos(phi+np.deg2rad(240.000)),alpha=0.2,label='L_5')
plt.plot(phi,theta*d*np.cos(phi+np.deg2rad(120.000)),alpha=0.2,label='L_6')
plt.legend()
"""
plt.show()