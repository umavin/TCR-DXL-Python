from hemisphere_visual import*
#Central backbone bending angle parameters
r = 125.000    #Range is from 125.000 to inf 
d = 30.000
c = np.array([0,0,0])
theta = 50.000 #Range is from 50.00 to 0 
phi = 0.000    #Range is from 0.000 to 80.000

cr1 = h_sphere(r,c,theta,phi,d)




'''
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(coord_p[0],coord_p[1],coord_p[2],s=20.0,alpha=0.2,color='k')
ax.scatter(coord_s1[0],coord_s1[1],coord_s1[2],s=20.0,alpha=0.2,color='g')
ax.scatter(coord_s2[0],coord_s2[1],coord_s2[2],s=20.0,alpha=0.2,color='r')
# Set plot properties
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_box_aspect([1.0, 1.0, 1.0])

# Set the limits of the plot
ax.set_xlim([0, 0.5*r])
ax.set_ylim([-100, 100])
ax.set_zlim([0, r])
plt.show()
'''
