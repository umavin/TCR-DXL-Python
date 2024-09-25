import numpy as np
import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib as mpl
import matplotlib
#matplotlib.use("Agg")



class plt_QPcc:
    
    def __init__(
            self,
            QPcc_set,
            d,
            m,
            fname):
        
        self.qpcc_set = QPcc_set #Assuming input shape of [n*3,m] rows denote all qpcc coordinates 
        self.n =int(len(QPcc_set[:,0])/3)
        self.sz = len(QPcc_set[0,:])
        self.m = m #Number of discretizations for plotting
        self.i_glb_max = len(QPcc_set[0,:])
        #self.qseg = np.reshape(QPcc,[3,self.n])
        self.Tf_seg = np.ones((4,4,self.n+1,self.m))
        self.lmax = -1000    #Can be changed
        self.d = d

        for i in range(0,self.n+1):
            for j in range(0,self.m):
                self.Tf_seg[:,:,i,j] = np.ones(4)


        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111,projection='3d')
        self.ax.view_init(elev=45,azim=30,roll=0)
        #self.ax1 = self.fig2.add_subplot(111,projection='3d')
        #self.ax1.view_init(elev=0,azim=-90,roll=0)
        #self.line, = self.ax.plot([],[],[],lw=2)

        self.coords = np.zeros((3,self.n*self.m))
        self.i_glb = 0

        self.label = []
        self.coords_traj = np.zeros((3,self.sz))
        # Set up formatting for the movie files
        self.Writer = animation.writers['ffmpeg']
        self.writer = self.Writer(fps=30, metadata=dict(artist='Me'), bitrate=-1)
        self.fname = fname + '.mp4'
    
    def Qseg_tf(self,qpcc):

        qseg = np.reshape(qpcc,[3,self.n])
        delX = qseg[0,:]
        delY = qseg[1,:]
        delL = qseg[2,:]
        delta = np.sqrt(delX**2+delY**2)
        ndelX = np.divide(delX,delta)
        ndelY = np.divide(delY,delta)
        Rcos = np.cos(delta)
        Rsin = np.sin(delta)
        
        Rmat = np.array([[1+(Rcos-1)*ndelX*ndelX , 
                               (Rcos-1)*ndelX*ndelY ,
                               -ndelX*Rsin],
                         [(Rcos-1)*ndelX*ndelY ,
                          1+(Rcos-1)*ndelY*ndelY ,
                          -ndelY*Rsin ],
                         [ Rsin*ndelX ,
                          Rsin*ndelY , 
                          Rcos]])   # rotation matrix arrays in the shape of 3,3,n
        Tr1 = np.divide(self.d*(delL),(delta*delta))
        #print("Tr1 : " , Tr1)
        Trx = Tr1*delX*(1-Rcos)     #np.multiply(Tr1,np.reshape(delX,[1,n]),(1-np.reshape(Rcos,[1,n])))
        Try = Tr1*delY*(1-Rcos)         #np.multiply(Tr1,np.reshape(delY,[1,n]),(1-np.reshape(Rcos,[1,n])))
        Trz = Tr1*delta*Rsin         #np.multiply(Tr1,np.reshape(delta,[1,n]),np.reshape(Rsin,[1,n]))
        Trmat = np.vstack((Trx,Try,Trz))  # translation matrix arrays in the shape of 3,n
        
        Tmat_br = np.array([0,0,0,1])

        Tr_all = np.zeros((4,4,self.n+1))

        for i in range(0,self.n):
            Tmat_int_temp = np.hstack((Rmat[:,:,i],np.reshape(Trmat[:,i],[3,1])))
            #ARRAY NOT BEING ASSIGNED IN JAX NUMPY ARRAYS
            Tmat_int_temp = np.vstack((Tmat_int_temp,Tmat_br))
            self.Tr_all[:,:,i+1] = Tmat_int_temp      #Ref.[3]

    def Qaseg_tf_discrete(self,qpcc):

        qseg = np.reshape(qpcc,[self.n,3]).T
        delX = qseg[0,:]
        delY = qseg[1,:]
        delL = qseg[2,:]
        delta = np.zeros(len(delL))
        for i in range (0,len(delL)):
            if (delX[i]<0):
                delta[i] = -np.sqrt(delX[i]**2+delY[i]**2)
            else:
                delta[i] = np.sqrt(delX[i]**2+delY[i]**2)
        #delta = np.sqrt(delX**2 + delY**2)
        phi = np.arcsin(np.divide(delY,delta))
        theta = delta/self.d
        kappa = theta/delL

        Tf_seg = self.Tf_seg
        Tmat_br = np.array([0,0,0,1])
        for i in range(1,self.n+1):
            theta_d = np.linspace(0.0001,theta[i-1],num=self.m)
            cth = np.cos(theta_d)
            sth = np.sin(theta_d)
            onz = np.ones((self.m,))

            #print("Cos theta:", np.shape(cth))

            cph = np.cos(phi[i-1])*onz
            sph = np.sin(phi[i-1])*onz
            kappa_d = kappa[i-1]*onz
            
            #print("kappa : ", np.shape(kappa_d))


            #Conventional 
            Tf_mat_temp1 = np.array([[cph*cth,-sph,np.multiply(cph,sth),np.divide(cph*(1-cth),kappa_d)],
                        [sph*cth,cph,np.multiply(sph,sth),np.divide(sph*(1-cth),kappa_d)],
                        [ -sth ,np.zeros((self.m,)),cth,np.divide(sth,kappa_d)]])
            
            #Bishop's
            
            Tf_mat_temp2 = np.array([[(cth-1)*cph**2+1,sph*cph*(cth-1),np.multiply(cph,sth),np.divide(cph*(1-cth),kappa_d)],
                        [sph*cph*(cth-1) , (cth-1)*cph**2+cth , sph*sth , np.divide(sph*(1-cth),kappa_d) ],
                        [ -sth*cph ,-sph*sth,cth,np.divide(sth,kappa_d)]])

            for k in range(0,self.m):
                Tf_seg[:,:,i,k] = np.vstack((Tf_mat_temp2[:,:,k],Tmat_br))
                #matrix multiplication is sort of working needs to be checked
                #Tf_seg[:,:,i,k] = np.dot(Tf_seg[:,:,i,k],Tf_seg[:,:,i-1,-1]) 
                #print("Transform: ",Tf_seg[:,:,i,k])
                #Tf_seg[0:3,0:3,i,k] = np.dot(Tf_seg[0:3,0:3,i-1,-1],Tf_seg[0:3,0:3,i,k])
                #Tf_seg[0:3,3,i,k] = Tf_seg[0:3,3,i-1,-1]+Tf_seg[0:3,3,i,k]
                if i>1:
                    Tf_seg[0:3,3,i,k]  = Tf_seg[0:3,3,i-1,-1]+np.matmul(Tf_seg[0:3,0:3,i-1,-1],Tf_seg[0:3,3,i,k])

        return Tf_seg

    def Qseg_tf_discrete(self,qpcc):

        qseg = np.reshape(qpcc,[self.n,3]).T
        
        Tf_seg = self.Tf_seg
        Tmat_br = np.array([0,0,0,1])
        for i in range(1,self.n+1):
            delX_d = np.linspace(0.001,qseg[0,i-1],num=self.m)
            delY_d = np.linspace(0.001,qseg[1,i-1],num=self.m)
            delL_d = np.linspace(0.001,qseg[2,i-1],num=self.m)
            delta = np.sqrt(delX_d**2+delY_d**2)
            ndelX = np.divide(delX_d,delta)
            ndelY = np.divide(delY_d,delta)
            Rcos = np.cos(delta)
            Rsin = np.sin(delta)
            

            
             # rotation matrix arrays in the shape of 3,3,n
            Tr1 = np.divide(self.d*(delL_d),(delta*delta))
            #print("Tr1 : " , Tr1)
            Trx = Tr1*delX_d*(1-Rcos)     #np.multiply(Tr1,np.reshape(delX,[1,n]),(1-np.reshape(Rcos,[1,n])))
            Try = Tr1*delY_d*(1-Rcos)         #np.multiply(Tr1,np.reshape(delY,[1,n]),(1-np.reshape(Rcos,[1,n])))
            Trz = Tr1*delta*Rsin         #np.multiply(Tr1,np.reshape(delta,[1,n]),np.reshape(Rsin,[1,n]))
              # translation matrix arrays in the shape of 3,n
            #print("Trx:",Trx)
            Tf_mat_temp = np.array([[1+(Rcos-1)*ndelX*ndelX,(Rcos-1)*ndelX*ndelY ,-ndelX*Rsin,Trx],
                        [(Rcos-1)*ndelX*ndelY,1+(Rcos-1)*ndelY*ndelY ,-ndelY*Rsin,Try],
                        [ Rsin*ndelX ,Rsin*ndelY ,Rcos,Trz]])
            for k in range(0,self.m):
                Tf_seg[:,:,i,k] = np.vstack((Tf_mat_temp[:,:,k],Tmat_br))
                #matrix multiplication is sort of working needs to be checked
                Tf_seg[:,:,i,k] =np.matmul(Tf_seg[:,:,i-1,-1],Tf_seg[:,:,i,k]) 
                #print("Transform: ",Tf_seg[:,:,i,k])

        return Tf_seg

    def update_QPCC_set(self,QPcc_set):
        self.qpcc_set = QPcc_set

    def update_qpcc(self,j):
        qpcc = self.qpcc_set[:,j]
        return qpcc

    def update_plot(self,j):
        
        
        qpcc = self.update_qpcc(j)
        Tf_seg = self.Qaseg_tf_discrete(qpcc)
        
        self.coords_traj[0,j] = Tf_seg[0,3,self.n,self.m-1]
        self.coords_traj[1,j] = Tf_seg[1,3,self.n,self.m-1]
        self.coords_traj[2,j] = Tf_seg[2,3,self.n,self.m-1]        
        plt.cla()
        self.ax.set_title("Simulation of Q-parametrized PCC")
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_zlabel("Z (mm)")
        

        self.ax.set_xlim([-0.5*self.lmax,0.5*self.lmax])
        self.ax.set_ylim([-0.5*self.lmax,0.5*self.lmax])
        self.ax.set_zlim([0,self.lmax])
    



        for i in range(1,self.n+1):
            self.coords[0,(i-1)*self.m:(i)*self.m] = Tf_seg[0,3,i,:]
            self.coords[1,(i-1)*self.m:(i)*self.m] = Tf_seg[1,3,i,:]
            self.coords[2,(i-1)*self.m:(i)*self.m] = Tf_seg[2,3,i,:]
            
            

            #Later on add quiver function to mark orientation arrow and force arrow for dynamics!
            labl = 'PCC segment' + str(i)
            plt.plot(self.coords[0,(i-1)*self.m:(i)*self.m],self.coords[1,(i-1)*self.m:(i)*self.m],self.coords[2,(i-1)*self.m:(i)*self.m],label=labl,linewidth=2.5)
            #self.line.set_data(self.coords[0,(i-1)*self.m:(i)*self.m],self.coords[1,(i-1)*self.m:(i)*self.m],self.coords[2,(i-1)*self.m:(i)*self.m])
            #ax1.plot(self.coords[0,(i-1)*self.m:(i)*self.m],self.coords[1,(i-1)*self.m:(i)*self.m],self.coords[2,(i-1)*self.m:(i)*self.m],label=labl,linewidth=1.5)


        #Plotting the end-point to mark trajectory
        plt.plot(self.coords_traj[0,0:j],self.coords_traj[1,0:j],self.coords_traj[2,0:j],'.', markersize = 3,label='Trajectory')
        plt.legend(loc='upper right')
        self.i_glb = self.i_glb+1




    def plot_animate(self):
        
        ani = animation.FuncAnimation(self.fig,self.update_plot,interval=30)
        plt.show()

        # saving to m4 using ffmpeg writer 
        ani.save(self.fname,writer=self.writer)


if __name__ == "__main__":  # pragma nocover

    print("Testing code for Q-PCC parametrization plotting")

    tds=200
    m=100
    d=1.000 #mm
    theta1 = np.linspace(np.pi/4,np.pi/4,num=tds)
    phi1 = np.linspace(0,np.pi-0.01,num=tds)
    L2 = np.linspace(400,400,num=tds)
    
    delx1p = np.linspace(0,1,num=int(0.5*tds))#theta1*np.cos(phi1)*d
    dely1p = np.sqrt(1-delx1p**2)#theta1*np.sin(phi1)*d
    delx1n = np.linspace(0,1,num=int(0.5*tds))#theta1*np.cos(phi1)*d
    dely1n = np.sqrt(1-delx1n**2)#
    delx1 = np.hstack((delx1p,delx1n))
    dely1 = np.hstack((dely1n,dely1p))

    theta2 = np.linspace(np.pi/4,np.pi/4,num=tds)
    phi2 = np.linspace(0,np.pi-0.01,num=tds)
    L1 = 200*np.ones(tds)
    
    delx2p = np.linspace(-1,0,num=int(0.5*tds))#theta1*np.cos(phi1)*d
    dely2p =  np.sqrt(1-delx2p**2)#theta1*np.sin(phi1)*d
    delx2n = np.linspace(-1,0,num=int(0.5*tds))#theta1*np.cos(phi1)*d
    dely2n = -np.sqrt(1-delx2n**2)#
    delx2 = np.hstack((delx2p,delx2n))
    dely2 = np.hstack((dely2n,dely2p))

    Qpcc_set = np.vstack((delx1,dely1,L1,delx2,dely2,L2))

    print("Shape of PCC input test array :",np.shape(Qpcc_set[0,:]))

    
    pltQpcc = plt_QPcc(Qpcc_set,d,m,'self_test')
    pltQpcc.plot_animate()
    

        
#The next task is to
'''
1. Jax-ify this
2. Use this to simulate Q-PCC parametrization for IK
3. Make the IK+ simulation interactive by giving user inputs  
'''