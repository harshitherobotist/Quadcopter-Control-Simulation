# -*- coding: utf-8 -*-
"""
Created on Sun Oct 27 10:55:43 2019

@author: Harshit
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
#from scipy.integrate import dblquad
#from scipy.integrate import quad
animate=1

gravity=9.81
mass=0.18
kp=1.5
kd=0.9
Ixx=1.0
kpy=2.5
kdy=0.9
kpPhi=1.5
kdPhi=0.9

def controllerz(z, zd):

    # PD controller for the height
    # z: 1x2 vector containing the current state [z; zDot]
    # zd:1x2 vector containing desired state [zd; zdDot]

    # Error and Input
    errorPos[i]=zd[0]-z[0]

    position=zd[0]-z[0]
    velocity=zd[1]-z[1]

    

    u=mass*(gravity+kp*position+kd*velocity)
    return u


def controllerPhi(phiO, phiD,phiDdCommanded,phidCommanded,phiDd):
    
    
    errorPhi[i] = phiD[0]-phiO[0]  #phi Desired minus Phi Obtained
    
    angle_phi=phiDd-phiO[0]
    angulerVelocity=phidCommanded-phiO[1]
    
    
    ##phiDdCommanded=(0+kdPhi*angulerVelocity + kpPhi*angle_phi)
    
    
    u2=Ixx*(kdPhi*angulerVelocity + kpPhi*angle_phi) #as commanded acc is zero so 0 added
    return u2
    
def PhiValue(yOb,yDes):
    
    errorPosy[i]=yDes[0]-yOb[0]
    pos_y=yDes[0]-yOb[0]
    Vely=yDes[1]-yOb[1]
    
    CurrentPhi=-(kdy*(-yDes[1])+kpy*-(pos_y))/gravity
    return CurrentPhi
    
# function that returns dz/dt
def model(z,t,u):
    x1=z[0]     # Intial Condition for First FODE
    x2=z[1]     # Initial condition for Second FODE
    dx1dt=x2                            # First FODE
    dx2dt=(u)/mass-gravity    # Second FODE
    dxdt = [dx1dt,dx2dt]      #vector containing zDoubleDot and zDot values
    return dxdt

def model2(phi,t,u2):
    x5=phi[0]
    x6=phi[1]
    dx5dt=x5
    dx6dt=u2/Ixx
    dphi2dt=[dx5dt,dx6dt]   # vector containing phiDoubleDot and phiDot values
    return dphi2dt

def model1(y,t,phi):
    x3=y[0]   #inital condition for first FODE and initial y value
    x4=y[1]   #initial condition for seconf FODE and initial yDot value
    dx3dt=x4
    dx4dt=-gravity*phi
    dydt=[dx3dt,dx4dt]    #vector containing yDoubleDot and yDot values 
    return dydt    
# initial condition
z0=[0,0]
zDes=[1,0]
y0=[0,0]
yDes=[1,0]
phi0=[0,0]
phiDes=[0,0]

# number of time points
finalTime=10
timeSteps=5*finalTime+1

# time points
time=np.linspace(0,finalTime,timeSteps)
t=finalTime/timeSteps
# store solution
x1=np.empty_like(time)
x2=np.empty_like(time)
x3=np.empty_like(time)
x4=np.empty_like(time)
x5=np.empty_like(time)
x6=np.empty_like(time)

errorPos=np.empty_like(time)
errorPosy=np.empty_like(time)
errorPhi=np.empty_like(time)
u1=np.empty_like(time)
uphi=np.empty_like(time)
phiDdCommanded=np.empty_like(time)
phiDd=np.empty_like(time)
phidCommanded=np.empty_like(time)

# record initial conditions
x1[0]=z0[0]   # x1 or z
x2[0]=z0[1]   # x2 or zdot
x3[0]=y0[0]   # x3 or
x4[0]=y0[1]
x5[0]=phi0[0]
x6[0]=phi0[1]

errorPos[0]=zDes[0]
errorPosy[0]=yDes[0]
errorPhi[0]=phiDes[0]
u1[0]=0
uphi[0]=0
phiDdCommanded[0]=0

plt.figure(1,figsize=(5,4))
if animate:
    plt.ion()
    plt.show()

# solve ODE
for i in range(1,timeSteps):
    # span for next time step
    timeSpan=[time[i-1],time[i]]

    u1[i]=controllerz(z0, zDes)
    
    phiDd[i]=PhiValue(y0,yDes)
    phidCommanded[i]=(phiDd[i]-phiDd[i-1])/t
    phiDdCommanded[i]=(phidCommanded[i]-phidCommanded[i-1])/t
    
    #phiDd[i-1]=phiDd[i]
   # phidCommanded[i-1]=phidCommanded[i]
   # phiDdCommanded[i-1]=phiDdCommanded[i]
                  
    uphi[i]=controllerPhi(phi0, phiDes,phiDdCommanded[i],phiDdCommanded[i],phiDd[i])
    
    phi=odeint(model2,phi0,timeSpan,args=(uphi[i],))
    
    x5[i]=phi[1][0]  #phi value
    x6[i]=phi[1][1]  #phiDot value
    phi0=phi[1]
    
    
    y=odeint(model1,y0,timeSpan,args=(x5[i],))
    
    x3[i]=y[1][0]
    x4[i]=y[1][1]
    
    y0=y[1]
    

    # solve for next step
    z=odeint(model,z0,timeSpan,args=(u1[i],))

    # store solution for plotting
    x1[i]=z[1][0]
    x2[i]=z[1][1]

    # next initial condition stored as answer of the above ode
    z0=z[1]
    

    #print(np.round(z0[0],2),zDes[0],np.round(errorPos[i],2))
    if animate:
        plt.clf()
        plt.subplot(4,1,1)
        plt.ylim((0,x1[i]+1))
        plt.xlim((-2,2))
        plt.text(0,x1[i], str("-O-"))
        plt.text(-2,2.5, str("Quadcopter Simulation"))
        plt.plot(time[0],x1[i])
        plt.ylabel('Z Position')
        plt.xlabel('Ground Level')
        plt.subplot(4,1,2)
        plt.plot(time[0:i],u1[0:i],'g:',label='u1(t)')
        plt.plot(time[0:i],errorPos[0:i],'r:',label='ErrorZ(t)')
        plt.ylabel('inputs')
        plt.xlabel('time')
        plt.legend()
        plt.subplot(4,1,3)
        plt.plot(time[0:i],uphi[0:i],'blue',label='u2(t)')
        plt.plot(time[0],x3[i],'black',label='Y(t)')
        plt.ylabel('inputs')
        plt.xlabel('time')
        plt.legend()
        plt.subplot(4,1,4)
        plt.plot(time[0],x5[i],'y',label='Phi(t)')
        plt.legend()
        plt.pause(0.1)    

if not animate:
    # plot results
    plt.subplot(3,1,1)
    plt.plot(time,u1,'g:',label='u(t)')
    plt.ylabel('input')
    plt.xlabel('time')
    plt.legend(loc='best')

    plt.subplot(3,1,2)
    plt.plot(time,errorPos,'g:',label='u(t)')
    #plt.plot(time,u,'g:',label='u(t)')
    plt.ylabel('error')
    plt.xlabel('time')
    plt.legend(loc='best')

    plt.subplot(3,1,3)
    plt.plot(time,x1,'b-',label='x1(t) or z')
    plt.plot(time,x2,'r--',label='x2(t) or zdot')
    plt.ylabel('response')
    plt.xlabel('time')
    plt.legend(loc='best')
    plt.show()
