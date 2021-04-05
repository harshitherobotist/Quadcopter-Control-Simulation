# -*- coding: utf-8 -*-
"""
Created on Tue Nov  5 15:22:48 2019

@author: Harshit
"""

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

animate=1 # True / False


kp=1.5 #1.5
kd=0.9  #0.9
kpy=25.1  #25.1
kdy=1.9  #1.9
kdPhi=10.0  #10.0
kpPhi=30.5  #30.5
PhiCDd=0  #0 
yDesDd=0  #0
gravity=9.81
Ixx=0.0015  #0.0015
mass=0.18  #0.18
c=0
k=0

def controller(z, zd):
    
    errorPos[i]=zd[0]-z[0]

    position=zd[0]-z[0]
    velocity=zd[1]-z[1]

    u=mass*(gravity+kp*position+kd*velocity)
    return u

def y_controller(y,yDes):
    
    error_y=yDes[0]-y[0]
    error_yDot=yDes[1]-y[1]
    
    phiC=-(yDesDd+kdy*error_yDot+kpy*error_y)/gravity
    return phiC


y0=[0,0]
yDes=[1,0]
phi0=[0,0]
z0=[0,0]
zDes=[1,0]
phiC0=[0,0]

#phiC=y_controller(y0,yDes)
def phiC_dot_controller(y,yDes):
    vel_error=yDes[1]-y[1]
    phiCDot=-(kpy*vel_error)/gravity
    return phiCDot
    
def rollController(phi,phiC,phiC_dot):
    
    
    error_pos=phiC-phi[0]
    
    error_vel=phiC_dot-phi[1]
    
    u2=Ixx*(PhiCDd+error_pos*kpPhi+error_vel*kdPhi)
    
    return u2

def model(z,t,u):
    x1=z[0]     # Intial Condition for First FODE
    x2=z[1]     # Initial condition for Second FODE
    dx1dt=x2                            # First FODE
    dx2dt=(u-c*x2-k*x1)/mass-gravity    # Second FODE
    dxdt = [dx1dt,dx2dt]    #zdot and zDoubleDot
    return dxdt

def modely(y,t,phi):
    x3=y[0]
    x4=y[1]
    dx3dt=x4
    dx4dt=-gravity*phi
    y_val=[dx3dt,dx4dt]
    return y_val

def modelPhi(phi,t,u2):
    x5=phi[0]
    x6=phi[1]
    dx5dt=x6
    dx6dt=u2/Ixx
    phi_val=[dx5dt,dx6dt]
    return phi_val

finalTime=10
timeSteps=5*finalTime+1

# time points
time=np.linspace(0,finalTime,timeSteps)

# store solution
x1=np.empty_like(time)   #position in z direction
x2=np.empty_like(time)   # velocity in z direction
x3=np.empty_like(time)
x4=np.empty_like(time)
x5=np.empty_like(time)
x6=np.empty_like(time)

phiC=np.empty_like(time)
phiC_dot=np.empty_like(time)

errorPos=np.empty_like(time)
u1=np.empty_like(time)
u2=np.empty_like(time)

x1[0]=z0[0]   # x1 or z
x2[0]=z0[1]   # x2 or zdot
x3[0]=y0[0]
x4[0]=y0[1]
x5[0]=phi0[0]
x6[0]=phi0[1]
phiC[0]=phiC0[0]
phiC_dot[0]=phiC0[1]


errorPos[0]=zDes[0]
u1[0]=0
u2[0]=0

plt.figure(1,figsize=(5,4))
if animate:
    plt.ion()
    plt.show()

# solve ODE
for i in range(1,timeSteps):
    # span for next time step
    timeSpan=[time[i-1],time[i]]

    u1[i]=controller(z0, zDes)  #giving vectors ofz,zdot and zdesired and zotDesired into controller func
    phiC[i]=y_controller(y0,yDes)
    phiC_dot[i]=phiC_dot_controller(y0,yDes)
    u2[i]=rollController(phi0,phiC[i],phiC_dot[i])
    # solve for next step
    z=odeint(model,z0,timeSpan,args=(u1[i],))   #solving the differential equation using new u every time
    #the initail conditions given are initail pos and vel  and the differential equations are solved by 
    #putting in value of zdot and zdoubleDot value through model
    # store solution for plotting
    x1[i]=z[1][0]
    x2[i]=z[1][1]

    # next initial condition stored as answer of the above ode
    
    
    phi=odeint(modelPhi,phi0,timeSpan,args=(u2[i],))
    
    x5[i]=phi[1][0]
    x6[i]=phi[1][1]
    
    
    
    y=odeint(modely,y0,timeSpan,args=(x5[i],))
    
    x3[i]=y[1][0]
    x4[i]=y[1][1]
    
    z0=z[1]
    y0=y[1]
    phi0=phi[1]
    
    

    #print(np.round(z0[0],2),zDes[0],np.round(errorPos[i],2))
    if animate:
        plt.clf()
        plt.subplot(3,1,1)
        plt.ylim((0,x1[i]+1))
        plt.xlim((-2,2))
        plt.text(0,x1[i], str("-O-"))
        plt.text(-2,2.5, str("Quadcopter Simulation"))
        plt.plot(time[0],x1[i])
        plt.ylabel('Position')
        plt.xlabel('Ground Level')
        plt.subplot(3,1,2)
        plt.plot(time[0:i],u1[0:i],'g:',label='u(t)')
        plt.plot(time[0:i],errorPos[0:i],'r:',label='Error(t)')
        plt.ylabel('input')
        plt.xlabel('time')
        plt.subplot(3,1,3)
    
        plt.text(x3[i],x1[i],  str('--O--'))
        plt.plot(x3[0:i],x1[0:i],'b:',label='Position Coordinates')
        plt.xlabel('y Position')
        plt.ylabel('z Position')
        
      #  plt.legend(loc='best')
        plt.pause(0.1)    





    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

