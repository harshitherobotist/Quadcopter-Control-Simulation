# -*- coding: utf-8 -*-
"""
Created on Fri Oct 25 14:13:29 2019

@author: Harshit
"""

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# animate plots?
animate=1 # True / False
c=0
k=0
gravity=9.81
mass=0.18
kp=1.5    #12.5,  1.5,    10.5
kd=0.9     #2.9,   0.9,    1.0
SettlingTime=0
maxovershoot=0
def controller(z, zd):

    # PD controller for the height
    # z: 1x2 vector containing the current state [z; zDot]
    # zd:1x2 vector containing desired state [zd; zdDot]

    # Error and Input
    errorPos[i]=zd[0]-z[0]

    position=zd[0]-z[0]
    velocity=zd[1]-z[1]

    

    u=mass*(gravity+kp*position+kd*velocity)
    return u

# function that returns dz/dt
def model(z,t,u):
    x1=z[0]     # Intial Condition for First FODE
    x2=z[1]     # Initial condition for Second FODE
    dx1dt=x2                            # First FODE
    dx2dt=(u-c*x2-k*x1)/mass-gravity    # Second FODE
    dxdt = [dx1dt,dx2dt]    #zdot and zDoubleDot
    return dxdt

# initial condition
z0=[0,0]
zDes=[1,0]   #controller defined to reach to this position
# number of time points
finalTime=10
timeSteps=5*finalTime+1

# time points
time=np.linspace(0,finalTime,timeSteps)

# store solution
x1=np.empty_like(time)   #position in z direction
x2=np.empty_like(time)   # velocity in z direction
errorPos=np.empty_like(time)
u1=np.empty_like(time)
SettlingTime=np.empty_like(time)
# record initial conditions
x1[0]=z0[0]   # x1 or z
x2[0]=z0[1]   # x2 or zdot
errorPos[0]=zDes[0]
u1[0]=0


plt.figure(1,figsize=(5,4))
if animate:
    plt.ion()
    plt.show()
# solve ODE
for i in range(1,timeSteps):
    # span for next time step
    
    timeSpan=[time[i-1],time[i]]

    u1[i]=controller(z0, zDes)  #giving vectors ofz,zdot and zdesired and zotDesired into controller func

    # solve for next step
    z=odeint(model,z0,timeSpan,args=(u1[i],))   #solving the differential equation using new u every time
    #the initail conditions given are initail pos and vel  and the differential equations are solved by 
    #putting in value of zdot and zdoubleDot value through model
    # store solution for plotting
    x1[i]=z[1][0]
    x2[i]=z[1][1]
    
        
       # print(maxOvershoot)
    
    
    
    if(x1[i]>=(0.95*zDes[0]) and x1[i]<=(1.05*zDes[0])):
        SettlingTime[i]=time[i]
    if(x1[i]<=(0.95*zDes[0]) or x1[i]>=(1.05*zDes[0])):
        SettlingTime[i]=0
    #print(SettlingTime[i])
   # print(x1[i])

    # next initial condition stored as answer of the above ode
    z0=z[1]
    
    
        
    #print(np.round(z0[0],2),zDes[0],np.round(errorPos[i],2))
    if animate:
        plt.clf()  #clear screen
        plt.subplot(2,1,1)  
        plt.ylim((0,x1[i]+1))
        plt.xlim((-2,2))
        plt.text(0,x1[i], str("-O-"))
        plt.text(0,maxOvershoot, str('MaxOvershoot'))
        plt.text(-2,2.5, str("Quadcopter Simulation"))
        plt.plot(time[0],x1[i])
        plt.ylabel('Position')
        plt.xlabel('Ground Level')
        plt.subplot(2,1,2)
        plt.plot(time[0:i],u1[0:i],'g:',label='u(t)')
        plt.plot(time[0:i],errorPos[0:i],'r:',label='Error(t)')
        plt.ylabel('input')
        plt.xlabel('time')
        plt.legend(loc='best')
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
max x1[i]=maxOvershoot
print(maxOvershoot)