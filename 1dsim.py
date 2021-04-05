# -*- coding: utf-8 -*-
"""
Created on Thu Oct 31 14:42:36 2019

@author: Harshit
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

gravity=9.8
mass=0.15
u=10
def model1(z,t,u):
   
    x1=z[0]
    x2=z[1]
    dx1dt=x2
    dx2dt=(u-gravity)/mass
    z_val=[dx1dt, dx2dt]
    return z_val

y0=[0,0]

time=np.linspace(0,10,50)
    
zpos=odeint(model1,y0,time,args=(u,))


x1=zpos[1][0]
x2=zpos[1][1]
#y0=zpos[1]
#print(x1,x2)
plt.plot(time,zpos[:,0],'g:', label='position')
plt.plot(time,zpos[:,1],'b:', label='velocity')
plt.xlabel=('time')
plt.ylabel=('response')
plt.legend()
plt.show()