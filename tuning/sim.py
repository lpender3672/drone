
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

import sympy as sp

g=9.81  # m/s^2
L=0.125 # distance from pendulum's centre of mass to pivot in m
a=0.016 # radius of pulley in m
m=0.32  # mass of pendulum in kg
M=0.7  # mass of carriage in kg
I=8e-5  # moment of inertia on motor shaft in kg m^2
km = 0.08  # torque motor constant in Nm/A
ka = -0.50 # amplifier constant in A/V

gamma =  M/m + I/(m*np.power(a,2))

#scale factors to get physical units metres radians and seconds.
#in the crane/down position
# [x xdot Ltheta Lthetadot]=[CP CV PP PV]*Sc where,
Sc=np.diag([-1/12.5, -1/2.23, L/3.18, L/0.64])

# in the inverted position
# [x xdot Lphi Lphidot]=[CP CV PP PV]*Sp where,
Sp=np.diag([-1/12.5, -1/2.23, L/3.18, -L/0.64])

#controller amplifier gains on each measurement
opamp_c = np.diag([-20,-30, 20, -10]) # for crane controller
opamp_p = np.diag([10,20,30,-20]) # for inverted pendulum controller

#maximum torque from the motor in Nm
Tmax=0.4

# squares of the natural frequencies
om12=g/L 
om02=om12*(1+1/gamma)

#linearized crane  model

Ac=np.array([[0, 1, 0, 0,], [0, 0, om12-om02, 0], [0, 0, 0, 1], [0, 0, -om02, 0]])
Cc=-(ka*km/(m*a*gamma))*np.linalg.solve(Sc,opamp_c)
B=np.array([[0],[1],[0],[1]])

#linearized inverted pendulum model
Ap=np.array([[0, 1, 0, 0], [0, 0, om02-om12, 0], [0, 0, 0, 1], [0, 0, om02, 0]])
Cp=-(ka*km/(m*a*gamma))*np.linalg.solve(Sp,opamp_p)

# Sampling period 
h=0.0025 #sampling period
h_sim=0.002 #simulation step size

class solver_drone():
    def __init__(self, x0, tsim, tdem, xdem, p):
      self.x0 = x0
      self.tsim = tsim
      self.tdem = tdem
      self.xdem = xdem
      self.p = p

    def next_state(self, x):
        phi = x[2]/L
        sinphi = np.sin(phi)
        cosphi = np.cos(phi)
        newx = np.empty(4) 

        x24dot =np.linalg.solve(np.array([[1+gamma,-cosphi],[cosphi,-1]]),np.array([-sinphi/L*x[3]**2 + T/(m*a)-(F/m)*np.sign(x[1]),-g*sinphi]))    
        newx[[0,2]]=x[[0,2]]+h_sim*x[[1,3]]+0.5*(h_sim**2)*x24dot

        sinphi1 = np.sin(newx[2]/L)
        cosphi1 = np.cos(newx[2]/L)
        x24dot1 = np.linalg.solve(np.array([[1+gamma,-cosphi1],[cosphi1,-1]]),np.array([-sinphi1/L*x[3]**2 + T/(m*a)-(F/m)*np.sign(x[1]),-g*sinphi1]))    
        newx[[1,3]]=x[[1,3]]+h_sim*0.5*(x24dot+x24dot1)

        if((x[1]!=0.0) & (x[1]*newx[1]<=0.0)):
            self.stopped = True

        return newx
   
         
    def run(self, x0):
        x = np.copy(x0)
        x_sim = x.reshape(1,4)
        T_sim = []

        for t in self.tsim:
            xd = np.interp(t, self.tdem, self.xdem)
            T= (ka*km*P@ opamp_p@ np.linalg.solve(Sp,(x-np.array([xd,0,0,0]))))[0]
            if np.abs(T)>Tmax:
                T= Tmax*np.sign(T)
            T_sim.append(T.item())
            x = self.next_state_pendulum(x)
            x_sim = np.vstack((x_sim,np.transpose(x)))
        return x_sim