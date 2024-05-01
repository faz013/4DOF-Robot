# -*- coding: utf-8 -*-
"""
Created on Sat Apr 29 14:30:11 2023

@author: Fazil
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Define system parameters
m1 = 0.20
m2 = 0.20
l1 = 0.18
l2 = 0.18
g = -9.81



# Define desired trajectory
q_des = [140*np.pi/180, 50*np.pi/180] #[np.pi/2, np.pi/2]
qdot_des = [0*np.pi/180, 0*np.pi/180]
qddot_des = [0*np.pi/180, 0*np.pi/180]


# Define initial conditions
q = [90*np.pi/180, 0*np.pi/180]#[0, 0]
qdot = [0*np.pi/180, 0*np.pi/180]


# Define control gains
Kp = np.diag([32.5, 32.5])
Kd = np.diag([11.2, 11.2])


# Define time vector
time_period = 5
step_size = 0.001
steps = int(time_period/step_size)
t = np.linspace(0, time_period, steps)


us = 0

# Feedback linearization control law
def feedback_linearization(q, qdot, q_des, qdot_des, qddot_des):
    # Compute system dynamics matrices
    
   #mass matrix
    m11 = m1*l1**2 + m2*(l1**2 + l2**2 + 2*l1*l2*np.cos(q[1]))
    m12 = m2*(l2**2 + l1*l2*np.cos(q[1]))
    m21 = m2*(l2**2 + l1*l2*np.cos(q[1]))
    m22 = m2*l2**2
    
    #coreolis
    c11 = -m2*l1*l2*np.sin(q[1])*qdot[1]
    c12 = -m2*l1*l2*np.sin(q[1])*(qdot[0]+qdot[1])
    c21 = m2*l1*l2*np.sin(q[1])*qdot[0]
    c22 = 0
    
    #Gravity
    g1 = (m1+m2)*g*l1*np.cos(q[0]) + m2*g*l2*np.cos(q[0]+q[1])
    g2 = m2*g*l2*np.cos(q[0]+q[1]) 
    
    M = np.array([[m11, m12],
                  [m21, m22]])
    
    C = np.array([[c11,c12],
                 [c21,c22]])

    #C = np.array([[-m2*l1*l2*np.sin(q[2])*qdot[2]^2-2*m2*l1*l2*np.sin(q[2])*qdot[1]*qdot[2]],
                  #[m2*l1*l2*np.sin(q[2])*qdot[1]^2]])
    
    G = np.array([[g1],
                  [g2]])

    # Compute control input
    
    e = np.array([q_des]).T - np.array([q]).T
    edot = np.array([qdot_des]).T - np.array([qdot]).T
    #u = us*(M.dot(np.array([qddot_des]).T + Kp.dot(e) + Kd.dot(edot))  + G + C.dot(np.array([qdot]).T))
    u = Kp.dot(e) + Kd.dot(edot)
    # u = 0
    if u[0] > 3.8:
        u[0] = 3.8
    elif u[0] < -3.8:
        u[0] = -3.8

    if u[1] > 3.8:
        u[1] = 3.8
    elif u[1] < -3.8:
          u[1] = -3.8  # compare each element with 3.8 and replace with 3.8 if greater
    
    return u, M, C, G




# Define arrays to store target and actual positions
q_desired = np.zeros((2, len(t)))
q_actual = np.zeros((2, len(t)))
error = np.zeros((2, len(t)))

u1_actual = np.zeros((2, len(t)))
u2_actual = np.zeros((2, len(t)))

# Simulate system and store results
for i in range(len(t)):
# Compute control input
    u, M, C, G = feedback_linearization(q, qdot, q_des, qdot_des, qddot_des)
    if t[i] < 1.5:
        us = 0
    elif t[i] >= 1.5:
        us = 1
   # Update system state using 4th order Runge-Kutta
    k1 = np.concatenate((qdot, np.linalg.inv(M).dot(u - C.dot(np.array([qdot]).T) - G).flatten()), axis=None)
    k2 = np.concatenate((qdot + step_size/2 * k1[2:], np.linalg.inv(M).dot(u - C.dot(np.array([qdot + step_size/2 * k1[2:]]).T) - G).flatten()), axis=None)
    k3 = np.concatenate((qdot + step_size/2 * k2[2:], np.linalg.inv(M).dot(u - C.dot(np.array([qdot + step_size/2 * k2[2:]]).T) - G).flatten()), axis=None)
    k4 = np.concatenate((qdot + step_size * k3[2:], np.linalg.inv(M).dot(u - C.dot(np.array([qdot + step_size * k3[2:]]).T) - G).flatten()), axis=None)
    qdot += step_size/6 * (k1[2:] + 2*k2[2:] + 2*k3[2:] + k4[2:])
    q += step_size/6 * (k1[:2] + 2*k2[:2] + 2*k3[:2] + k4[:2])
    
    # Store target and actual positions
    q_desired[:, i] = q_des
    q_actual[:, i] = q
    u1_actual[:, i] = u[0]
    u2_actual[:, i] = u[1]

    

# Plot results
# plt.figure()
# # plt.plot(t, q_desired[0]*180/np.pi, 'r--', label='Desired')
# plt.plot(t, q_desired[0]*180/np.pi, 'b--', label='Motor 2: Desired')
# plt.plot(t, q_actual[0]*180/np.pi, 'b', label='Motor 2: Actual')
# plt.plot(t, q_desired[1]*180/np.pi, 'r--', label='Motor 3: Desired')
# plt.plot(t, q_actual[1]*180/np.pi, 'r', label='Motor 3: Actual')
# plt.legend()
# plt.xlabel('Time')
# plt.ylabel('Joint angle (rad/2)')
# plt.title('Joint 1 position')

# plt.figure()
# plt.plot(t, q_desired[1]*180/np.pi, 'r--', label='Desired')
# plt.plot(t, q_actual[1]*180/np.pi, 'b', label='Actual')
# plt.legend()
# plt.xlabel('Time')
# plt.ylabel('Joint angle (degrees)')
# plt.title('Joint 2 position')

# plt.figure()
# plt.plot(t, u1_actual[0], 'b', label='Actual')
# plt.legend()
# plt.xlabel('Time')
# plt.ylabel('Joint Torque')
# plt.title('Joint 1 Torque')

# plt.figure()
# plt.plot(t, u2_actual[0], 'b', label='Actual')
# plt.legend()
# plt.xlabel('Time')
# plt.ylabel('Joint Torque')
# plt.title('Joint 2 Torque')

plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.size'] = 14

plt.figure()
# plt.plot(t, q_desired[0]*180/np.pi, 'r--', label='Desired')
plt.grid()
plt.plot(t, (q_desired[0]-q_actual[0])*180/np.pi, 'b--', label='Motor 2')
plt.plot(t, (q_desired[1]-q_actual[1])*180/np.pi, 'r--', label='Motor 3')
# plt.plot(t, u1_actual[0], 'b', label='Torque 1')
# plt.plot(t, u2_actual[0], 'b', label='Torque 2')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Joint Error (Degrees)')



plt.show()
