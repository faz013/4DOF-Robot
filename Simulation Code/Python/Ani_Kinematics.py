# -*- coding: utf-8 -*-
"""
Created on Sun Jul 30 15:29:02 2023

@author: Fazil
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Inverse kinematics

l1 = 19
l2 = 18
l3 = 18
l4 = 8

theta = np.pi/2
cx = 15
cy = 15

radius = 5

px = cx-radius*np.cos(theta)
py = cy-radius*np.sin(theta)
pz = 15
phi = 0

q1 = np.arctan2(py,px)


eq1 = px/np.cos(q1) - l4*np.cos(phi)

eq2 = pz-l1-l4*np.sin(phi)

n1 = (eq1*eq1 + eq2*eq2 - l2*l2 - l3*l3)/(2*l2*l3)

n2 = -np.sqrt(1-n1*n1)

q3 = np.arctan2(n2,n1)

k1 = l2+l3*np.cos(q3)
k2 = l2*np.sin(q3)

q2 = np.arctan2(eq2,eq1) - np.arctan2(k2,k1)

q4 = phi-q2-q3


#forward kinematics

#define cos and sin 
c1 = np.cos(q1)
c2 = np.cos(q2)
c3 = np.cos(q3)
c4 = np.cos(q4)

c12 = np.cos(q1+q2)
c23 = np.cos(q2+q3)
c234 = np.cos(q2+q3+q4)


s1 = np.sin(q1)
s2 = np.sin(q2)
s3 = np.sin(q3)
s4 = np.sin(q4)

s12 = np.sin(q1+q2)
s23 = np.sin(q2+q3)
s234 = np.sin(q2+q3+q4) 

#Equations

#base
x1 = 0
y1 = 0
z1 = 0

#first link
x2=0
y2=0
z2 = l1

#second
x3 = l2*c1*c2
y3 = l2*c2*s1
z3 = l1+l2*s2

#third
x4 = c1*(l3*c23+l2*c2)
y4 = s1*(l3*c23+l2*c2)
z4 = l1+l3*s23+l2*s2

#end effector
x5 = c1*(l2*c2+l3*c23+l4*c234)
y5 = s1*(l2*c2+l3*c23+l4*c234)
z5 = l1 + l2*s2 + l3*s23 + l4*s234




fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the first link
ax.plot([x1, x2, x3, x4, x5], [y1, y2, y3, y4, y5], [z1, z2, z3, z4, z5], 'bo-')

# Set axis labels
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')

# Set plot limits
ax.set_xlim([min(x1, x2, x3, x4, x5) - 5, max(x1, x2, x3, x4, x5) + 5])
ax.set_ylim([min(y1, y2, y3, y4, y5) - 5, max(y1, y2, y3, y4, y5) + 5])
ax.set_zlim([min(z1, z2, z3, z4, z5) - 5, max(z1, z2, z3, z4, z5) + 5])

# Show the plot
plt.show()


