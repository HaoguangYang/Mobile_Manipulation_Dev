#!/usr/bin/env python 

import matplotlib
matplotlib.use('tkagg')
import matplotlib.pyplot as plt
import numpy as np
import redis

r = redis.StrictRedis(host='localhost', port=6379, db=0)
pos_vector = [] 
vel_vector = []
pos_des_vector = []

try:
    while(True):
        for key in r.scan_iter("*"): 
            value = r.get(key)
            if (key == "odometry from base"): 
                pos_vector.append(value)
            if (key == "operational space velocities from base"): 
                vel_vector.append(value)
            if (key == "MMP goal location"): 
            	pos_des_vector.append(value)   
except KeyboardInterrupt:
    pass

# In[472]:


x = []
y = []
z = []

for i in range(len(pos_vector)): 
    x_vector_string = pos_vector[i][1:-1].split(",")
    x_vector_float = [float(j) for j in x_vector_string]
    x.append(x_vector_float[0])
    y.append(x_vector_float[1])
    z.append(x_vector_float[2])
    


# In[473]:


dx = []
dy = []
dz = []

for i in range(len(vel_vector)): 
    xd_vector_string = vel_vector[i][1:-1].split(",")
    xd_vector_float = [float(j) for j in xd_vector_string]
    dx.append(xd_vector_float[0])
    dy.append(xd_vector_float[1])
    dz.append(xd_vector_float[2])


# In[474]:


x_des = []
y_des = []
z_des = []

for i in range(len(pos_des_vector)): 
    x_vector_string = des_position_vector[i][1:-1].split(",")
    x_vector_float = [float(j) for j in x_vector_string]
    x_des.append(x_vector_float[0])
    y_des.append(x_vector_float[1])
    z_des.append(x_vector_float[2])

t_final = 0.001*min(len(x), len(x_des), len(dx))
t = np.arange(0,t_final,0.001)

ic = 0
ic = min(xrange(len(x)), key=x.__getitem__) 

ec = min(len(x), len(x_des), len(dx))

fig, axs = plt.subplots(3)
plt.rcParams.update({'font.size': 22})
fig.set_size_inches(18.5, 20)
# fig.suptitle('Operational Point Trajectory', fontsize=20)

lw = 4.0
# plt.title('Operational Point Trajectory', fontsize=20)
axs[0].plot(x, 'r', linewidth = lw, label='x')
axs[0].plot(x_des, 'k--', linewidth = lw, label='x desired')
axs[1].plot(y, 'b', linewidth = lw, label='y')
axs[1].plot(y_des, 'k--', linewidth = lw, label='y desired')
axs[2].plot(z, 'g', linewidth = lw, label='z')
axs[2].plot(z_des, 'k--', linewidth = lw, label='z desired')

plt.xlabel('Time (s)', fontsize=18)
# plt.ylabel('Position (m)', fontsize=18)

#axs[0].legend(loc='best')
axs[0].set_title('Vehicle Trajectory')
axs[0].set(ylabel = 'Position (m)')
axs[1].set(ylabel = 'Position (m)')
axs[2].set(ylabel = 'Heading (rad)')

axs[0].legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
axs[1].legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
axs[2].legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)

plt.show()

fig, axs = plt.subplots(3)
plt.rcParams.update({'font.size': 22})
# fig.suptitle('Operational Point Trajectory', fontsize=20)

lw = 4.0
axs[0].plot( dx, 'r:', linewidth = lw, label='Joint 4: Top limit')
axs[1].plot( dy, 'b:', linewidth = lw, label='Joint 4: Top limit')
axs[2].plot( dz, 'g:', linewidth = lw, label='Joint 4: Top limit')



plt.xlabel('Time (s)', fontsize=18)

axs[0].set_title('Vx (m/s)')
axs[1].set_title('Vy (m/s)')
axs[2].set_title('Vz (m/s)')
axs[0].set(ylabel = 'Velocity (m/s)')
axs[1].set(ylabel = 'Velocity (m/s)')
axs[2].set(ylabel = 'Velocity (m/s)')

axs[0].legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
axs[1].legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)


plt.show()
