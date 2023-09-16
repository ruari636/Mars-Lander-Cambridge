# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
M = 6.42 * (10 ** 23)
r = np.array([10**6,0.0,0.0])
v = np.array([0.0,0.0,0.0])
G = 6.6743 * (10**-11)

# simulation time, timestep and time
t_max = 170
dt = 0.1
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
r_list = []
v_list = []

r_list.append(v * dt + r)
v_list.append(v)

# Euler integration
for t in range(len(t_array) - 1):

    # append current state to trajectories
    r_list.append(r)
    v_list.append(v)

    # calculate new position and velocity
    dir = r / np.linalg.norm(r)
    a = (-(M * G) / (r.dot(r))) * dir
    r = 2 * r_list[-1] - r_list[-2] + (dt**2)*(a)
    v = (1 / (2 * dt)) * (r - r_list[-2])

# convert trajectory lists into arrays, so they can be sliced (useful for Assignment 2)
x_array = np.array(r_list)
v_array = np.array(v_list)

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x_array, label='x (m)')
plt.plot(t_array, v_array, label='v (m/s)')
plt.legend()
plt.show()
