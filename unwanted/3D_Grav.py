# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt

# mass, G constant, initial position and velocity
straight_down_descent = False

m = 1
M = 6.42 * (10 ** 23)
r = np.array([10**6,0.0,0.0])
v = np.array([0.0,0.0,0.0])
G = 6.6743 * (10**-11)

if not straight_down_descent: # do circular orbit
    m = 1                     # F = 4285N at this distance
    M = 6.42 * (10 ** 23)     # Therefore V initial = 20700m/s
    r = np.array([10**5,0.0,0.0])
    v = np.array([0.0,20700,0.0])
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
pos_array = np.array(r_list)
v_array = np.array(v_list)
dim_array = ["x", "y", "z"]

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
for i in range(3):
    cur_dim_pos_array = []
    cur_dim_vel_array = []
    for n in range(len(pos_array)):
        cur_dim_pos_array.append(pos_array[n][i])
        cur_dim_vel_array.append(v_array[n][i])
    plt.plot(t_array, cur_dim_pos_array, label=f'{dim_array[i]} (m)')
    plt.plot(t_array, cur_dim_vel_array, label=f'{dim_array[i]} (m/s)')
plt.legend()
plt.show()
