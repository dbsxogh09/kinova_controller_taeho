import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

data = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/tro_data/application/PD.txt') 
time = data[:,0]
x = data[:,1]
y = data[:,2]
z = data[:,3]
nx = data[:,4]
ny = data[:,5]
nz = data[:,6]
x_d = data[:,7]
y_d = data[:,8]
z_d = data[:,9] ## 13~19
ex = data[:,10]
ey = data[:,11]
ez = data[:,12]
rot_r_pd = data[:,23]
rot_p_pd = data[:,24]
rot_y_pd = data[:,25]

data_L1 = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/tro_data/application/L1.txt') 
time_l1 = data_L1[:,0]
x_l1 = data_L1[:,1]
y_l1 = data_L1[:,2]
z_l1 = data_L1[:,3]
nx_l1 = data_L1[:,4]
ny_l1 = data_L1[:,5]
nz_l1 = data_L1[:,6]
x_d_l1 = data_L1[:,7]
y_d_l1 = data_L1[:,8]
z_d_l1 = data_L1[:,9]
ey_l1 = data_L1[:,10]
ex_l1 = data_L1[:,11]
ez_l1 = data_L1[:,12]
rot_r_l1 = data_L1[:,23]
rot_p_l1 = data_L1[:,24]
rot_y_l1 = data_L1[:,25]

data_x = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/tro_data/application/non.txt') 
time_x = data_x[:,0]
x_x = data_x[:,1]
y_x = data_x[:,2]
z_x = data_x[:,3]
nx_x = data_x[:,4]
ny_x = data_x[:,5]
nz_x = data_x[:,6]
x_d_x = data_x[:,7]
y_d_x = data_x[:,8]
z_d_x = data_x[:,9]
ex_x = data_x[:,10]
ey_x = data_x[:,11]
ez_x = data_x[:,12]
rot_r_x = data_x[:,23]
rot_p_x = data_x[:,24]
rot_y_x = data_x[:,25]

fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')

# 3D Trajectories
ax.plot(x_d_x, y_d_x, z_d_x, 'k', label='reference')
ax.plot(x_x, y_x, z_x, 'm', label='only pd')
ax.plot(x, y, z, 'b', label='conventional')
ax.plot(x_l1, y_l1, z_l1, 'r', label='proposed')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
ax.view_init(elev=30, azim=-60)  # 시점 각도 조절

plt.tight_layout()

plt.figure(2)
plt.subplot(3, 1, 1)
plt.plot(time_x, x_d_x - x_x, 'm', label='only pd')
plt.plot(time, x_d - x, 'b', label='conventional')
plt.plot(time_l1, x_d_l1 - x_l1, 'r', label='proposed')

plt.title('X error')
plt.xlabel('Time (s)')
plt.ylabel('Error (m)')
plt.legend()
plt.subplot(3, 1, 2)
plt.plot(time_x, y_d_x - y_x, 'm', label='only pd')
plt.plot(time, y_d - y, 'b', label='conventional')
plt.plot(time_l1, y_d_l1 - y_l1, 'r', label='proposed')
plt.title('Y error')
plt.xlabel('Time (s)')
plt.ylabel('Error (m)')
plt.subplot(3, 1, 3)
plt.plot(time_x, z_d_x - z_x, 'm', label='only pd')
plt.plot(time, z_d - z, 'b', label='conventional')
plt.plot(time_l1, z_d_l1 - z_l1, 'r', label='proposed')
plt.title('Z error')
plt.xlabel('Time (s)')
plt.ylabel('Error (m)')
plt.tight_layout()


plt.figure(3)
plt.plot(x_x, y_x, 'r', label='only pd')
plt.plot(x, y, 'g', label='conventional')
plt.plot(x_l1, y_l1, 'b', label='proposed')
# plt.plot(nx, ny, 'm', label='nominal')
plt.plot(x_d, y_d, 'k--')
plt.legend()

plt.show()
