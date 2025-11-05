import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/tro_data/collision_tracking/PD.txt') 
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

data_L1 = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/tro_data/collision_tracking/L1.txt') 
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

data_x = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/tro_data/collision_tracking/non.txt') 
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

plt.figure(1)
# plt.subplot(3,1,1)
# plt.plot(time, x_d, 'g')
# # plt.plot(time_x, x_x, 'm')
# plt.plot(time, x, 'b')
# plt.plot(time_l1, x_l1, 'r')
# # plt.axhline(0.0, 0, 8, color='lightgray', linestyle='--', linewidth=2)
# plt.xlim([0, 15])

# plt.subplot(3,1,2)
plt.plot(time, y_d, 'g')
plt.plot(time, y, 'b')
plt.plot(time_l1, y_l1, 'r')
# plt.axhline(0.0, 0, 8, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0, 15])

# plt.subplot(3,1,3)
# plt.plot(time, z_d, 'g')
# # plt.plot(time_x, z_x, 'm')
# plt.plot(time, z, 'b')
# plt.plot(time_l1, z_l1, 'r')
# # plt.axhline(0.0, 0, 8, color='lightgray', linestyle='--', linewidth=2)
# plt.xlim([0, 15])

plt.show()
