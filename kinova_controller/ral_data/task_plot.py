import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt('/home/nuc/seowook/ros2_ws/src/kinova_controller/ral_data/PD.txt') 
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
ey_dn = data[:,10]
ex_dn = data[:,11]
ez_dn = data[:,12]
rot_e_pd = data[:,23]

<<<<<<< HEAD
data_L1 = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/ral_data/L1.txt') 
time_l1 = data_L1[:,0]
=======
data_L1 = np.loadtxt('/home/nuc/seowook/ros2_ws/src/kinova_controller/ral_data/L1.txt') 
time = data_L1[:,0]
>>>>>>> b76de30e5faddca47b141879b18a237dc88b4537
x_l1 = data_L1[:,1]
y_l1 = data_L1[:,2]
z_l1 = data_L1[:,3]
nx_l1 = data_L1[:,4]
ny_l1 = data_L1[:,5]
nz_l1 = data_L1[:,6]
x_d_l1 = data_L1[:,7]
y_d_l1 = data_L1[:,8]
z_d_l1 = data_L1[:,9]
ey_dn_l1 = data_L1[:,10]
ex_dn_l1 = data_L1[:,11]
ez_dn_l1 = data_L1[:,12]
rot_e_l1 = data_L1[:,23]

data_x = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/ral_data/non.txt') 
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
rot_e_x = data_x[:,23]

plt.figure(1)
plt.subplot(3,1,1)
plt.plot(time_l1, (x_l1 - x_d_l1)*100, 'r--')
plt.plot(time, (x - x_d)*100, 'b-.')
plt.plot(time_x, (x_x - x_d_x)*100, 'm:')
plt.axhline(0.0, 0, 8, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0, 6])

plt.subplot(3,1,2)
plt.plot(time_l1, (y_l1 - y_d_l1)*100, 'r--')
plt.plot(time, (y - y_d)*100, 'b-.')
plt.plot(time_x, (y_x - y_d_x)*100, 'm:')
plt.axhline(0.0, 0, 8, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0, 6])

plt.subplot(3,1,3)
plt.plot(time_l1, (z_l1 - z_d_l1)*100, 'r--')
plt.plot(time, (z - z_d)*100, 'b-.')
plt.plot(time_x, (z_x - z_d_x)*100, 'm:')
plt.axhline(0.0, 0, 8, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0, 6])

plt.figure(2)
plt.plot(y_d_l1, z_d_l1, 'k')
plt.plot(y_l1, z_l1, 'r--')
plt.plot(y - (0.0246306 - 0.0245691), z + (0.519803 - 0.519876), 'b-.')
plt.plot(y_x - (0.0246306 - 0.024776), z_x + (0.519803 - 0.518963), 'm:')
plt.axis('equal')

plt.figure(3)
plt.subplot(3,1,1)
plt.plot(time_l1, ey_dn_l1, 'r')
plt.plot(time, ey_dn, 'b--')

plt.subplot(3,1,2)
plt.plot(time_l1, ex_dn_l1, 'r')
plt.plot(time, ex_dn, 'b--')

plt.subplot(3,1,3)
plt.plot(time_l1, ez_dn_l1, 'r')
plt.plot(time, ez_dn, 'b--')
plt.show()

