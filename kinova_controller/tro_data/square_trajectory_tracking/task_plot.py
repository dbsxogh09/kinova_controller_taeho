import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/tro_data/square_trajectory_tracking/PD2.txt') 
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

data_L1 = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/tro_data/square_trajectory_tracking/L12.txt') 
data_L2 = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/tro_data/square_trajectory_tracking/PD4.txt') 
time_l1 = data_L1[:,0]
x_l1 = data_L1[:,1]
y_l1 = data_L1[:,2]
# z_l1 = data_L1[:,3]
z_l1 = data_L2[:,3]
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

data_x = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/tro_data/square_trajectory_tracking/non.txt') 
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
plt.subplot(2,1,1)
plt.plot(time, ny - y_d, 'g')
plt.plot(time_x, y_x - y_d_x, 'm')
plt.plot(time, y - y_d, 'b')
plt.plot(time_l1, y_l1 - y_d_l1, 'r')
plt.axhline(0.0, 0, 8, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0, 25])

plt.subplot(2,1,2)
plt.plot(time, nz - z_d, 'g')
plt.plot(time_x, z_x - z_d_x, 'm')
plt.plot(time, z - z_d, 'b')
plt.plot(time_l1, z_l1 - z_d_l1, 'r')
plt.axhline(0.0, 0, 8, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0, 25])

plt.figure(2)
plt.plot(y_d_l1, z_d_l1, 'k')
plt.plot(ny, nz, 'g')
plt.plot(y_x, z_x, 'm:')
plt.plot(y, z, 'b-.')
plt.plot(y_l1, z_l1, 'r--')
plt.legend(["reference", "nominal", "only pd", "conventional", "proposed"])
plt.axis('equal')   


plt.figure(3)
plt.plot(x_d_l1, z_d_l1, 'k')
plt.plot(nx, nz, 'g')
plt.plot(x_x, z_x, 'm:')
plt.plot(x, z, 'b-.')
plt.plot(x_l1, z_l1, 'r--')
plt.legend(["reference", "nominal", "only pd", "conventional", "proposed"])
plt.axis('equal')   

plt.show()


def rmse(y_true, y_pred):
    return np.sqrt(np.mean((y_true - y_pred) ** 2))

# y 좌표 RMSE
rmse_y_x = rmse(y_x[:20000], y_d_x[:20000])
rmse_y_conv = rmse(y[:20000], y_d[:20000])
rmse_y_prop = rmse(y_l1[:20000], y_d_l1[:20000])

# z 좌표 RMSE
rmse_z_x = rmse(z_x[:20000], z_d_x[:20000])
rmse_z_conv = rmse(z[:20000], z_d[:20000])
rmse_z_prop = rmse(z_l1[:20000], z_d_l1[:20000])

# 결과 출력
print(f"RMSE (y-axis): Non={rmse_y_x:.5f}, Conventional={rmse_y_conv:.5f}, Proposed={rmse_y_prop:.5f}")
print(f"RMSE (z-axis): Non={rmse_z_x:.5f}, Conventional={rmse_z_conv:.5f}, Proposed={rmse_z_prop:.5f}")

# position vector RMSE 계산 함수
def rmse_3d(pos, ref):
    return np.sqrt(np.mean(np.sum((pos - ref) ** 2, axis=1)))

# Non
pos_x = np.stack([x_x[:20000], y_x[:20000], z_x[:20000]], axis=1)
ref_x = np.stack([x_d_x[:20000], y_d_x[:20000], z_d_x[:20000]], axis=1)
rmse_xyz_x = rmse_3d(pos_x, ref_x)

# Conventional
pos_conv = np.stack([x[:20000], y[:20000], z[:20000]], axis=1)
ref_conv = np.stack([x_d[:20000], y_d[:20000], z_d[:20000]], axis=1)
rmse_xyz_conv = rmse_3d(pos_conv, ref_conv)

# Proposed
pos_prop = np.stack([x_l1[:20000], y_l1[:20000], z_l1[:20000]], axis=1)
ref_prop = np.stack([x_d_l1[:20000], y_d_l1[:20000], z_d_l1[:20000]], axis=1)
rmse_xyz_prop = rmse_3d(pos_prop, ref_prop)

print(f"RMSE (position vector): Non={rmse_xyz_x:.5f}, Conventional={rmse_xyz_conv:.5f}, Proposed={rmse_xyz_prop:.5f}")
