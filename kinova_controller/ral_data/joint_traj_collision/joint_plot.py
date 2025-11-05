import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import trapz
from sklearn.metrics import mean_squared_error
from math import sqrt

final_time = 30.0
traj_final_time = 21.2

# data_pd = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/ral_data/joint_traj_collision/PD.txt') 
data_pd = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/ral_data/joint_traj_collision/PD_2.txt') 
time_pd = data_pd[:,0]
j1_pd = np.arctan2(data_pd[:,2], data_pd[:,1])
j2_pd = data_pd[:,3]
j3_pd = np.arctan2(data_pd[:,5], data_pd[:,4])
j4_pd = data_pd[:,6]
j5_pd = np.arctan2(data_pd[:,8], data_pd[:,7])    
j6_pd = data_pd[:,9]
j7_pd = np.arctan2(data_pd[:,11], data_pd[:,10])
j1_d_pd = np.arctan2(data_pd[:,13], data_pd[:,12])
j2_d_pd = data_pd[:,14]
j3_d_pd = np.arctan2(data_pd[:,16], data_pd[:,15])
j4_d_pd = data_pd[:,17]
j5_d_pd = np.arctan2(data_pd[:,19], data_pd[:,18])
j6_d_pd = data_pd[:,20]
j7_d_pd = np.arctan2(data_pd[:,22], data_pd[:,21])
enr_1_pd = data_pd[:,23]
enr_2_pd = data_pd[:,24]
enr_3_pd = data_pd[:,25]
enr_4_pd = data_pd[:,26]
enr_5_pd = data_pd[:,27]
enr_6_pd = data_pd[:,28]
enr_7_pd = data_pd[:,29]
edn_1_pd = data_pd[:,30]
edn_2_pd = data_pd[:,31]
edn_3_pd = data_pd[:,32]
edn_4_pd = data_pd[:,33]
edn_5_pd = data_pd[:,34]
edn_6_pd = data_pd[:,35]
edn_7_pd = data_pd[:,36]

# data_l1 = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/ral_data/joint_traj_collision/L1.txt') 
data_l1 = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/ral_data/joint_traj_collision/L1_2.txt') 
time_l1 = data_l1[:,0]
j1_l1 = np.arctan2(data_l1[:,2], data_l1[:,1])
j2_l1 = data_l1[:,3]
j3_l1 = np.arctan2(data_l1[:,5], data_l1[:,4])
j4_l1 = data_l1[:,6]
j5_l1 = np.arctan2(data_l1[:,8], data_l1[:,7])    
j6_l1 = data_l1[:,9]
j7_l1 = np.arctan2(data_l1[:,11], data_l1[:,10])
j1_d_l1 = np.arctan2(data_l1[:,13], data_l1[:,12])
j2_d_l1 = data_l1[:,14]
j3_d_l1 = np.arctan2(data_l1[:,16], data_l1[:,15])
j4_d_l1 = data_l1[:,17]
j5_d_l1 = np.arctan2(data_l1[:,19], data_l1[:,18])
j6_d_l1 = data_l1[:,20]
j7_d_l1 = np.arctan2(data_l1[:,22], data_l1[:,21])
enr_1_l1 = data_l1[:,23]
enr_2_l1 = data_l1[:,24]
enr_3_l1 = data_l1[:,25]
enr_4_l1 = data_l1[:,26]
enr_5_l1 = data_l1[:,27]
enr_6_l1 = data_l1[:,28]
enr_7_l1 = data_l1[:,29]
edn_1_l1 = data_l1[:,30]
edn_2_l1 = data_l1[:,31]
edn_3_l1 = data_l1[:,32]
edn_4_l1 = data_l1[:,33]
edn_5_l1 = data_l1[:,34]
edn_6_l1 = data_l1[:,35]
edn_7_l1 = data_l1[:,36]

plt.figure(1)
plt.subplot(7,2,1)
plt.plot(time_pd, j1_d_pd, 'k')
plt.plot(time_pd, j1_pd, 'r')
plt.plot(time_l1, j1_l1, 'b-.')
plt.xlim([0.0, final_time])

plt.subplot(7,2,2)
plt.plot(time_pd, j1_d_pd - j1_pd, 'r')
plt.plot(time_l1, j1_d_l1 - j1_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])

plt.subplot(7,2,3)
plt.plot(time_pd, j2_d_pd, 'k')
plt.plot(time_pd, j2_pd, 'r')
plt.plot(time_l1, j2_l1, 'b-.')
plt.xlim([0.0, final_time])

plt.subplot(7,2,4)
plt.plot(time_pd, j2_d_pd - j2_pd, 'r')
plt.plot(time_l1, j2_d_l1 - j2_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])

plt.subplot(7,2,5)
plt.plot(time_pd, j3_d_pd, 'k')
plt.plot(time_pd, j3_pd, 'r')
plt.plot(time_l1, j3_l1, 'b-.')
plt.xlim([0.0, final_time])

plt.subplot(7,2,6)
plt.plot(time_pd, j3_d_pd - j3_pd, 'r')
plt.plot(time_l1, j3_d_l1 - j3_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])

plt.subplot(7,2,7)
plt.plot(time_pd, j4_d_pd, 'k')
plt.plot(time_pd, j4_pd, 'r')
plt.plot(time_l1, j4_l1, 'b-.')
plt.xlim([0.0, final_time])

plt.subplot(7,2,8)
plt.plot(time_pd, j4_d_pd - j4_pd, 'r')
plt.plot(time_l1, j4_d_l1 - j4_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])

plt.subplot(7,2,9)
plt.plot(time_pd, j5_d_pd, 'k')
plt.plot(time_pd, j5_pd, 'r')
plt.plot(time_l1, j5_l1, 'b-.')
plt.xlim([0.0, final_time])

plt.subplot(7,2,10)
plt.plot(time_pd, j5_d_pd - j5_pd, 'r')
plt.plot(time_l1, j5_d_l1 - j5_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])

plt.subplot(7,2,11)
plt.plot(time_pd, j6_d_pd, 'k')
plt.plot(time_pd, j6_pd, 'r')
plt.plot(time_l1, j6_l1, 'b-.')
plt.xlim([0.0, final_time])

plt.subplot(7,2,12)
plt.plot(time_pd, j6_d_pd - j6_pd, 'r')
plt.plot(time_l1, j6_d_l1 - j6_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])

plt.subplot(7,2,13)
plt.plot(time_pd, j7_d_pd, 'k')
plt.plot(time_pd, j7_pd, 'r')
plt.plot(time_l1, j7_l1, 'b-.')
plt.xlim([0.0, final_time])

plt.subplot(7,2,14)
plt.plot(time_pd, j7_d_pd - j7_pd, 'r')
plt.plot(time_l1, j7_d_l1 - j7_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])

plt.figure(2)
plt.subplot(7,2,1)
plt.plot(time_pd, enr_1_pd, 'r')
plt.plot(time_l1, enr_1_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])

plt.subplot(7,2,2)
plt.plot(time_pd, edn_1_pd, 'r')
plt.plot(time_l1, edn_1_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])
plt.ylim([-0.02, 0.02])

plt.subplot(7,2,3)
plt.plot(time_pd, enr_2_pd, 'r')
plt.plot(time_l1, enr_2_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])

plt.subplot(7,2,4)
plt.plot(time_pd, edn_2_pd, 'r')
plt.plot(time_l1, edn_2_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])
plt.ylim([-0.02, 0.02])

plt.subplot(7,2,5)
plt.plot(time_pd, enr_3_pd, 'r')
plt.plot(time_l1, enr_3_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])

plt.subplot(7,2,6)
plt.plot(time_pd, edn_3_pd, 'r')
plt.plot(time_l1, edn_3_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])
plt.ylim([-0.02, 0.02])

plt.subplot(7,2,7)
plt.plot(time_pd, enr_4_pd, 'r')
plt.plot(time_l1, enr_4_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])

plt.subplot(7,2,8)
plt.plot(time_pd, edn_4_pd, 'r')
plt.plot(time_l1, edn_4_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])
plt.ylim([-0.02, 0.02])

plt.subplot(7,2,9)
plt.plot(time_pd, enr_5_pd, 'r')
plt.plot(time_l1, enr_5_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])

plt.subplot(7,2,10)
plt.plot(time_pd, edn_5_pd, 'r')
plt.plot(time_l1, edn_5_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])
plt.ylim([-0.02, 0.02])

plt.subplot(7,2,11)
plt.plot(time_pd, enr_6_pd, 'r')
plt.plot(time_l1, enr_6_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])

plt.subplot(7,2,12)
plt.plot(time_pd, edn_6_pd, 'r')
plt.plot(time_l1, edn_6_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])

plt.subplot(7,2,13)
plt.plot(time_pd, enr_7_pd, 'r')
plt.plot(time_l1, enr_7_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])

plt.subplot(7,2,14)
plt.plot(time_pd, edn_7_pd, 'r')
plt.plot(time_l1, edn_7_l1, 'b-.')
plt.axhline(0.0, 0, 10, color='lightgray', linestyle='--', linewidth=2)
plt.xlim([0.0, traj_final_time])
plt.show()

time_range = (time_pd >= 0) & (time_pd <= 21.2)
j1_pd = j1_pd[time_range]
j1_d_l1 = j1_d_l1[time_range]
j1_l1 = j1_l1[time_range]
j1_d_pd = j1_d_pd[time_range]

j2_pd = j2_pd[time_range]
j2_d_l1 = j2_d_l1[time_range]
j2_l1 = j2_l1[time_range]
j2_d_pd = j2_d_pd[time_range]

j3_pd = j3_pd[time_range]
j3_d_l1 = j3_d_l1[time_range]
j3_l1 = j3_l1[time_range]
j3_d_pd = j3_d_pd[time_range]

j4_pd = j4_pd[time_range]
j4_d_l1 = j4_d_l1[time_range]
j4_l1 = j4_l1[time_range]
j4_d_pd = j4_d_pd[time_range]

j5_pd = j5_pd[time_range]
j5_d_l1 = j5_d_l1[time_range]
j5_l1 = j5_l1[time_range]
j5_d_pd = j5_d_pd[time_range]

j6_pd = j6_pd[time_range]
j6_d_l1 = j6_d_l1[time_range]
j6_l1 = j6_l1[time_range]
j6_d_pd = j6_d_pd[time_range]

j7_pd = j7_pd[time_range]
j7_d_l1 = j7_d_l1[time_range]
j7_l1 = j7_l1[time_range]
j7_d_pd = j7_d_pd[time_range]

rmse1_pd = sqrt(mean_squared_error(j1_pd, j1_d_l1))
rmse1_l1 = sqrt(mean_squared_error(j1_l1, j1_d_l1))

rmse2_pd = sqrt(mean_squared_error(j2_pd, j2_d_l1))
rmse2_l1 = sqrt(mean_squared_error(j2_l1, j2_d_l1))

rmse3_pd = sqrt(mean_squared_error(j3_pd, j3_d_l1))
rmse3_l1 = sqrt(mean_squared_error(j3_l1, j3_d_l1))

rmse4_pd = sqrt(mean_squared_error(j4_pd, j4_d_l1))
rmse4_l1 = sqrt(mean_squared_error(j4_l1, j4_d_l1))

rmse5_pd = sqrt(mean_squared_error(j5_pd, j5_d_l1))
rmse5_l1 = sqrt(mean_squared_error(j5_l1, j5_d_l1))

rmse6_pd = sqrt(mean_squared_error(j6_pd, j6_d_l1))
rmse6_l1 = sqrt(mean_squared_error(j6_l1, j6_d_l1))

rmse7_pd = sqrt(mean_squared_error(j7_pd, j7_d_l1))
rmse7_l1 = sqrt(mean_squared_error(j7_l1, j7_d_l1))

print(f"j1: PD: {rmse1_pd:.4f}, L1: {rmse1_l1:.4f}") 
print(f"j2: PD: {rmse2_pd:.4f}, L1: {rmse2_l1:.4f}")
print(f"j3: PD: {rmse3_pd:.4f}, L1: {rmse3_l1:.4f}")
print(f"j4: PD: {rmse4_pd:.4f}, L1: {rmse4_l1:.4f}")
print(f"j5: PD: {rmse5_pd:.4f}, L1: {rmse5_l1:.4f}")
print(f"j6: PD: {rmse6_pd:.4f}, L1: {rmse6_l1:.4f}")
print(f"j7: PD: {rmse7_pd:.4f}, L1: {rmse7_l1:.4f}")


# print(rmse_l1)
# print(rmse_pd)
