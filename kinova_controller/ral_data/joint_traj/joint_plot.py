import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import trapz
from sklearn.metrics import mean_squared_error
from math import sqrt

data = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/ral_data/joint_traj/L1_2.txt') 
time = data[:,0]
j2 = data[:,3]
j4 = data[:,6]
j6 = data[:,9]
j2_d = data[:,14]
j4_d = data[:,17]
j6_d = data[:,20]
enr_2 = data[:,23]
enr_4 = data[:,25]
enr_6 = data[:,27]
edn_2 = data[:,30]
edn_4 = data[:,32]
edn_6 = data[:,34]

data_pd = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/ral_data/joint_traj/PD_2.txt') 
time_pd = data_pd[:,0]
j2_pd = data_pd[:,3]
j4_pd = data_pd[:,6]
j6_pd = data_pd[:,9]
j2_d_pd = data_pd[:,14]
j4_d_pd = data_pd[:,17]
j6_d_pd = data_pd[:,20]
enr_2_pd = data_pd[:,23]
enr_4_pd = data_pd[:,25]
enr_6_pd = data_pd[:,27]
edn_2_pd = data_pd[:,30]
edn_4_pd = data_pd[:,32]
edn_6_pd = data_pd[:,34]

data_non = np.loadtxt('/home/lee8646/ros2_ws/src/kinova_controller/ral_data/joint_traj/non_2.txt') 
time_non = data_non[:,0]
j2_non = data_non[:,3]
j4_non = data_non[:,6]
j6_non = data_non[:,9]

plt.figure(1)
plt.subplot(2,2,1)
plt.plot(time, j2_d, 'k')
plt.plot(time, j2, 'r--')
plt.plot(time, j2_pd, 'b-.')
plt.plot(time, j2_non, 'm:')
plt.xlim([0.0, 15.0])

plt.subplot(2,2,2)
plt.plot(time, j4_d, 'k')
plt.plot(time, j4, 'r--')
plt.plot(time, j4_pd, 'b-.')
plt.plot(time, j4_non, 'm:')
plt.xlim([0.0, 15.0])

plt.subplot(2,2,3)
plt.plot(time, enr_2_pd, 'r')
plt.plot(time, enr_4_pd, 'g')
plt.plot(time, enr_6_pd, 'b')
# plt.plot(time, j2_d - j2_non, 'm')
# plt.plot(time, j2_d - j2, 'r')
# plt.plot(time, j2_d_pd - j2_pd, 'b')
plt.xlim([0.0, 15.0])

plt.subplot(2,2,4)
plt.plot(time, enr_2, 'r')
plt.plot(time, enr_4, 'g')
plt.plot(time, enr_6, 'b')
# plt.plot(time, j4_d - j4_non, 'm')
# plt.plot(time, j4_d - j4, 'r')
# plt.plot(time, j4_d_pd - j4_pd, 'b')
plt.xlim([0.0, 15.0])
plt.show()

time_range = (time >= 0) & (time <= 11)

abs_error_non2 = np.abs(j6_d - j6_non)
abs_error_l12 = np.abs(j6_d - j6)
abs_error_pd2 = np.abs(j6_d - j6_pd)

abs_error_non2 = abs_error_non2[time_range]
abs_error_l12 = abs_error_l12[time_range]
abs_error_pd2 = abs_error_pd2[time_range]
time = time[time_range]

# integral_abs_error_non2 = trapz(abs_error_non2, time)
# integral_abs_error_l12 = trapz(abs_error_l12, time)
# integral_abs_error_pd2 = trapz(abs_error_pd2, time)
# print(integral_abs_error_non2)
# print(integral_abs_error_l12)
# print(integral_abs_error_pd2)

rmse_l1 = sqrt(mean_squared_error(j2, j2_d))
rmse_pd = sqrt(mean_squared_error(j2_pd, j2_d))
rmse_rd = sqrt(mean_squared_error(j2_non, j2_d))

print(rmse_l1)
print(rmse_pd)
print(rmse_rd)