#!/usr/bin/env python3
import numpy as np
from scipy.interpolate import interp1d

from matplotlib import rc
rc('font',**{'family':'serif','serif':['Times']})
rc('text', usetex=True)
import matplotlib.pylab as plt

traj_data = np.genfromtxt('traj.csv', delimiter=',')
traj_ts = traj_data[:, 0]
traj_pos = traj_data[:, 1:4]

odom_data = np.genfromtxt('odom.csv', delimiter=',')
odom_ts = odom_data[:, 2] + odom_data[:, 3] * 1e-9
odom_pos = odom_data[:, 5:8]

first_ts = min(odom_ts[0], traj_ts[0])
odom_ts = odom_ts - first_ts
traj_ts = traj_ts - first_ts

traj_x_func = interp1d(traj_ts, traj_pos[:, 0])
traj_y_func = interp1d(traj_ts, traj_pos[:, 1])
traj_z_func = interp1d(traj_ts, traj_pos[:, 2])

cmp_ts = []
odom_x = []
odom_y = []
odom_z = []
for k in range(len(odom_ts)):
    ts = odom_ts[k]

    if ts > traj_ts[0] and ts < traj_ts[-1]:
        cmp_ts.append(ts)
        odom_x.append(odom_pos[k, 0])
        odom_y.append(odom_pos[k, 1])
        odom_z.append(odom_pos[k, 2])

traj_x = traj_x_func(cmp_ts)
traj_y = traj_y_func(cmp_ts)
traj_z = traj_z_func(cmp_ts)

err_x = np.abs(traj_x - odom_x) * 1e3  # Convert to mm
err_y = np.abs(traj_y - odom_y) * 1e3  # Convert to mm
err_z = np.abs(traj_z - odom_z) * 1e3  # Convert to mm

rmse_x = np.sqrt(np.mean(np.power(err_x, 2)))
rmse_y = np.sqrt(np.mean(np.power(err_y, 2)))
rmse_z = np.sqrt(np.mean(np.power(err_z, 2)))

mean_x = np.mean(err_x)
mean_y = np.mean(err_y)
mean_z = np.mean(err_z)

median_x = np.median(err_x)
median_y = np.median(err_y)
median_z = np.median(err_z)

stddev_x = np.std(err_x)
stddev_y = np.std(err_y)
stddev_z = np.std(err_z)

max_x = np.max(err_x)
max_y = np.max(err_y)
max_z = np.max(err_z)

# plt.plot(cmp_ts, err_x, 'r')
# plt.plot(cmp_ts, err_y, 'g')
# plt.plot(cmp_ts, err_z, 'b')
# plt.show()

# plt.boxplot([err_x, err_y, err_z], labels=['x', 'y', 'z'])
# plt.text(1.17, median_x, "Mean: %.2f" % median_x, fontsize=9)
# plt.text(1.17, median_x - 1, "Median: %.2f" % mean_x, fontsize=9)
# plt.text(1.17, median_x - 2, "Stddev: %.2f" % stddev_x, fontsize=9)
# plt.text(1.17, median_x - 3, "Max: %.2f" % max_x, fontsize=9)
#
# plt.text(2.17, median_y, "Mean: %.2f" % median_y, fontsize=9)
# plt.text(2.17, median_y - 1, "Median: %.2f" % mean_y, fontsize=9)
# plt.text(2.17, median_y - 2, "Stddev: %.2f" % stddev_y, fontsize=9)
# plt.text(2.17, median_y - 3, "Max: %.2f" % max_y, fontsize=9)
#
# plt.text(3.17, median_z, "Mean: %.2f" % median_z, fontsize=9)
# plt.text(3.17, median_z - 1, "Median: %.2f" % mean_z, fontsize=9)
# plt.text(3.17, median_z - 2, "Stddev: %.2f" % stddev_z, fontsize=9)
# plt.text(3.17, median_z - 3, "Max: %.2f" % max_z, fontsize=9)
#
# step = 10.0
# val_min = 0.0
# val_max = max(np.amax(err_x), np.amax(err_y), np.amax(err_z)) + step
# val_max += val_max % 5
# plt.yticks(np.arange(val_min, val_max, step))
# plt.ylabel("Displacement Error [mm]")
# plt.title("Tracking Error Box Plot")
# plt.show()

# # Plot tracking
# plt.plot(odom_ts, odom_pos[:, 0], 'r-', label="x")
# plt.plot(odom_ts, odom_pos[:, 1], 'g-', label="y")
# plt.plot(odom_ts, odom_pos[:, 2], 'b-', label="z")
#
# plt.plot(traj_ts, traj_pos[:, 0], 'r--', label="ref x")
# plt.plot(traj_ts, traj_pos[:, 1], 'g--', label="ref y")
# plt.plot(traj_ts, traj_pos[:, 2], 'b--', label="ref z")
#
# plt.xlim([traj_ts[0], traj_ts[-1]])
#
# step = 0.25
# val_min = round(min(np.amin(odom_pos), np.amin(traj_pos)) - step, 1)
# val_max = round(max(np.amax(odom_pos), np.amax(traj_pos)) + step, 1)
# plt.yticks(np.arange(val_min, val_max, step))
#
# plt.xlabel("Time [s]")
# plt.ylabel("Displacement [m]")
# plt.title("Trajectory Tracking Over Time")
# plt.legend(loc=0)
#
# plt.show()
