#!/usr/bin/env python3
import numpy as np
from scipy.interpolate import interp1d

from matplotlib import rc
rc('font',**{'family':'serif','serif':['Times']})
rc('text', usetex=True)
import matplotlib.pylab as plt


def calculate_errors(odom_ts, odom_pos, ref_ts, ref_pos):
    # Extract the odometry data out to individual x, y, z components
    cmp_ts = []
    odom_x = []
    odom_y = []
    odom_z = []
    for k in range(len(odom_ts)):
        ts = odom_ts[k]

        if ts > ref_ts[0] and ts < ref_ts[-1]:
            cmp_ts.append(ts)
            odom_x.append(odom_pos[k, 0])
            odom_y.append(odom_pos[k, 1])
            odom_z.append(odom_pos[k, 2])

    # Fit a spline function to the reference signal
    # We do this because the trajectory maybe of different rate to the odom,
    # therefore we need to fit a spline so that we can interpolate reference
    # positions at the same timestamps as the odometry data
    ref_x_func = interp1d(ref_ts, ref_pos[:, 0])
    ref_y_func = interp1d(ref_ts, ref_pos[:, 1])
    ref_z_func = interp1d(ref_ts, ref_pos[:, 2])

    # Sample new reference signal data
    ref_x = ref_x_func(cmp_ts)
    ref_y = ref_y_func(cmp_ts)
    ref_z = ref_z_func(cmp_ts)

    # Calculate absolute error between reference and odometry data
    err_x = np.abs(ref_x - odom_x) * 1e3  # Convert to mm
    err_y = np.abs(ref_y - odom_y) * 1e3  # Convert to mm
    err_z = np.abs(ref_z - odom_z) * 1e3  # Convert to mm

    return [cmp_ts, err_x, err_y, err_z]


def plot_errors(odom_ts, odom_pos, ref_ts, ref_pos):
    err_data = calculate_errors(odom_ts, odom_pos, ref_ts, ref_pos)
    [cmp_ts, err_x, err_y, err_z] = err_data

    # Calculate RMSE error between reference and odometry data
    rmse_x = np.sqrt(np.mean(np.power(err_x, 2)))
    rmse_y = np.sqrt(np.mean(np.power(err_y, 2)))
    rmse_z = np.sqrt(np.mean(np.power(err_z, 2)))

    # Mean error
    mean_x = np.mean(err_x)
    mean_y = np.mean(err_y)
    mean_z = np.mean(err_z)

    # Median error
    median_x = np.median(err_x)
    median_y = np.median(err_y)
    median_z = np.median(err_z)

    # Standard deviation
    stddev_x = np.std(err_x)
    stddev_y = np.std(err_y)
    stddev_z = np.std(err_z)

    # Max Error
    max_x = np.max(err_x)
    max_y = np.max(err_y)
    max_z = np.max(err_z)

    print("Tracking Errors")
    print("rmse:   [%f, %f, %f]" % (rmse_x, rmse_y, rmse_z))
    print("mean:   [%f, %f, %f]" % (mean_x, mean_y, mean_z))
    print("median: [%f, %f, %f]" % (median_x, median_y, median_z))
    print("stddev: [%f, %f, %f]" % (stddev_x, stddev_y, stddev_z))

    # plt.plot(cmp_ts, err_x, 'r')
    # plt.plot(cmp_ts, err_y, 'g')
    # plt.plot(cmp_ts, err_z, 'b')
    # plt.show()

    # Plot boxplot
    plt.figure()
    padding = [0, -2, -4, -6]
    plt.boxplot([err_x, err_y, err_z], labels=['x', 'y', 'z'])
    plt.text(1.17, median_x + padding[0], "Mean: %.2f" % median_x, fontsize=9)
    plt.text(1.17, median_x + padding[1], "Median: %.2f" % mean_x, fontsize=9)
    plt.text(1.17, median_x + padding[2], "Stddev: %.2f" % stddev_x, fontsize=9)
    plt.text(1.17, median_x + padding[3], "Max: %.2f" % max_x, fontsize=9)

    plt.text(2.17, median_y + padding[0], "Mean: %.2f" % median_y, fontsize=9)
    plt.text(2.17, median_y + padding[1], "Median: %.2f" % mean_y, fontsize=9)
    plt.text(2.17, median_y + padding[2], "Stddev: %.2f" % stddev_y, fontsize=9)
    plt.text(2.17, median_y + padding[3], "Max: %.2f" % max_y, fontsize=9)

    plt.text(3.17, median_z + padding[0], "Mean: %.2f" % median_z, fontsize=9)
    plt.text(3.17, median_z + padding[1], "Median: %.2f" % mean_z, fontsize=9)
    plt.text(3.17, median_z + padding[2], "Stddev: %.2f" % stddev_z, fontsize=9)
    plt.text(3.17, median_z + padding[3], "Max: %.2f" % max_z, fontsize=9)

    step = 10.0
    # val_min = 0.0
    # val_max = max(np.amax(err_x), np.amax(err_y), np.amax(err_z)) + step
    # val_max += val_max % 5
    # plt.yticks(np.arange(val_min, val_max, step))
    plt.ylim([0, 100])
    plt.ylabel("Displacement Error [mm]")
    plt.title("Tracking Error Box Plot")


def plot_tracking(odom_ts, odom_pos, ref_ts, ref_pos):
    plt.figure()
    plt.plot(odom_ts, odom_pos[:, 0], 'r-', label="x")
    plt.plot(odom_ts, odom_pos[:, 1], 'g-', label="y")
    plt.plot(odom_ts, odom_pos[:, 2], 'b-', label="z")

    plt.plot(ref_ts, ref_pos[:, 0], 'r--', label="ref x")
    plt.plot(ref_ts, ref_pos[:, 1], 'g--', label="ref y")
    plt.plot(ref_ts, ref_pos[:, 2], 'b--', label="ref z")

    step = 0.25
    val_min = round(min(np.amin(odom_pos), np.amin(ref_pos)) - step, 1)
    val_max = round(max(np.amax(odom_pos), np.amax(ref_pos)) + step, 1)
    plt.yticks(np.arange(val_min, val_max, step))

    plt.xlim([ref_ts[0], ref_ts[-1]])
    plt.xlabel("Time [s]")
    plt.ylabel("Displacement [m]")
    plt.title("Trajectory Tracking Over Time")
    plt.legend(loc=0)


def load_odom_data(data_path):
    odom_data = np.genfromtxt(data_path, delimiter=',')
    odom_ts = odom_data[:, 2] + odom_data[:, 3] * 1e-9
    odom_pos = odom_data[:, 5:8]
    return [odom_ts, odom_pos]


def load_ref_data(data_type, data_path):
    ref_ts = []
    ref_pos = []

    if data_type == "traj_ref":
        ref_data = np.genfromtxt(data_path, delimiter=',')
        ref_ts = ref_data[:, 0]
        ref_pos = ref_data[:, 1:4]
    elif data_type == "pos_ref":
        ref_data = np.genfromtxt(data_path, delimiter=',')
        ref_ts = ref_data[:, 2] + ref_data[:, 3] * 1e-9
        ref_pos = ref_data[:, 4:7]
    else:
        print("ERROR! UNRECOGIZED DATA TYPE [%s]!" % data_type)

    return [ref_ts, ref_pos]


def adjust_timestamps(odom_ts, ref_ts):
    """
    Converts timestamps in nano-seconds to seconds relative to first timestamp.
    """
    first_ts = min(odom_ts[0], ref_ts[0])
    odom_ts = odom_ts - first_ts
    ref_ts = ref_ts - first_ts
    return [odom_ts, ref_ts]


if __name__ == "__main__":
    odom_data_path = "odom.csv"
    ref_data_type = "pos_ref"
    ref_data_path = "pos_ref.csv"

    [odom_ts, odom_pos] = load_odom_data(odom_data_path)
    [ref_ts, ref_pos] = load_ref_data(ref_data_type, ref_data_path)
    [odom_ts, ref_ts] = adjust_timestamps(odom_ts, ref_ts)

    plot_tracking(odom_ts, odom_pos, ref_ts, ref_pos)
    plot_errors(odom_ts, odom_pos, ref_ts, ref_pos)
    plt.show()
