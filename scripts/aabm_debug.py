#!/usr/bin/env python
import sys
from os.path import isfile

import rosbag
import numpy as np
from scipy.interpolate import interp1d

from matplotlib import rc
rc('font',**{'family':'serif','serif':['Times']})
rc('text', usetex=True)
import matplotlib.pylab as plt


from bag2csv import convert
from bag2csv import check_topic_exists


def load_battery_state(data_path):
    data = np.genfromtxt(data_path, delimiter=',')
    ts = data[:, 2] + data[:, 3] * 1e-9
    voltage = data[:,4]
    percentage = data[:,9]

    battery_state = {}
    battery_state["ts"] = ts
    battery_state["voltage"] = voltage
    battery_state["percentage"] = percentage

    return battery_state


def load_imu_data(data_path):
    data = np.genfromtxt(data_path, delimiter=',')
    ts = data[:, 2] + data[:, 3] * 1e-9
    angvel = data[:, 11:14]
    accel = data[:, 14:17]

    imu_data = {}
    imu_data["ts"] = ts
    imu_data["angvel"] = angvel
    imu_data["accel"] = accel

    return imu_data


def load_attitude_data(data_path):
    data = np.genfromtxt(data_path, delimiter=',')

    ts = data[:, 2] + data[:, 3] * 1e-9
    rpy = data[:, 8:11]
    body_rate = data[:, 11:14]
    thrust = data[:, 14]

    data = {}
    data["ts"] = ts
    data["rpy"] = rpy
    data["body_rate"] = body_rate
    data["thrust"] = thrust

    return data


def load_odom_data(data_path):
    data = np.genfromtxt(data_path, delimiter=',')
    ts = data[:, 2] + data[:, 3] * 1e-9
    pos = data[:, 5:8]
    rpy = data[:, 12:15]
    body_rate = data[:, 51:54]

    odom_data = {}
    odom_data["ts"] = ts
    odom_data["pos"] = pos
    odom_data["rpy"] = rpy
    odom_data["body_rate"] = body_rate
    return odom_data


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

    ref_data = {}
    ref_data["ts"] = ref_ts
    ref_data["pos"] = ref_pos
    return ref_data


def form_relative_time(datasets):
    """
    Converts dataset timestamps in nano-seconds to seconds relative to first
    timestamp.
    """
    # Find first timestamp between all datasets
    first_ts = float('inf')
    for ds in datasets:
        if min(ds["ts"]) < first_ts:
            first_ts = min(ds["ts"])

    # Create a new relative time entry on all datasets
    retval = []
    for ds in datasets:
        ds["time"] = ds["ts"] - first_ts
        retval.append(ds)

    return retval


def calculate_errors(odom, ref):
    # Extract the odometry data out to individual x, y, z components
    cmp_time = []
    odom_x = []
    odom_y = []
    odom_z = []
    for k in range(len(odom['time'])):
        t = odom['time'][k]

        if t > ref['time'][0] and t < ref['time'][-1]:
            cmp_time.append(t)
            odom_x.append(odom['pos'][k, 0])
            odom_y.append(odom['pos'][k, 1])
            odom_z.append(odom['pos'][k, 2])

    # Fit a spline function to the reference signal
    # We do this because the trajectory maybe of different rate to the odom,
    # therefore we need to fit a spline so that we can interpolate reference
    # positions at the same timestamps as the odometry data
    ref_x_func = interp1d(ref['time'], ref['pos'][:, 0])
    ref_y_func = interp1d(ref['time'], ref['pos'][:, 1])
    ref_z_func = interp1d(ref['time'], ref['pos'][:, 2])

    # Sample new reference signal data
    ref_x = ref_x_func(cmp_time)
    ref_y = ref_y_func(cmp_time)
    ref_z = ref_z_func(cmp_time)

    # Calculate absolute error between reference and odometry data
    err_x = np.abs(ref_x - odom_x) * 1e3  # Convert to mm
    err_y = np.abs(ref_y - odom_y) * 1e3  # Convert to mm
    err_z = np.abs(ref_z - odom_z) * 1e3  # Convert to mm

    return [cmp_time, err_x, err_y, err_z]


def plot_errors(odom, ref, **kwargs):
    err_data = calculate_errors(odom, ref)
    [cmp_time, err_x, err_y, err_z] = err_data

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

    print("")
    print("Tracking Errors")
    print("---------------")
    print("rmse:   [%f, %f, %f]" % (rmse_x, rmse_y, rmse_z))
    print("mean:   [%f, %f, %f]" % (mean_x, mean_y, mean_z))
    print("median: [%f, %f, %f]" % (median_x, median_y, median_z))
    print("stddev: [%f, %f, %f]" % (stddev_x, stddev_y, stddev_z))

    # plt.plot(cmp_time, err_x, 'r')
    # plt.plot(cmp_time, err_y, 'g')
    # plt.plot(cmp_time, err_z, 'b')
    # plt.show()

    # Plot boxplot
    padding = [(i * 2) for i in reversed(range(4))]

    if kwargs.get("subplot", None):
        plt.subplot(kwargs["subplot"])
        padding = [(i * 6) for i in reversed(range(4))]
    else:
        plt.figure()

    plt.boxplot([err_x, err_y, err_z], labels=['x', 'y', 'z'])
    plt.text(1.17, median_x + padding[0], "Mean: %.2f" % mean_x, fontsize=9)
    plt.text(1.17, median_x + padding[1], "Median: %.2f" % median_x, fontsize=9)
    plt.text(1.17, median_x + padding[2], "Stddev: %.2f" % stddev_x, fontsize=9)
    plt.text(1.17, median_x + padding[3], "Max: %.2f" % max_x, fontsize=9)

    plt.text(2.17, median_y + padding[0], "Mean: %.2f" % mean_y, fontsize=9)
    plt.text(2.17, median_y + padding[1], "Median: %.2f" % median_y, fontsize=9)
    plt.text(2.17, median_y + padding[2], "Stddev: %.2f" % stddev_y, fontsize=9)
    plt.text(2.17, median_y + padding[3], "Max: %.2f" % max_y, fontsize=9)

    plt.text(3.17, median_z + padding[0], "Mean: %.2f" % mean_z, fontsize=9)
    plt.text(3.17, median_z + padding[1], "Median: %.2f" % median_z, fontsize=9)
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


def plot_tracking(odom, ref, **kwargs):
    if kwargs.get("subplot", None):
        plt.subplot(kwargs["subplot"])
    else:
        plt.figure()

    plt.plot(odom['time'], odom['pos'][:, 0], 'r-', label="x")
    plt.plot(odom['time'], odom['pos'][:, 1], 'g-', label="y")
    plt.plot(odom['time'], odom['pos'][:, 2], 'b-', label="z")

    plt.plot(ref['time'], ref['pos'][:, 0], 'r--', label="ref x")
    plt.plot(ref['time'], ref['pos'][:, 1], 'g--', label="ref y")
    plt.plot(ref['time'], ref['pos'][:, 2], 'b--', label="ref z")

    step = 0.25
    val_min = round(min(np.amin(odom['pos']), np.amin(ref['pos'])) - step, 1)
    val_max = round(max(np.amax(odom['pos']), np.amax(ref['pos'])) + step, 1)
    plt.yticks(np.arange(val_min, val_max, step))

    plt.title("Trajectory Tracking Over Time")
    plt.xlim([ref['time'][0], ref['time'][-1]])
    plt.xlabel("Time [s]")
    plt.ylabel("Displacement [m]")
    plt.legend(loc=0)


def plot_battery_state(batt_data, **kwargs):
    if kwargs.get("subplot", None):
        plt.subplot(kwargs["subplot"])
    else:
        plt.figure()

    plt.plot(batt_data["time"], batt_data["voltage"])

    plt.title("Battery State")
    plt.xlabel("Time [s]");
    plt.ylabel("Voltage [V]");
    plt.xlim([batt_data["time"][0], batt_data["time"][-1]])
    plt.ylim([min(batt_data["voltage"]), max(batt_data["voltage"])])


def plot_imu(data, **kwargs):
    plt.figure()

    plt.subplot(211)
    plt.plot(data["time"], data["angvel"][:, 0], 'r-', label="x")
    plt.plot(data["time"], data["angvel"][:, 1], 'g-', label="y")
    plt.plot(data["time"], data["angvel"][:, 2], 'b-', label="z")

    plt.title("Gyroscope");
    plt.xlabel("Time [s]");
    plt.ylabel("Angular Velocity [rad s^-1]");
    plt.xlim([data["time"][0], data["time"][-1]])
    plt.legend(loc=0)

    plt.subplot(212)
    plt.plot(data["time"], data["accel"][:, 0], 'r-', label="x")
    plt.plot(data["time"], data["accel"][:, 1], 'g-', label="y")
    plt.plot(data["time"], data["accel"][:, 2], 'b-', label="z")

    plt.title("Accelerometer");
    plt.xlabel("Time [s]");
    plt.ylabel("Acceleration [ms^-2]");
    plt.xlim([data["time"][0], data["time"][-1]])
    plt.legend(loc=0)

    plt.tight_layout()
    plt.suptitle("IMU Measurements", fontsize=16)


def plot_attitude(target, odom):
    # Plot attitude
    plt.figure()

    plt.subplot(411)
    plt.plot(target["time"], target["rpy"][:, 0], 'r--', label='Target')
    plt.plot(odom["time"], odom["rpy"][:, 0], 'r-', label='Actual')
    plt.title("Roll");
    plt.xlabel("Time [s]");
    plt.ylabel("Angle [deg]");
    plt.xlim([target["time"][0], target["time"][-1]])
    plt.legend(loc=0)

    plt.subplot(412)
    plt.plot(target["time"], target["rpy"][:, 1], 'g--', label='Target')
    plt.plot(odom["time"], odom["rpy"][:, 1], 'g-', label='Actual')
    plt.title("Pitch");
    plt.xlabel("Time [s]");
    plt.ylabel("Angle [deg]");
    plt.xlim([target["time"][0], target["time"][-1]])
    plt.legend(loc=0)

    plt.subplot(413)
    plt.plot(target["time"], target["rpy"][:, 2], 'b--', label='Target')
    plt.plot(odom["time"], odom["rpy"][:, 2], 'b-', label='Actual')
    plt.title("Yaw");
    plt.xlabel("Time [s]");
    plt.ylabel("Angle [deg]");
    plt.xlim([target["time"][0], target["time"][-1]])
    plt.legend(loc=0)

    plt.subplot(414)
    plt.plot(target["time"], target["thrust"], 'k-', label='Thrust')
    plt.title("Thrust");
    plt.xlabel("Time [s]");
    plt.ylabel("Thrust");
    plt.xlim([target["time"][0], target["time"][-1]])
    plt.legend(loc=0)

    plt.suptitle("MAV Attitude", fontsize=16)
    plt.subplots_adjust(top=0.9, bottom=0.05, hspace=0.65)


    # Plot body rates
    plt.figure()

    plt.subplot(311)
    plt.plot(target["time"], target["body_rate"][:, 0], 'r--', label='Target')
    plt.plot(odom["time"], odom["body_rate"][:, 0], 'r-', label='Actual')
    plt.title("Body Rate: x-axis");
    plt.xlabel("Time [s]");
    plt.ylabel("Angular Velocity [rad/s]");
    plt.xlim([target["time"][0], target["time"][-1]])
    plt.legend(loc=0)

    plt.subplot(312)
    plt.plot(target["time"], target["body_rate"][:, 1], 'g--', label='Target')
    plt.plot(odom["time"], odom["body_rate"][:, 1], 'g-', label='Actual')
    plt.title("Body Rate: y-axis");
    plt.xlabel("Time [s]");
    plt.ylabel("Angular Velocity [rad/s]");
    plt.xlim([target["time"][0], target["time"][-1]])
    plt.legend(loc=0)

    plt.subplot(313)
    plt.plot(target["time"], target["body_rate"][:, 2], 'b--', label='Target')
    plt.plot(odom["time"], odom["body_rate"][:, 2], 'b-', label='Actual')
    plt.title("Body Rate: y-axis");
    plt.xlabel("Time [s]");
    plt.ylabel("Angular Velocity [rad/s]");
    plt.xlim([target["time"][0], target["time"][-1]])
    plt.legend(loc=0)

    plt.suptitle("MAV Body Rates", fontsize=16)
    plt.subplots_adjust(top=0.9, bottom=0.05, hspace=0.65)


if __name__ == "__main__":
    bag_path = sys.argv[1]
    bag = rosbag.Bag(bag_path, 'r')

    # Convert battery messages to csv
    batt_topic = "/AABM_01/mavros/battery"
    batt_file = "/tmp/battery.csv"
    if convert(bag, batt_topic, batt_file) is False:
        exit(-1)

    # Convert imu messages to csv
    imu_topic = "/AABM_01/mavros/imu/data_raw"
    imu_file = "/tmp/imu_raw.csv"
    if convert(bag, imu_topic, imu_file) is False:
        exit(-1)

    # Convert setpoint attitude messages to csv
    target_att_topic = "/AABM_01/mavros/setpoint_raw/target_attitude"
    target_att_file = "/tmp/target_attitude.csv"
    if convert(bag, target_att_topic, target_att_file) is False:
        exit(-1)

    # Convert odom messages to csv
    odom_topic = "/AABM_01/mavros/local_position/odom"
    odom_file = "/tmp/odom.csv"
    if convert(bag, odom_topic, odom_file) is False:
        exit(-1)

    # Convert trajectory or position reference messages to csv
    ref_data_type = None
    ref_file = "/tmp/ref.csv"
    traj_ref_topic = "/AABM_01/autopilot/TrajectoryReference"
    pos_ref_topic = "/AABM_01/autopilot/PositionReference"

    if check_topic_exists(bag, traj_ref_topic):
        ref_data_type = "traj_ref"
        convert(bag, traj_ref_topic, ref_file)
    else:
        ref_data_type = "pos_ref"
        convert(bag, pos_ref_topic, ref_file)

    # Load data
    batt_data = load_battery_state(batt_file)
    imu_data = load_imu_data(imu_file)
    target_att_data = load_attitude_data(target_att_file)
    odom_data = load_odom_data(odom_file)
    ref_data = load_ref_data(ref_data_type, ref_file)

    # Process data
    datasets = [batt_data, imu_data, target_att_data, odom_data, ref_data]
    [batt_data, imu_data, target_att_data, odom_data, ref_data] = form_relative_time(datasets)


    # Plot
    plot_imu(imu_data)
    # plot_battery_state(batt_data)
    plot_attitude(target_att_data, odom_data)

    plt.figure()
    plot_tracking(odom_data, ref_data, subplot="211")
    plot_errors(odom_data, ref_data, subplot="212")
    plt.subplots_adjust(top=0.95, bottom=0.05, hspace=0.4)
    plt.show()
