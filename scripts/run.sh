#!/usr/bin/env

# ./scripts/plot_ts_diff.m ./ts.csv

# BAG_PATHS=(/data/aabm/roll.bag /data/aabm/pitch.bag)
#
# convert_bag() {
#   BAG_PATH=$1;
#
#   python src/bag2csv.py \
#     $BAG_PATH \
#     /AABM_01/abm_ucl/vrpn_client/raw_transform \
#     raw_transform.csv
#
#   python src/bag2csv.py \
#     $BAG_PATH \
#     /AABM_01/mavros/imu/data \
#     imu.csv
#
#   python src/bag2csv.py \
#     $BAG_PATH \
#     /AABM_01/mavros/setpoint_raw/target_attitude \
#     target_attitude.csv
#
#   zip `basename ${BAG_PATH/.bag/.zip}` raw_transform.csv imu.csv target_attitude.csv
#   rm raw_transform.csv
#   rm imu.csv
#   rm target_attitude.csv
# }

# for BAG_PATH in ${BAG_PATHS[@]}; do
#   convert_bag $BAG_PATH
# done

# python src/bag2csv.py \
#   /data/aabm/mpc_circle_traj.bag \
#   /AABM_01/mavros/local_position/odom \
#   odom.csv

# python src/bag2csv.py \
#   /data/aabm/mpc_circle_traj.bag \
#   /AABM_01/autopilot/TrajectoryReference \
#   traj.csv

# python3 scripts/plot_tracking.py


# python src/bag2imgs.py \
#   /data/euroc_mav/calib/cam_april.bag \
#   /cam0/image_raw \
#   /tmp/cam0
#
# python src/bag2imgs.py \
#   /data/euroc_mav/calib/cam_april.bag \
#   /cam1/image_raw \
#   /tmp/cam1
