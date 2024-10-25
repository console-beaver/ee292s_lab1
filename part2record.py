#!usr/bin/env python3

# this is the file for running part2

import numpy as np
from filterpy.kalman import KalmanFilter
import pandas as pd
from matplotlib import pyplot as plt

DELTA_T = 0.00415183  # 21.610280 / 5205

# first, calibration

columns = ['row', 't', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']

calib_df = pd.read_csv('recorded_data/super_high_sample_rate_stationary.csv')
live_df = pd.read_csv('recorded_data/latest_chair.csv')

avg_ax = calib_df['ax'].sum() / len(calib_df['ax'])
avg_ay = calib_df['ay'].sum() / len(calib_df['ay'])
avg_az = calib_df['az'].sum() / len(calib_df['az'])

# kalman init

kf = KalmanFilter(dim_x=3, dim_z=1)
kf.x = np.array([0, 0, 0])  # position, velocity, accel
kf.P = np.eye(3)  # guess for the noise variance

kf.R = [[1]]
kf.Q = np.zeros((3, 3))

kf.F = np.array([[1, DELTA_T, 0.5 * DELTA_T**2],
                 [0, 1, DELTA_T],
                 [0, 0, 1]])

kf.H = np.array([[1, 0, 0]])

# simulate going through the data

elapsed_time = 0
x_pos = []
for row in range(live_df.shape[0]):
    x_accel_corrected = (live_df['ax'][row] - avg_ax) 
    kf.predict()
    kf.update(x_accel_corrected)
    x_pos.append(kf.x[0] * 9.81 / 16384)

    elapsed_time += DELTA_T

plt.plot(list(range(live_df.shape[0])), x_pos)
plt.show()
