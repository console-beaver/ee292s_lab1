#!usr/bin/env python3

# this is the file for running part2

import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import pandas as pd
from matplotlib import pyplot as plt

# DELTA_T = 0.00415183  # 21.610280 / 5205
DELTA_T = 0.01  # 21.610280 / 5205

# first, calibration

columns = ['row', 't', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']

# calib_df = pd.read_csv('recorded_data/super_high_sample_rate_stationary.csv')
df = pd.read_csv('recorded_data/6foot2.csv')

avg_ax = df["ax"].iloc[:1000].mean()

# kalman init

kf = KalmanFilter(dim_x=3, dim_z=1)
kf.x = np.array([0, 0, 0])  # position, velocity, accel
kf.P = np.eye(3)  # guess for the noise variance

kf.R = np.array([[0.01]])
kf.Q = Q_discrete_white_noise(dim=3, dt=DELTA_T, var=0.001)

kf.F = np.array([[1, DELTA_T, 0.5 * DELTA_T**2],
                 [0, 1, DELTA_T],
                 [0, 0, 1]])

kf.H = np.array([[0, 0, 1]])

# simulate going through the data
df = df.iloc[
    np.abs(df['t'].values[:, np.newaxis] - np.arange(0, df.t.max(), 0.01)).argmin(axis=0)
].reset_index(drop=True)

# %%

def integrate(x):
    y = [0]
    for i in range(len(x)):
        y += [x[i] * DELTA_T + y[-1]]
    return y[1:]

elapsed_time = 0
x_pos = []
a = []
for row in range(df.shape[0]):
    x_accel_corrected = (df['ax'][row] - avg_ax) 
    kf.predict()
    kf.update(x_accel_corrected)
    x_pos.append(kf.x[0] * 9.81 / 16384)
    a += [(df['ax'][row] - avg_ax)]

    elapsed_time += DELTA_T

plt.plot(list(range(df.shape[0])), x_pos)
plt.plot(list(range(df.shape[0])), np.array(integrate(integrate(a))) * 9.81 / 16384)
plt.show()
