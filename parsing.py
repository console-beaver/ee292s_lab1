# %%
import numpy as np
import pandas as pd
import math

import matplotlib.pyplot as plt

# %%

columns = ['roll', 'pitch', 'yaw', 'accel_0', 'accel_1', 'accel_2', 'gyro_0', 'gyro_1', 'gyro_2']
lines = [
    np.fromstring(line, sep=',', dtype=float).tolist()
    for line in open("./recorded_data/10_17_24_to90.txt")
]

lines = np.array([l for l in lines if len(l) > 0])

df = pd.DataFrame(lines, columns=columns)

# %%

GYRO_SCALE = 1000 / 2 ** 16

g_fn = lambda theta: np.array([0, np.sin(theta), np.cos(theta)])

accel_fn = lambda x: math.atan2(
    (
        x["accel_0"]**2 + x["accel_1"]**2)**0.5,
        float(x["accel_2"]
    )
)

R_x = lambda theta: np.array([
    [1, 0, 0],
    [0, np.cos(theta), -np.sin(theta)],
    [0, np.sin(theta), np.cos(theta)]
])

R_y = lambda theta: np.array([
    [np.cos(theta), 0, np.sin(theta)],
    [0, 1, 0],
    [-np.sin(theta), 0, np.cos(theta)]
])

R_z = lambda theta: np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta), np.cos(theta), 0],
    [0, 0, 1],
])
delta_t = 0.4

# %%

beta = 0.5

fused_thetas = []
gyro_thetas = []
accel_thetas = []

theta = accel_fn(df.iloc[0])
g_gyro = g_fn(accel_fn(df.iloc[0])) 

for i in range(len(df)):
    delta_theta_x, delta_theta_y, delta_theta_z = (
        df[["gyro_0", "gyro_1", "gyro_2"]].iloc[i]
        * delta_t * GYRO_SCALE
        * np.pi / 180
    )
    
    g = (
        R_x(delta_theta_x)
        @ R_y(delta_theta_y) @ R_z(delta_theta_z) @ g_fn(theta)
    )
    theta_fused = math.atan2((g[0] ** 2 + g[1] ** 2) ** 0.5, g[2])
    theta_accel = accel_fn(df.iloc[i])

    theta = theta_fused * beta + (1 - beta) * theta_accel
    fused_thetas += [theta]

    g_gyro = (
        R_x(delta_theta_x)
        @ R_y(delta_theta_y) @ R_z(delta_theta_z) @ g_gyro
    )
    gyro_thetas += [math.atan2((g_gyro[0] ** 2 + g_gyro[1] ** 2) ** 0.5, g_gyro[2])]


    accel_thetas += [accel_fn(df.iloc[i])]

plt.plot(np.array(fused_thetas) * 57.3, label="fused")
plt.plot(np.array(gyro_thetas) * 57.3, label="gyro")
plt.plot(np.array(accel_thetas) * 57.3, label="accel")
plt.legend()
plt.show()

# %%


# %%


