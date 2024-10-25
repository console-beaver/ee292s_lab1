# %%
import numpy as np
import pandas as pd
import math

import matplotlib.pyplot as plt

# %%

df = pd.read_csv("./recorded_data/newdata.csv")

# %%

df["dt"] = [df.t[0]] + (df.t[1:].values - df.t[:-1].values).tolist()


GYRO_SCALE = 2000 / (2 ** 16)
# %%

df[["gx", "gy", "gz"]] *= GYRO_SCALE

# %%

accel_fn = lambda x: math.atan2(
    (x["ax"]**2 + x["ay"]**2)**0.5,
    float(x["az"])
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

# %%

beta = 0.5

fused_thetas = []
gyro_thetas = []
accel_thetas = []

theta = accel_fn(df.iloc[0])
g_accel_fn = lambda x: np.array([x["ax"], x["ay"], x["az"]])
g = g_accel_fn(df.iloc[0])
g_gyro = g.copy()

k = 10
for i in range(0, len(df), k):
    delta_theta_x, delta_theta_y, delta_theta_z = (
        df[["gx", "gy", "gz"]].iloc[i]
        * df["dt"].iloc[i: i + k].sum()
        * np.pi / 180
    )
    
    g = (
        R_x(delta_theta_x)
        @ R_y(delta_theta_y) @ R_z(delta_theta_z) @ g
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

# plt.plot(np.array(fused_thetas) * 57.3, label="fused")
plt.plot(np.array(gyro_thetas) * 57.3, label="gyro")
plt.plot(np.array(accel_thetas) * 57.3, label="accel")
plt.legend()
plt.show()

# %%

theta_x = [0]
theta_y = [0]
theta_z = [0]
k = 10
for i in range(0, len(df), k):
    delta_theta_x, delta_theta_y, delta_theta_z = (
        df[["gx", "gy", "gz"]].iloc[i]
        * df["dt"].iloc[i: i + k].sum() #* GYRO_SCALE
        # * np.pi / 180
    )

    theta_x += [delta_theta_x + theta_x[-1]]
    theta_y += [delta_theta_y+ theta_y[-1]]
    theta_z += [delta_theta_z+ theta_z[-1]]

# %%


