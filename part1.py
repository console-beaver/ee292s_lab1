#!/usr/bin/env python3

import time
from ICM20948 import *
import ICM20948
import smbus
import math

import matplotlib.pyplot as plt
import numpy as np

UPDATE_PERIOD = 0.2
GYRO_SCALE = 2000 / 2 ** 16 
COMBINED_THRESH = 10
beta = 0.9

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

def main():
    icm20948=ICM20948.ICM20948()
    gyro_state = [0.0, 0.0, 0.0]  # TODO: fix this initialization!
    delta_t = 0
    first_time = True
    fused = 0.0
    constant = 0.0

    # real-time plotting init
    accel_hist = [0]
    gyro_hist = [0]
    fused_hist = [0]
    time_vec = [0]
    beginning_time = time.time()
    fig, ax = plt.subplots(1,1)
    line1, = ax.plot(time_vec, np.squeeze(accel_hist))
    line2, = ax.plot(time_vec, np.squeeze(gyro_hist))
    line3, = ax.plot(time_vec, np.squeeze(fused_hist))
    refline, = ax.plot(time_vec, np.squeeze(fused_hist))

    plt.xlabel('time')
    plt.ylabel('gravity angle')
    plt.title('gravity angle live plot')
    plt.legend(['accel', 'gyro', 'fused'], loc='upper left')
    fig.canvas.draw()
    axbackground = fig.canvas.copy_from_bbox(ax.bbox)
    ax.set_ylim(0, 180)
    plt.show(block=False)

    icm20948.icm20948_Gyro_Accel_Read()
    g = np.array(Accel)
    g_gyro = np.array(Accel)
    start_time = time.time()
    while True:
        # time.sleep(0.1)

        icm20948.icm20948_Gyro_Accel_Read()
        delta_t = time.time() - start_time
        start_time = time.time()

        delta_theta_x, delta_theta_y, delta_theta_z = (
            np.array(Gyro) * delta_t * np.pi / 180 * GYRO_SCALE
        )
        
        g = (
            R_x(delta_theta_x)
            @ R_y(delta_theta_y) @ R_z(delta_theta_z) @ g
        )

        part1_fused = math.atan2((g[0] ** 2 + g[1] ** 2) ** 0.5, g[2])
        part1_accel = math.atan2((Accel[0] ** 2 + Accel[1] ** 2) ** 0.5, Accel[2])

        part1_fused = (part1_fused * beta + (1 - beta) * part1_accel)

        g_gyro = (
            R_x(delta_theta_x)
            @ R_y(delta_theta_y) @ R_z(delta_theta_z) @ g_gyro
        )
        part1_gyro = math.atan2((g_gyro[0] ** 2 + g_gyro[1] ** 2) ** 0.5, g_gyro[2])

        # real-time plot:
        time_vec.append(start_time - beginning_time)
        accel_hist.append(part1_accel * 57.3)
        gyro_hist.append(part1_gyro * 57.3)
        fused_hist.append(part1_fused * 57.3)

        line1.set_xdata(time_vec)
        line1.set_ydata(np.squeeze(accel_hist))

        line2.set_xdata(time_vec)
        line2.set_ydata(np.squeeze(gyro_hist))

        line3.set_xdata(time_vec)
        line3.set_ydata(np.squeeze(fused_hist))

        refline.set_xdata(time_vec)
        refline.set_ydata(np.squeeze([90]*len(time_vec)))

        fig.canvas.restore_region(axbackground)
        ax.set_xlim(0, time_vec[-1])
        ax.draw_artist(line1)
        ax.draw_artist(line2)
        ax.draw_artist(line3)
        ax.draw_artist(refline)
        fig.canvas.blit(ax.bbox)

if __name__ == '__main__':
    main()
