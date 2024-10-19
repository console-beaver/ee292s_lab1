#!/usr/bin/env python3

import time
from ICM20948 import *
import ICM20948
import smbus
import math

import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import KalmanFilter

UPDATE_PERIOD = 0.2
GYRO_SCALE = 1000 / 2 ** 16 
COMBINED_THRESH = 10

DELTA_T = 0.1

def main():
    icm20948=ICM20948.ICM20948()
    gyro_state = [0.0, 0.0, 0.0]  # TODO: fix this initialization!
    delta_t = 0
    first_time = True
    fused = 0.0
    constant = 0.0

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

    # real-time plotting init
    xpos_hist = [0]
    time_vec = [0]
    beginning_time = time.time()
    fig, ax = plt.subplots(1,1)
    line1, = ax.plot(time_vec, np.squeeze(xpos_hist))
    refline, = ax.plot(time_vec, np.squeeze(xpos_hist))

    plt.xlabel('time')
    plt.ylabel('gravity angle')
    plt.title('gravity angle live plot')
    plt.legend(['Kalman'], loc='upper left')
    fig.canvas.draw()
    axbackground = fig.canvas.copy_from_bbox(ax.bbox)
    # ax.set_ylim(0, 180)
    plt.show(block=False)
    while True:
        start_time = time.time()
        # time.sleep(0.1)

        icm20948.icm20948_Gyro_Accel_Read()
        icm20948.icm20948MagRead()
        icm20948.icm20948CalAvgValue()
        icm20948.imuAHRSupdate(MotionVal[0] * 0.0175, MotionVal[1] * 0.0175,MotionVal[2] * 0.0175,
                    MotionVal[3],MotionVal[4],MotionVal[5],
                    MotionVal[6], MotionVal[7], MotionVal[8])
        pitch = math.asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3
        roll  = math.atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3
        yaw   = math.atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1) * 57.3

        print("\r\n /-------------------------------------------------------------/ \r\n")

        # remove accel drift from accel measurement caused by gravity

        corrected_accel = Accel[0]
        part1_accel =  ...
        

        # kalman filter x position

        kf.predict()
        kf.update(corrected_accel)
        part2_xpos = kf.x[0]  # x = [pos, vel, accel]
        # part1_accel = math.atan2((Accel[0]**2 + Accel[1]**2)**0.5, float(Accel[2])) * 57.3
        print(f'PART 2 (with accelerometer): {part2_xpos}')

        # general measurements:

        print('\r\nRoll = %.2f , Pitch = %.2f , Yaw = %.2f\r\n'%(roll,pitch,yaw))
        print('\r\nAcceleration:  X = %d , Y = %d , Z = %d\r\n'%(Accel[0],Accel[1],Accel[2]))
        print('\r\nGyroscope:     X = %d , Y = %d , Z = %d\r\n'%(Gyro[0],Gyro[1],Gyro[2]))

        # real-time plot:
        time_vec.append(start_time - beginning_time)
        xpos_hist.append(part2_xpos)

        line1.set_xdata(time_vec)
        line1.set_ydata(np.squeeze(xpos_hist))

        refline.set_xdata(time_vec)
        refline.set_ydata(np.squeeze([0]*len(time_vec)))

        fig.canvas.restore_region(axbackground)
        # ax.set_xlim(0, time_vec[-1])
        ax.relim()
        ax.autoscale_view()
        ax.draw_artist(line1)
        ax.draw_artist(refline)
        fig.canvas.blit(ax.bbox)

        delta_t = time.time() - start_time

if __name__ == '__main__':
    main()
