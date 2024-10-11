#!/usr/bin/env python3

import time
from ICM20948 import *
import ICM20948
import smbus
import math

UPDATE_PERIOD = 0.2
GYRO_SCALE = 1000 / 2 ** 16 

def main():
    icm20948=ICM20948.ICM20948()
    gyro_state = [0.0, 0.0, 0.0]  # TODO: fix this initialization!
    delta_t = 0
    while True:
        start_time = time.time()
        time.sleep(0.1)

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

        # accelerometer approach:

        part1_accel = math.atan2((Accel[0]**2 + Accel[1]**2)**0.5, float(Accel[2])) * 57.3
        print(f'PART 1 (with accelerometer): {part1_accel}')

        # gyro approach:

        gyro_state[0] += delta_t * Gyro[0] * GYRO_SCALE
        gyro_state[1] += delta_t * Gyro[1] * GYRO_SCALE
        gyro_state[2] += delta_t * Gyro[2] * GYRO_SCALE

        part1_gyro = math.acos(math.cos(math.radians(gyro_state[0])) * math.cos(math.radians(gyro_state[1]))) * 57.3
        print(f'PART 1 (with gyro): {part1_gyro}')

        # general measurements:

        print('\r\nRoll = %.2f , Pitch = %.2f , Yaw = %.2f\r\n'%(roll,pitch,yaw))
        print('\r\nAcceleration:  X = %d , Y = %d , Z = %d\r\n'%(Accel[0],Accel[1],Accel[2]))
        print('\r\nGyroscope:     X = %d , Y = %d , Z = %d\r\n'%(Gyro[0],Gyro[1],Gyro[2]))

        # real-time plot:


        delta_t = time.time() - start_time

if __name__ == '__main__':
    main()
