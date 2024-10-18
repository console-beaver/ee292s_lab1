#!/usr/bin/env python3

import numpy as np
from tqdm import tqdm
from matplotlib import pyplot as plt

num_lines = 4561  # 25_min.txt
tau_range = 601

def get_cluster_avg(f, size):
    accum = np.zeros(9, dtype=float)
    for i in range(size):
        line = f.readline()
        if line == '': break
        datapoint = np.fromstring(line, sep=',', dtype=float)
        accum += datapoint
    return accum / size

with open('../lab1/ee292s_lab1/recorded_data/25_min.txt', 'r') as f:
    adevs = []
    x_axis = range(1, tau_range)

    all_data = np.array([np.fromstring(line, sep=',', dtype=float) for line in f])

    for t in tqdm(x_axis):
        K = (num_lines - 1) // t + 1
        avar = 0
        remaining = num_lines
        prev_cluster = None
        for k in range(K):
            if remaining < t: continue  # don't even consider incomplete clusters
            if prev_cluster is None:
                prev_cluster = get_cluster_avg(f, t)
                remaining -= t
                continue
            this_cluster = get_cluster_avg(f, t)
            avar += (this_cluster - prev_cluster)**2
            prev_cluster = this_cluster
            remaining -= t
        avar /= 2*(K - 1)
        adevs.append(np.sqrt(avar))
        f.seek(0)

    adevs = np.array(adevs)
    items = ['roll', 'pitch', 'yaw', 'accel_0', 'accel_1', 'accel_2', 'gyro_0', 'gyro_1', 'gyro_2']
    for i in range(len(items)):
        plt.loglog(x_axis, adevs[:, i], label=items[i])
        if i % 3 == 2:
            plt.grid()
            plt.legend()
            plt.xlabel('tau')
            plt.ylabel('ADEV')
            if i != 8: plt.figure()
    plt.show()
