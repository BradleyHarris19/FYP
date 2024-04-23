#!/bin/python3.6
import numpy as np

# Load the numpy file to see the matrix inside
camera_calibration = np.load("calibration.npy")
mtx, dist, rvecs, tvecs = camera_calibration
for i in mtx:
    print(i)

