import numpy as np


camera_calibration = np.load("calibration.npy")
mtx, dist, rvecs, tvecs = camera_calibration
for i in mtx:
    print(i)

