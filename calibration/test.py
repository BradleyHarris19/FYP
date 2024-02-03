import numpy as np
import cv2 as cv
import glob

# Load previously saved data
X = np.load('calibration.npy')
mtx, dist, rvecs, tvecs = X#X[0], X[1], X[2], X[3]

