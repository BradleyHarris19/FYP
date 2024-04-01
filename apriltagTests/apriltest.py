import cv2
import numpy as np
from dt_apriltags import Detector 

imagepath = '../captured_img/captured_image_uni.jpg'
image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)

camera_calibration = np.load("../calibration/calibration.npy")
mtx, dist, rvecs, tvecs = camera_calibration
fx, fy, cx, cy = mtx[0, 0], mtx[1, 1], mtx[0, 2], mtx[1, 2]

detector = Detector(families='tag16h5', nthreads=4, quad_decimate=1.0, quad_sigma=0.0,
                    refine_edges=1, decode_sharpening=0.25, debug=0)

tags = detector.detect(image, estimate_tag_pose=False, camera_params=[fx, fy, cx, cy], tag_size=0.03)

print(tags[0])
