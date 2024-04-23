#!/bin/python3.6
import cv2
import numpy as np
from dt_apriltags import Detector 
import subprocess

imagepath = '../captured_img/captured_image2.jpg'
image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)

camera_calibration = np.load("../calibration/calibration.npy")
mtx, dist, rvecs, tvecs = camera_calibration
fx, fy, cx, cy = mtx[0, 0], mtx[1, 1], mtx[0, 2], mtx[1, 2]

detector = Detector(families='tag16h5', nthreads=4, quad_decimate=1.0, quad_sigma=0.0, 
                    refine_edges=1, decode_sharpening=0.25, debug=0)
# undistort
h,  w = image.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
image = cv2.undistort(image, mtx, dist, None, newcameramtx)
# crop the image
x, y, w, h = roi
image = image[y:y+h, x:x+w]

#detect image
tags = detector.detect(image, estimate_tag_pose=False, camera_params=[fx, fy, cx, cy], tag_size=0.03)
r = tags[0]
image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

# extract the bounding box (x, y)-coordinates for the AprilTag # and convert each of the (x, y)-coordinate pairs to integers 
(ptA, ptB, ptC, ptD) = r.corners
ptB = (int(ptB[0]), int(ptB [1])) 
ptC = (int(ptC[0]), int(ptC [1])) 
ptD = (int(ptD[0]), int(ptD [1])) 
ptA = (int(ptA[0]), int(ptA [1]))

# draw the bounding box of the AprilTag detection 
cv2.line(image, ptA, ptB, (0, 255, 0), 1) 
cv2.line(image, ptB, ptC, (0, 255, 0), 1) 
cv2.line(image, ptC, ptD, (0, 255, 0), 1) 
cv2.line(image, ptD, ptA, (0, 255, 0), 1)

# draw the center (x, y)-coordinates of the AprilTag 
(cX, cY) = (int(r.center[0]), int(r.center[1]))
(cX, cY) = (int(r.center[0]), int(r.center[1]))
#cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

# draw the tag family on the image
tagFamily = r.tag_family.decode("utf-8")
cv2.putText(image, tagFamily, (ptA[0], ptA[1]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
print(f"[INFO] tag family: {tagFamily}")

base, extension = imagepath.rsplit('.', 1)
outimage = f"{base}_out_undistorted.{extension}"
cv2.imwrite(outimage, image)

scp_command = [ 'scp', outimage, 'bradley@10.0.0.1:/home/bradley/Downloads']
try:
    subprocess.run(scp_command, check=True)
    print("File successfully transferred via SCP!")
except subprocess.CalledProcessError as e:
    print(f"Error transferring file via SCP: {e}")


