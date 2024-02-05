import cv2
import numpy as np
from dt_apriltags import Detector 
import subprocess

camera_calibration = np.load("calibration/calibration.npy")
mtx, dist, rvecs, tvecs = camera_calibration

imagepath = 'captured_img/captured_image1.jpg'
image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
print(f"============== image shape: {image.shape}")
detector = Detector(families='tag16h5', nthreads=1, quad_decimate=1.0, quad_sigma=0.0, 
                    refine_edges=1, decode_sharpening=0.25, debug=0)
tags = detector.detect(image)
r = tags[0]
image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

# extract the bounding box (x, y)-coordinates for the AprilTag # and convert each of the (x, y)-coordinate pairs to integers 
(ptA, ptB, ptC, ptD) = r.corners
ptB = (int(ptB[0]), int(ptB [1])) 
ptC = (int(ptC[0]), int(ptC [1])) 
ptD = (int(ptD[0]), int(ptD [1])) 
ptA = (int(ptA[0]), int(ptA [1]))
corners = np.array([ptA, ptB, ptC, ptD])

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

#print(r.homography)
_, R, T, N = cv2.decomposeHomographyMat(r.homography, mtx)

R = np.array(R, np.float64)
T = np.array(T, np.float64)
N = np.array(N, np.float64)

print(f"Rotation matrix: {type(R)} {R.shape} \n{R}")
print(f"Translation vector: {type(T)} {T.shape} \n{T}")
print(f"Normal vector of plane: {type(N)} {N.shape} \n{N}")

Rvec = np.zeros((4, 3, 3))
Tvec = np.zeros((4, 3, 1))
# 3.3.1 i belive this step has already been done 
"""
found = 0
for i in range(4):
# put something here to check that the points translated in 3d space are positive
    #Reference-plane non-crossing constraint
    print("===============================")
    print(f"{N[i].shape}\n{N[i].transpose().shape}")
    print(f"{R[i]}\n{R[i].transpose()}")
    print("===============================")
    nc = N[i].transpose() * R[i].transpose()
    dnc= np.dot(N[i].T, R[i].T)
    print(nc)
    print(dnc)
    print("===============================")
    ncc = 1 + (nc * T[i])
    dncc=1 + np.dot(dnc, T[i])
    print(ncc)
    print(dncc)
    print("===============================")
    if ncc.all() > 0:
        Rvec[found] = R[i]
        Tvec[found] = T[i]
        found +=1

print(f"Rvec: {Rvec.shape} \n{Rvec}")
print(f"Tvec: {Tvec.shape} \n{Tvec}")
save_img = image
for i in range(4):
    image = save_img
"""

# 3.3.2 
"""
for i in range(4):
    m = dist.transpose() * corners[i]
    point = m.transpose()* N[i]
    if point.all() < 0:
        continue
    Rvec = R[i]
    Tvec = T[i]
"""

pad_width = ((0, 0), (0, 1))  # ((before_1, after_1), (before_2, after_2))
corners = np.pad(corners, pad_width, mode='constant', constant_values=0)


m = np.matmul(np.linalg.inv(mtx), corners.conjugate())
point = np.matmul(m.transpose(), N.conjugate())
print(point)


#axes = 2*np.sqrt((ptA[0] - ptB[0])**2 + (ptB[1] - ptA[1])**2)
#print(axes)
axis = np.float32([[1,0,0], [0,1,0], [0,0,-1]]).reshape(-1,3)
# project 3D points to image plane
imgpts, jac = cv2.projectPoints(axis, Rvec, Tvec, mtx, dist)
#print(f"imgpts:{i}\n{imgpts}")

corner = corners[2]
cv2.line(image, corner, tuple(imgpts[0].ravel()), (255,0,0), 1)
cv2.line(image, corner, tuple(imgpts[1].ravel()), (0,255,0), 1)
cv2.line(image, corner, tuple(imgpts[2].ravel()), (0,0,255), 1)

base, extension = imagepath.rsplit('.', 1)
outimage = f"{base}_pose[{i}].{extension}"
cv2.imwrite(outimage, image)

scp_command = [ 'scp', outimage, 'bradley@10.0.0.1:/home/bradley/Downloads']
try:
    subprocess.run(scp_command, check=True)
    print("File successfully transferred via SCP!")
except subprocess.CalledProcessError as e:
    print(f"Error transferring file via SCP: {e}")


