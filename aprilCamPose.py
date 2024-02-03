import cv2
import socket
import pickle
import struct
import numpy as np
from dt_apriltags import Detector


server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 5555))
server_socket.listen(5)
print("Server is listening...")
client_socket, client_address = server_socket.accept()
print(f"Connection from {client_address} accepted")

# Create a GStreamer pipeline for the CSI camera
video_source=0
gst_str = f'nvarguscamerasrc sensor-id={video_source} ! video/x-raw(memory:NVMM), width=640, height=480,\
                format=(string)NV12, framerate=(fraction)10/1 ! nvvidconv ! video/x-raw, \
                format=(string)GRAY8 ! videoconvert ! video/x-raw, format=(string)GRAY8 ! appsink'
cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

camera_calibration = np.load("calibration/calibration.npy")
mtx, dist, rvecs, tvecs = camera_calibration

detector = Detector(families='tag16h5', nthreads=1, quad_decimate=1.0, quad_sigma=0.0,\
                    refine_edges=1, decode_sharpening=0.25, debug=0)

while True:
    ret, image = cap.read()
    #image = cv2.cvtColor(image, cv2.COLOR_BGR2GREY)
    if ret == 0:
        continue
    
    print(f"============== image shape: {image.shape}")
    tags = detector.detect(image)
    if len(tags) > 0:
        r = tags[0]

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

        #print(r.homography)
        _, R, T, N = cv2.decomposeHomographyMat(r.homography, mtx)

        R = np.array(R, np.float64)
        T = np.array(T, np.float64)
        N = np.array(N, np.float64)

        print(f"Rotation matrix: {type(R)} {R.shape} \n{R}")
        print(f"Translation vector: {type(T)} {T.shape} \n{T}")
        print(f"Normal vector of plane: {type(N)} {N.shape} \n{N}")

        #axes = 2*np.sqrt((ptA[0] - ptB[0])**2 + (ptB[1] - ptA[1])**2)
        #print(axes)
        axis = np.float32([[1,0,0], [0,1,0], [0,0,-1]]).reshape(-1,3)
        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, R[1], T[1], mtx, dist)
        print(imgpts)

        corner = ptC
        cv2.line(image, corner, tuple(imgpts[0].ravel()), (255,0,0), 1)
        cv2.line(image, corner, tuple(imgpts[1].ravel()), (0,255,0), 1)
        cv2.line(image, corner, tuple(imgpts[2].ravel()), (0,0,255), 1)

    frame_data = pickle.dumps(image)
    client_socket.sendall(struct.pack("Q", len(frame_data)))
    client_socket.sendall(frame_data)
    if cv2.waitKey(1) == 13:
        break
cap.release()

