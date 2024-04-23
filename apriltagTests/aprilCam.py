#!/bin/python3.6
import time
import cv2
import socket
import pickle
import struct
import numpy as np
from dt_apriltags import Detector


server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    server_socket.bind(('0.0.0.0', 5555))
    port = "5555"
except OSError as e:
    server_socket.bind(('0.0.0.0', 5565))
    port = "5565"
server_socket.listen(5)
print(f"Server is listening on port {port}...")
client_socket, client_address = server_socket.accept()
print(f"Connection from {client_address} accepted")

# Create a GStreamer pipeline for the CSI camera
video_source=0
gst_str = f'nvarguscamerasrc sensor-id={video_source} ! video/x-raw(memory:NVMM), width=480, height=360,\
                format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, \
                format=(string)BGRx! videoconvert ! video/x-raw, format=(string)BGR ! appsink'
cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

camera_calibration = np.load("../calibration/calibration.npy")
mtx, dist, rvecs, tvecs = camera_calibration
fx, fy, cx, cy = mtx[0, 0], mtx[1, 1], mtx[0, 2], mtx[1, 2]

detector = Detector(families='tag16h5', nthreads=4, quad_decimate=1.0, quad_sigma=0.0,\
                    refine_edges=1, decode_sharpening=0.25, debug=0)


# Set the frame rate
frame_rate = 10  # Desired frame rate for streaming
prev_time = time.time()

while True:
    ret, image = cap.read()

    # undistort
    h,  w = image.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    image = cv2.undistort(image, mtx, dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    image = image[y:y+h, x:x+w]

    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if ret == 0:
        continue
    
    #print(f"============== image shape: {image.shape}")
    tags = detector.detect(img, estimate_tag_pose=False, camera_params=[fx, fy, cx, cy], tag_size=0.03)
    realTags = [tag for tag in tags if (tag.decision_margin > 1) and (tag.tag_id == 0)]
    #print([tag.decision_margin for tag in realTags])

    # Check the time elapsed
    current_time = time.time()
    elapsed_time = current_time - prev_time
    
    if len(realTags) > 0:
        r = realTags[0]

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
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

        # draw the tag family on the image
        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(image, tagFamily, (ptA[0], ptA[1]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        #print(f"[INFO] tag family: {tagFamily}")
        print(f"detected: {r.decision_margin:.2f} {int(1/elapsed_time)} FPS")
    else:
        print(f"XXX -- Nothing -- XXX: {int(1/elapsed_time)} FPS")

    prev_time = current_time
        
    """
    image = cv2.resize(image, (320, 240))
    frame_data = pickle.dumps(image)
    client_socket.sendall(struct.pack("Q", len(frame_data)))
    client_socket.sendall(frame_data)
    """
cap.release()
#client_socket.close()
