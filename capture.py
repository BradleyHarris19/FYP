#!/bin/python3.6

import cv2
import subprocess

def capture_and_save_image(file_name, width=640, height=480):
    # Open a connection to the default camera (usually 0)
    video_source=0
    gst_str = f'nvarguscamerasrc sensor-id={video_source} ! video/x-raw(memory:NVMM), width={width}, height={height},\
                format=(string)NV12, framerate=(fraction)10/1 ! nvvidconv ! video/x-raw, \
                format=(string)GRAY8 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'
    cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
    
    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Capture a frame
    ret, frame = cap.read()

    # Release the camera
    cap.release()

    # Check if the frame was captured successfully
    if not ret:
        print("Error: Could not capture frame.")
        return

    # Save the frame as a JPG image
    cv2.imwrite(file_name, frame)
    print(f"Image saved as {file_name}")
    
    scp_command = [ 'scp', file_name, 'bradley@10.0.0.1:/home/bradley/Downloads']
    try:
        subprocess.run(scp_command, check=True)
        print("File successfully transferred via SCP!")
    except subprocess.CalledProcessError as e:
        print(f"Error transferring file via SCP: {e}")

num = 2
# Specify the file name to save the image
file_name = f"captured_img/captured_image{num}.jpg"

# Capture and save the image
capture_and_save_image(file_name)