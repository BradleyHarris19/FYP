#!/bin/python3.6

import cv2
import socket
import pickle
import struct

# Function to send video frames over the network
def send_video_frames(video_source=0, host='localhost', port=5555):
    # Create a GStreamer pipeline for the CSI camera
    gst_str = f'nvarguscamerasrc sensor-id={video_source} ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'

    # Open a GStreamer pipeline
    cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

    # Create a socket to send data
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))

    try:
        while True:
            # Read a frame from the camera
            ret, frame = cap.read()

            # Serialize the frame
            data = pickle.dumps(frame)

            # Pack the frame size and data
            message = struct.pack("Q", len(data)) + data

            # Send the frame over the network
            client_socket.sendall(message)

    except KeyboardInterrupt:
        # Release resources on keyboard interrupt
        cap.release()
        client_socket.close()

if __name__ == "__main__":
    # Specify the host and port to send the video stream
    host_ip = '10.0.0.1'
    port_number = 5555

    # Start sending the video stream
    send_video_frames(video_source=0, host=host_ip, port=port_number)


