import numpy as np
import cv2
import socket
import pickle
import struct
import argparse

def startSocket():
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
    return client_socket, client_address

class Camera(object):
    def __init__(self, source:int=0, resolution:tuple=(1280, 720), fps:int=(30), stream:bool=False):
        # Create a GStreamer pipeline for the CSI camera
        gst_str = f'nvarguscamerasrc sensor-id={source} ! \
                    video/x-raw(memory:NVMM), width={resolution[0]}, height={resolution[1]},\
                    format=(string)NV12, framerate=(fraction){fps}/1 ! nvvidconv ! video/x-raw, \
                    format=(string)BGRx! videoconvert ! video/x-raw, format=(string)BGR ! appsink'
        self.capture = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        camera_calibration = np.load("calibration/calibration.npy")
        self.mtx, self.dist, _, _ = camera_calibration 
        if stream:
            self.clientConnection, self.clientAddress = startSocket()
          
    def undistort(self, img):
        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
        # undistort
        img = cv2.undistort(img, self.mtx, self.dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        img = img[y:y+h, x:x+w]

        return img

    def read(self):
        ret, img = self.capture.read()
        if ret == 0: return False, 0
        img = self.undistort(img)
        return True, img

    def stream(self, img, resize:tuple=(320, 240)):
        if hasattr(self, 'clientAddress') and self.clientAddress is not None:
            img = cv2.resize(img, resize)
            frame_data = pickle.dumps(img)
            self.clientConnection.sendall(struct.pack("Q", len(frame_data)))
            self.clientConnection.sendall(frame_data)
        else:
            print("streaming not enabled")
            return 0
        
    def __del__(self):
        self.capture.release()
        self.clientConnection.close()
        



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Starts camera and streams over ports 5555/5565")
    parser.add_argument("--resolution", type=tuple, default=(320, 240), help="Resolution to stream")
    args = parser.parse_args()
    
    cam = Camera(0, args.resolution, 30, True)

    while cam.clientAddress is not None:
        ret, image = cam.read()
        if (ret == False): continue
        cam.stream(image)
        
