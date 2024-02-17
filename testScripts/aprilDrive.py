import time
import argparse
import cv2
import numpy as np
import socket
import pickle
import struct
from Gamepad import Gamepad
from jetbot import Robot, Camera, bgr8_to_jpeg
from dt_apriltags import Detector

# Gamepad settings
gamepadType = Gamepad.PG9099
buttonExit = 'Y'
speedUp = 'RB'
speedDown = 'LB'
joystickSpeed = 'LAS -Y'
joystickSteering = 'LAS -X'
pollInterval = 0.01

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

def drawCorners(img, tag):
    # extract the bounding box (x, y)-coordinates for the AprilTag # and convert each of the (x, y)-coordinate pairs to integers 
    (ptA, ptB, ptC, ptD) = tag.corners
    ptB = (int(ptB[0]), int(ptB [1])) 
    ptC = (int(ptC[0]), int(ptC [1])) 
    ptD = (int(ptD[0]), int(ptD [1])) 
    ptA = (int(ptA[0]), int(ptA [1]))
    # draw the bounding box of the AprilTag detection 
    cv2.line(img, ptA, ptB, (255, 255,   0), 1) 
    cv2.line(img, ptB, ptC, (255,   0,   0), 1) 
    cv2.line(img, ptC, ptD, (  0, 255,   0), 1) 
    cv2.line(img, ptD, ptA, (  0,   0, 255), 1)
    return img

def drawCenter(img, tag):
    (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
    cv2.circle(img, (cX, cY), 2, (255, 0, 255), -1)
    return img

def drawName(img, tag, corners):
    # draw the tag family on the image
    tagFamily = tag.tag_family.decode("utf-8")
    cv2.putText(img, tagFamily, (corners[0][0], corners[0][1]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return img, tagFamily


class Camera(object):
    def __init__(self, source:int=0, resolution:tuple=(1280, 720), fps:int=(30), stream:bool=False):
        # Create a GStreamer pipeline for the CSI camera
        gst_str = f'nvarguscamerasrc sensor-id={source} ! video/x-raw(memory:NVMM), width={resolution[0]}, height={resolution[1]},\
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
        


class drive(object):
    def __init__(self, robot, inSpeed):
        self.robot = robot
        self.running = True
        self.speed = inSpeed
        self.forward = 0.0
        self.steering = 0.0

    #Create some callback functions for single events
    def exitButtonPressed(self):
        print('EXIT')
        self.running = False
    def speedUpPressed(self):
        self.speed = min(1.0, self.speed + 0.1)# Increase speed by 0.1,but ensure it doesn't go above 1
        print(f"Speed: {self.speed}")
    def speedDownPressed(self):
        self.speed = max(0, self.speed - 0.1)  # Decrease speed by 0.1, but ensure it doesn't go below 0
        print(f"Speed: {self.speed}")
    
    def write(self):
        # Calculate left and right motor speeds based on steering and forward values
        left_speed = self.forward + (self.steering * 0.75) #+ 0.2
        right_speed = self.forward - (self.steering * 0.75) #+ 0.2

        # Scale the speeds by the overall speed scaler
        left_speed *= self.speed
        right_speed *= self.speed

        # Ensure motor speeds stay between 0 and 1
        left_speed = max(-1, min(1, left_speed))
        right_speed = max(-1, min(1, right_speed))

        # write to the motors
        self.robot.left_motor.value = left_speed
        self.robot.right_motor.value = right_speed
        #print(f"Left Motor: {left_speed:.2f}, Right Motor: {right_speed:.2f}, Speed: {self.speed:.2f}")
        #, Loop Speed: {execution_time:.3f} = {int(1/execution_time)}FPS")

def distance(p1:tuple, p2:tuple):
    return np.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

def steering(center:np.array, resolution:tuple):
        posx = (center[0] - (resolution[0]/2)) / resolution[0]
        rotate = round(posx, 3)
        return rotate
        
def forward(corners:np.array, ta:int):
    (ptA, ptB, ptC, ptD) = corners
    cd = distance(ptC, ptD)
    vel = (cd - ta)/-ta
    return vel


def main(baseSpeed, stream):
    robot = Robot()
    driver = drive(robot, baseSpeed)
    resolution = (640, 480)

    #wait for gamepad connection
    if not Gamepad.available():
        print('Please connect your gamepad...')
        while not Gamepad.available():
            time.sleep(1.0)
    gamepad = gamepadType()
    print('Gamepad connected')

    # Start the background updating
    gamepad.startBackgroundUpdates()
    # Register the callback functions
    gamepad.addButtonPressedHandler(buttonExit, driver.exitButtonPressed)
    gamepad.addButtonPressedHandler(speedUp, driver.speedUpPressed)
    gamepad.addButtonPressedHandler(speedDown, driver.speedDownPressed)
    
    detector = Detector(families='tag16h5', nthreads=3, quad_decimate=1.0, quad_sigma=0.0,\
           refine_edges=1, decode_sharpening=0.25, debug=0)
    cam = Camera(0, resolution, 30, stream)
    fps = 0
    execution_time = 0
    detected = False

    try:
        while driver.running and gamepad.isConnected():
            start_time = time.time()
            ret, image = cam.read()
            if (ret == False): continue
            gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            tags = detector.detect(gray_img)
            realTags = [tag for tag in tags if (tag.decision_margin > 15) and (tag.tag_id == 0)] 
            #(distance(tag.corners[0], tag.corners[1]) > 15)]
            #print([tag.decision_margin for tag in realTags])
            if len(realTags) > 0:
                #print("detected")
                detected = True
                tag = realTags[0]
                #print(distance(tag.corners[0], tag.corners[1]))
                corners = tag.corners.astype(int)
                center = tag.center.astype(int)
                if stream:
                    image = drawCorners(image, tag)
                    image = drawCenter(image, tag)
                    image, tagfamily = drawName(image, tag, corners)
                #print(f"Tag position  X: {center[0]}, Y: {center[1]}")

                rot = steering(center, resolution)
                fwd = forward(corners, 50)
                #print(f"forward: {fwd:.2f}, Rotation: {rot:.2f}")

                if stream:
                    cv2.putText(image, f"{rot} == {fwd}", (0, 0), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                
                driver.forward = fwd
                driver.steering = rot
            else:
                driver.forward = 0
                driver.steering = 0
                #print("nothing")
                detected = False

            driver.write()
            
            if stream: 
                cam.stream(image)
            
            end_time = time.time()
            # Adjust your robot control logic based on the speed argument
            time_taken = end_time - start_time
            try: time.sleep(pollInterval - time_taken)
            except: pass
            execution_time = time_taken
            fps = int(1/execution_time)

            print(f"Left Motor: {robot.left_motor.value:.2f}, Right Motor: {robot.right_motor.value:.2f}, Speed: {driver.speed:.2f}, Loop Speed: {execution_time:.3f} = {int(1/execution_time)}FPS, Detected: {detected}")

    finally:
        robot.stop()
        gamepad.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robot Teleoperation with Keyboard")
    parser.add_argument("--speed", type=float, default=0.5, help="Robot speed factor")
    parser.add_argument("--stream", type=bool, default=False, help="stream video over port 5555/5565")
    args = parser.parse_args()
    main(args.speed, args.stream)