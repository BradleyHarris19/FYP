from drive import Drive
from camera import Camera
from draw import *
from jetbot import Robot
import numpy as np
import time
import argparse
import cv2

POLLINTERVAL = 0.01

def main(baseSpeed, stream, p, i, d):
    robot = Robot()
    driver = Drive(robot, baseSpeed)
    resolution = (480, 360)
    
    cam = Camera(0, resolution, 30, stream)
    execution_time = 0

    cv2.cuda.setDevice(0)
    in_image = cv2.cuda_GpuMat()
    out_image = cv2.cuda_GpuMat()

    angle_last = 0.0
    slice_height = 10
    thresh = 65
    view_distance = 70 #distance between camera and slice
    kp = p
    kd = d

    try:
        while driver.running:
            start_time = time.time()
            ret, image = cam.read()
            if (ret == False): continue

            # Get x bottom rows of the image
            height, width, _ = image.shape
            bottom_rows = image[height - slice_height:, :]
            height, width, _ = bottom_rows.shape
            # Convert to grayscale
            grey_bottom_rows = cv2.cvtColor(bottom_rows, cv2.COLOR_BGR2GRAY)
            # get vertical mean of the bottom rows
            grey_bottom_row = np.mean(grey_bottom_rows, 0, keepdims=True).astype(np.uint8)


            #thresh_grey_bottom_row = cv2.adaptiveThreshold(grey_bottom_row, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 5) 
            _, thresh_grey_bottom_row = cv2.threshold(grey_bottom_row, thresh, 255, cv2.THRESH_BINARY_INV)
            #thresh_grey_bottom_row = np.zeros_like(grey_bottom_row)
            #cv2.cuda.threshold(grey_bottom_row, thresh_grey_bottom_row, thresh, 255, cv2.THRESH_BINARY_INV)
            
            # pick out x coord of line
            positions = np.where(thresh_grey_bottom_row == 255)
            line_center = np.mean(positions[1]).astype(int)

            # calculate angle
            line_drift = line_center - (width/2) 
            if line_center != 0:
                angle = np.arctan2(line_drift, view_distance)

            # calculate steering using pd controller
            rot = angle * kp + (angle - angle_last) * kd
            angle_last = angle

            driver.forward = 0.2
            driver.steering = rot
            driver.write()

            if stream:
                space = np.ones((10, bottom_rows.shape[1], 3), dtype=np.uint8) * 255
                thresh_grey_bottom_row = np.tile(thresh_grey_bottom_row, (10, 1))
                thresh_grey_bottom_row = cv2.cvtColor(thresh_grey_bottom_row, cv2.COLOR_GRAY2RGB)
                cv2.line(thresh_grey_bottom_row, (line_center, 0), (line_center, height), (0, 255, 0), 3)
                result = np.concatenate((bottom_rows, space, thresh_grey_bottom_row), axis=0)
                cam.stream(result)
            
            end_time = time.time()
            # Adjust your robot control logic based on the speed argument
            time_taken = end_time - start_time
            try: time.sleep(POLLINTERVAL - time_taken)
            except: pass
            execution_time = time_taken
            fps = int(1/execution_time)

            print(f"Left Motor: {robot.left_motor.value:.2f}, Right Motor: {robot.right_motor.value:.2f}, Speed: {driver.speed:.2f}, Loop Speed: {execution_time:.3f} = {int(1/execution_time)}FPS")

    finally:
        robot.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robot Teleoperation with Keyboard")
    parser.add_argument("--speed", type=float, default=1.0, help="Robot speed factor")
    parser.add_argument("--stream", type=bool, default=False, help="stream video over port 5555/5565")
    parser.add_argument("--P", type=float, default=0.085, help="Potential tuning")
    parser.add_argument("--I", type=float, default=0, help="Intergral tuning")
    parser.add_argument("--D", type=float, default=0.12, help="Differential tuning")
    
    args = parser.parse_args()
    main(args.speed, args.stream, args.P, args.I, args.D)
