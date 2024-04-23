#!/bin/python3.6
"""
This file contains the implementation of a robot that follows an AprilTag using PID control.
The robot uses a camera to detect the AprilTag and adjusts its steering and velocity to track the tag.

Classes:
- Steering: Implements PID control for steering.
- Velocity: Implements PID control for velocity.

Functions:
- main: The main function that initializes the robot and controls its movement.

Usage:
- Run this file with the appropriate command line arguments to adjust the robot's behavior.
"""

from drive import Drive
from camera import Camera
from draw import *
from jetbot import Robot
from dt_apriltags import Detector
import numpy as np
import time
import argparse
import cv2
import os
import paho.mqtt.publish as publish

bot = os.environ.get('jetbot')
mqttBroker = "10.0.0.1"
POLLINTERVAL = 0.01

class Steering:
    def __init__(self, kp:float, ki:float, kd:float, setpoint:float):
        self.kp = kp #
        self.ki = ki
        self.kd = kd
        publish.single(f"{bot}/steering/pid/P", kp, hostname=mqttBroker)
        publish.single(f"{bot}/steering/pid/I", ki, hostname=mqttBroker)
        publish.single(f"{bot}/steering/pid/D", kd, hostname=mqttBroker)
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def reset(self):
        self.prev_error = 0
        self.integral = 0

    def __call__(self, current_value:float):
        publish.single(f"{bot}/steering/pid/in", current_value, hostname=mqttBroker)
        error = self.setpoint - current_value
        publish.single(f"{bot}/steering/pid/error", error, hostname=mqttBroker)
        # Proportional term -- The difference between set point and current value
        p_term = self.kp * error
        publish.single(f"{bot}/steering/pid/P_term", p_term, hostname=mqttBroker)
        # Integral term -- error over time, if error does not close it increases
        self.integral += error
        i_term = self.ki * self.integral
        publish.single(f"{bot}/steering/pid/I_term", i_term, hostname=mqttBroker)
        # Derivative term -- tames the compounding nature of the other variables if increse is rapid
        d_term = self.kd * (error - self.prev_error)
        publish.single(f"{bot}/steering/pid/D_term", d_term, hostname=mqttBroker)
        # PID control output
        output = p_term + i_term + d_term
        # Update previous error for the next iteration
        self.prev_error = error

        publish.single(f"{bot}/steering/pid/out", -output, hostname=mqttBroker)
        return -output
        
class Velocity:
    def __init__(self, kp:float, ki:float, kd:float, setpoint:int):
        self.kp = kp #
        self.ki = ki
        self.kd = kd
        publish.single(f"{bot}/velocity/pid/P", kp, hostname=mqttBroker)
        publish.single(f"{bot}/velocity/pid/I", ki, hostname=mqttBroker)
        publish.single(f"{bot}/velocity/pid/D", kd, hostname=mqttBroker)
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def __call__(self, current_value:int):
        publish.single(f"{bot}/velocity/pid/in", current_value, hostname=mqttBroker)
        error = self.setpoint - current_value
        publish.single(f"{bot}/velocity/pid/error", error, hostname=mqttBroker)
        # Proportional term -- The difference between set point and current value
        p_term = self.kp * error
        publish.single(f"{bot}/velocity/pid/P_term", p_term, hostname=mqttBroker)
        # Integral term -- error over time, if error does not close it increases
        self.integral += error
        i_term = self.ki * self.integral
        publish.single(f"{bot}/velocity/pid/I_term", i_term, hostname=mqttBroker)
        # Derivative term -- tames the compounding nature of the other variables if increse is rapid
        d_term = self.kd * (error - self.prev_error)
        publish.single(f"{bot}/velocity/pid/D_term", d_term, hostname=mqttBroker)
        # PID control output
        output = p_term + i_term + d_term
        # Update previous error for the next iteration
        self.prev_error = error
        
        output = max(0, output)
        publish.single(f"{bot}/velocity/pid/out", output, hostname=mqttBroker)
        return output

def main(baseSpeed, tagid, stream, p, i, d):
    robot = Robot()
    driver = Drive(robot, baseSpeed)
    resolution = (480, 360)

    camera_calibration = np.load("calibration/calibration.npy")
    mtx, dist, rvecs, tvecs = camera_calibration
    fx, fy, cx, cy = mtx[0, 0], mtx[1, 1], mtx[0, 2], mtx[1, 2]
    
    detector = Detector(families='tag16h5', nthreads=4, quad_decimate=1.0, quad_sigma=0.0,\
                        refine_edges=1, decode_sharpening=0.25, debug=0)
    cam = Camera(0, resolution, 30, stream)
    execution_time = 0
    detected = False
    
    # 0.25, 0.01, 0.01
    # 0.1, 0.0025, 0.2
    # 0.08. 0.015, 0.2
    # 0.07, 0.005, 0.2
    steering = Steering(0.07, 0.0, 0.2, 0)
    forward = Velocity(0.05, 0, 0, 30)

    try:
        while driver.running:
            start_time = time.time()
            ret, image = cam.read()
            if (ret == False): continue
            
            gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            tags = detector.detect(gray_img, estimate_tag_pose=False, camera_params=[fx, fy, cx, cy], tag_size=0.03)
            realTags = [tag for tag in tags if (tag.decision_margin > 1) and (tag.tag_id == tagid)] 
            
            if len(realTags) > 0:
                detected = True
                tag = realTags[0]
                corners = tag.corners.astype(int)
                center = tag.center.astype(int)
                if stream:
                    image = drawCorners(image, tag)
                    image = drawCenter(image, tag)
                    image, tagfamily = drawName(image, tag, corners)
                
                size = distance(tag.corners[2], tag.corners[3])
                #fwd = forward(size)
                hor_pos = 2*(float(center[0]) / float(resolution[0])) - 1.0
                rot = steering(hor_pos)

                driver.forward = min(0.2, (2/size))  
                #driver.forward = 0.2
                driver.steering = rot
            else:
                steering.reset()
                driver.forward = 0
                driver.steering = 0
                #print("--- nothing ---")
                detected = False

            driver.write()
            
            if stream: 
                cam.stream(image)
            
            end_time = time.time()
            # Adjust your robot control logic based on the speed argument
            time_taken = end_time - start_time
            try: time.sleep(POLLINTERVAL - time_taken)
            except: pass
            execution_time = time_taken
            fps = int(1/execution_time)

            #print(f"Left Motor: {robot.left_motor.value:.2f}, Right Motor: {robot.right_motor.value:.2f}, Speed: {driver.speed:.2f}, Loop Speed: {execution_time:.3f} = {int(1/execution_time)}FPS, Detected: {detected}")

    finally:
        robot.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robot Teleoperation with Keyboard")
    parser.add_argument("--speed", type=float, default=1.0, help="Robot speed factor")
    parser.add_argument("--tagid", type=int, default=1, help="AprilTag ID to follow")
    parser.add_argument("--stream", type=bool, default=False, help="stream video over port 5555/5565")
    parser.add_argument("--P", type=float, default=0.1, help="Potential tuning")
    parser.add_argument("--I", type=float, default=0.0025, help="Intergral tuning")
    parser.add_argument("--D", type=float, default=0.2, help="Differential tuning")
    args = parser.parse_args()
    main(args.speed, args.tagid-1, args.stream, args.P, args.I, args.D)
