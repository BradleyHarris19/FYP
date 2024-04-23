#!/bin/python3.6
import time 
import cv2
import numpy as np
from jetbot import Robot, Camera, bgr8_to_jpeg

# The first test to drive the motors on the Jetbot

camera = Camera()
robot = Robot()
cv2.cuda.setDevice(0)

#robot.left_motor.value = max(min(speed_slider.value + steering_slider.value, 1.0), 0.0)
#robot.right_motor.value = max(min(speed_slider.value - steering_slider.value, 1.0), 0.0)
 
print('forward')
robot.right_motor.value = 1
robot.left_motor.value = 1
time.sleep(0.5)
robot.right_motor.value = 0
robot.left_motor.value = 0
print('stop')

time.sleep(0.1)
camera.stop()
robot.stop()
