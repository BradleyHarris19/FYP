#!/bin/python3.6

import time
import cv2
import numpy as np
import traitlets
from jetbot import Robot, Camera, bgr8_to_jpeg
import keyboard
import threading
import argparse

def on_key_event(e, robot, speed, exit_flag):
    if e.event_type == keyboard.KEY_DOWN:
        if e.name == 'w':
            robot.left_motor.value = speed
            robot.right_motor.value = speed
            print("going forward")
        elif e.name == 's':
            robot.left_motor.value = -speed
            robot.right_motor.value = -speed
        elif e.name == 'a':
            robot.left_motor.value = -speed
            robot.right_motor.value = speed
        elif e.name == 'd':
            robot.left_motor.value = speed
            robot.right_motor.value = -speed
        elif e.name == 'q':
            speed = min(1.0, speed + 0.1)  # Increase speed by 0.1
        elif e.name == 'e':
            speed = max(0, speed - 0.1)  # Decrease speed by 0.1, but ensure it doesn't go below 0
    elif e.event_type == keyboard.KEY_UP:
        robot.left_motor.value = 0
        robot.right_motor.value = 0
    elif e.event_type == keyboard.KEY_DOWN and e.name == 'esc':
        exit_flag.set()

def keyboard_thread(robot, speed):
    keyboard.hook(lambda e: on_key_event(e, robot, speed))
    keyboard.wait("esc")  # Wait for the 'esc' key to exit the thread

def main(speed):
    robot = Robot()
    exit_flag = threading.Event()
    
    # Start the keyboard thread
    #keyboard_thread_instance = threading.Thread(target=keyboard_thread, args=(robot, speed))
    #keyboard_thread_instance.start()
    keyboard.hook(lambda e: on_key_event(e, robot, speed))

    while not exit_flag.is_set():
        # Your robot control logic here
        print(f"Left Motor: {robot.left_motor.value}, Right Motor: {robot.right_motor.value}, Speed: {speed}")
        # Adjust your robot control logic based on the speed argument
        time.sleep(0.1)

    robot.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robot Teleoperation with Keyboard")
    parser.add_argument("--speed", type=float, default=0.5, help="Robot speed factor")
    args = parser.parse_args()
    main(args.speed)
