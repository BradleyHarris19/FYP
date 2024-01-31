#!/bin/python3.6

from Gamepad import Gamepad
import time
from jetbot import Robot, Camera, bgr8_to_jpeg
import argparse



# Gamepad settings
gamepadType = Gamepad.PG9099
buttonExit = 'Y'
speedUp = 'LB'
speedDown = 'RB'
joystickSpeed = 'LAS -Y'
joystickSteering = 'LAS -X'
pollInterval = 0.1

speed = 0.5
running = False

def main(inSpeed):
    robot = Robot()
    # Set some initial state
    running = True
    speed = inSpeed
    steering = 0.0

    #wait for gamepad connection
    if not Gamepad.available():
        print('Please connect your gamepad...')
        while not Gamepad.available():
            time.sleep(1.0)
    gamepad = gamepadType()
    print('Gamepad connected')
    
    #Create some callback functions for single events
    def exitButtonPressed():
        print('EXIT')
        running = False
    def speedUpPressed():
        speed = min(1.0, speed + 0.1)# Increase speed by 0.1,but ensure it doesn't go above 1
    def speedDownPressed():
        speed = max(0, speed - 0.1)  # Decrease speed by 0.1, but ensure it doesn't go below 0
    def joystickSpeedPressed():
        if gamepad.axis(joystickSpeed) > 0:
            robot.left_motor.value = speed
            robot.right_motor.value = speed

        if gamepad.axis(joystickSpeed) < 0:
            robot.left_motor.value = -speed
            robot.right_motor.value = -speed

    def joystickSteeringPressed():
        if gamepad.axis(joystickSteering) > 0:
            robot.left_motor.value = -speed
            robot.right_motor.value = speed

        if gamepad.axis(joystickSteering) < 0:
            robot.left_motor.value = speed
            robot.right_motor.value = -speed



    # Start the background updating
    gamepad.startBackgroundUpdates()
    # Register the callback functions
    gamepad.addButtonPressedHandler(buttonExit, exitButtonPressed)
    gamepad.addButtonPressedHandler(speedUp, speedUpPressed)
    gamepad.addButtonPressedHandler(speedDown, speedDownPressed)
    #gamepad.addButtonPressedHandler(joystickSpeed, joystickSpeedPressed)
    #gamepad.addButtonPressedHandler(joystickSteerin, joystickSteeringPressed)
    
    while True:
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

