#!/bin/python3.6

from Gamepad import Gamepad
import time
from jetbot import Robot, Camera, bgr8_to_jpeg
import argparse

# Gamepad settings
gamepadType = Gamepad.PG9099
buttonExit = 'Y'
speedUp = 'RB'
speedDown = 'LB'
joystickSpeed = 'LAS -Y'
joystickSteering = 'LAS -X'
pollInterval = 0.01

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
        left_speed = self.forward + (self.steering * 0.75)
        right_speed = self.forward - (self.steering * 0.75)

        # Scale the speeds by the overall speed scaler
        left_speed *= self.speed
        right_speed *= self.speed

        # Ensure motor speeds stay between 0 and 1
        left_speed = max(-1, min(1, left_speed))
        right_speed = max(-1, min(1, right_speed))

        # write to the motors
        self.robot.left_motor.value = left_speed
        self.robot.right_motor.value = right_speed


def main(baseSpeed):
    robot = Robot()
    driver = drive(robot, baseSpeed)

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
    
    try:
        while driver.running and gamepad.isConnected():
            start_time = time.time()

            #update stick positions
            driver.forward = -gamepad.axis(joystickSpeed)
            driver.steering = gamepad.axis(joystickSteering)
            driver.write()

            # Adjust your robot control logic based on the speed argument
            time.sleep(pollInterval)
            
            end_time = time.time()
            execution_time = end_time - start_time
            
            # Your robot control logic here
            print(f"Left Motor: {robot.left_motor.value:.2f}, Right Motor: {robot.right_motor.value:.2f}, Speed: {driver.speed:.2f}, Loop Speed: {execution_time:.3f} = {int(1/execution_time)}FPS")
    
    finally:
        robot.stop()
        gamepad.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robot Teleoperation with Keyboard")
    parser.add_argument("--speed", type=float, default=0.5, help="Robot speed factor")
    args = parser.parse_args()
    main(args.speed)
