#!/bin/python3.6
from Gamepad import Gamepad
import time
from jetbot import Robot, Camera, bgr8_to_jpeg
import argparse

# Gamepad settings
gamepadType = Gamepad.PG9099
buttonExit = 'Y'
right_speedUp = 'RB'
right_speedDown = 'LB'
left_speedUp = 'B'
left_speedDown = 'X'
pollInterval = 0.01

class drive(object):
    def __init__(self, robot, inSpeed):
        self.robot = robot
        self.running = True
        self.speed = inSpeed
        self.left_speed = 0.1
        self.right_speed = 0.1

    #Create some callback functions for single events
    def exitButtonPressed(self):
        print('EXIT')
        self.running = False

    def left_speedUpPressed(self):
        self.left_speed = min(1.0, self.left_speed + 0.01)# Increase speed by 0.1,but ensure it doesn't go above 1
    def left_speedDownPressed(self):
        self.left_speed = max(0, self.left_speed - 0.01)  # Decrease speed by 0.1, but ensure it doesn't go below 0

    def right_speedUpPressed(self):
        self.right_speed = min(1.0, self.right_speed + 0.01)# Increase speed by 0.1,but ensure it doesn't go above 1
    def right_speedDownPressed(self):
        self.right_speed = max(0, self.right_speed - 0.01)  # Decrease speed by 0.1, but ensure it doesn't go below 0
    
    def write(self):
        # write to the motors
        self.robot.left_motor.value = self.left_speed
        self.robot.right_motor.value = self.right_speed


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
    gamepad.addButtonPressedHandler(left_speedUp, driver.left_speedUpPressed)
    gamepad.addButtonPressedHandler(left_speedDown, driver.left_speedDownPressed)
    gamepad.addButtonPressedHandler(right_speedUp, driver.right_speedUpPressed)
    gamepad.addButtonPressedHandler(right_speedDown, driver.right_speedDownPressed)
    
    try:
        while driver.running and gamepad.isConnected():
            start_time = time.time()

            # Adjust your robot control logic based on the speed argument
            time.sleep(pollInterval)
            
            end_time = time.time()
            execution_time = end_time - start_time
            
            driver.write()

            # Your robot control logic here
            print(f"Left Motor: {robot.left_motor.value:.3f}, Right Motor: {robot.right_motor.value:.3f}, Speed: {driver.speed:.2f}, Loop Speed: {execution_time:.3f} = {int(1/execution_time)}FPS")
    
    finally:
        robot.stop()
        gamepad.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robot Teleoperation with Keyboard")
    parser.add_argument("--speed", type=float, default=0.5, help="Robot speed factor")
    args = parser.parse_args()
    main(args.speed)

