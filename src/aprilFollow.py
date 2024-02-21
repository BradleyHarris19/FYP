from drive import Drive
from camera import Camera
from draw import *
from jetbot import Robot
from dt_apriltags import Detector
import numpy as np
import time
import argparse
import cv2
import paho.mqtt.publish as publish

mqttBroker = "10.0.0.1"
POLLINTERVAL = 0.01

class Steering:
    def __init__(self, kp:float, ki:float, kd:float, setpoint:int):
        self.kp = kp #
        self.ki = ki
        self.kd = kd
        publish.single("jetbot1/steering/pid/P", kp, hostname=mqttBroker)
        publish.single("jetbot1/steering/pid/I", ki, hostname=mqttBroker)
        publish.single("jetbot1/steering/pid/D", kd, hostname=mqttBroker)
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def __call__(self, current_value:int):
        error = self.setpoint - current_value
        # Proportional term -- The difference between set point and current value
        p_term = self.kp * error
        publish.single("jetbot1/steering/pid/P_term", p_term, hostname=mqttBroker)
        # Integral term -- error over time, if error does not close it increases
        self.integral += error
        i_term = self.ki * self.integral
        publish.single("jetbot1/steering/pid/I_term", p_term, hostname=mqttBroker)
        # Derivative term -- tames the compounding nature of the other variables if increse is rapid
        d_term = self.kd * (error - self.prev_error)
        publish.single("jetbot1/steering/pid/D_term", p_term, hostname=mqttBroker)
        # PID control output
        output = p_term + i_term + d_term

        # Update previous error for the next iteration
        self.prev_error = error

        return -output
        
class Velocity:
    def __init__(self, kp:float, ki:float, kd:float, setpoint:int):
        self.kp = kp #
        self.ki = ki
        self.kd = kd
        publish.single("jetbot1/velocity/pid/P", kp, hostname=mqttBroker)
        publish.single("jetbot1/velocity/pid/I", ki, hostname=mqttBroker)
        publish.single("jetbot1/velocity/pid/D", kd, hostname=mqttBroker)
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def __call__(self, current_value:int):
        error = self.setpoint - current_value
        # Proportional term -- The difference between set point and current value
        p_term = self.kp * error
        publish.single("jetbot1/velocity/pid/P_term", p_term, hostname=mqttBroker)
        # Integral term -- error over time, if error does not close it increases
        self.integral += error
        i_term = self.ki * self.integral
        publish.single("jetbot1/velocity/pid/I_term", i_term, hostname=mqttBroker)
        # Derivative term -- tames the compounding nature of the other variables if increse is rapid
        d_term = self.kd * (error - self.prev_error)
        publish.single("jetbot1/velocity/pid/D_term", d_term, hostname=mqttBroker)
        # PID control output
        output = p_term + i_term + d_term
        # Update previous error for the next iteration
        self.prev_error = error

        return output

def main(baseSpeed, stream, p, i, d):
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

    steering = Steering(0.0018, 0.00005, 0.005, resolution[0]/2)
    forward = Velocity(p, i, d, 50)

    try:
        while driver.running:
            start_time = time.time()
            ret, image = cam.read()
            if (ret == False): continue
            
            gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            tags = detector.detect(gray_img, estimate_tag_pose=False, camera_params=[fx, fy, cx, cy], tag_size=0.03)
            realTags = [tag for tag in tags if (tag.decision_margin > 1) and (tag.tag_id == 0)] 
            #(distance(tag.corners[0], tag.corners[1]) > 15)]
            #print([tag.decision_margin for tag in realTags])
            
            if len(realTags) > 0:
                print("detected")
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
                
                fwd = forward(distance(tag.corners[2], tag.corners[3]))
                #rot = steering(center[0])
                #print(f"Rotation: {rot:.2f}")
                #print(f"forward: {fwd:.2f}, Rotation: {rot:.2f}")

                #if stream:
                    #cv2.putText(image, f"{rot} == {fwd}", (0, 0), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                
                driver.forward = fwd
                #driver.steering = rot
            else:
                driver.forward = 0
                driver.steering = 0
                print("--- nothing ---")
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
    parser.add_argument("--speed", type=float, default=0.5, help="Robot speed factor")
    parser.add_argument("--stream", type=bool, default=False, help="stream video over port 5555/5565")
    parser.add_argument("--P", type=float, default=2, help="Potential tuning")
    parser.add_argument("--I", type=float, default=0, help="Intergral tuning")
    parser.add_argument("--D", type=float, default=0, help="Differential tuning")
    args = parser.parse_args()
    main(args.speed, args.stream, args.P, args.I, args.D)
