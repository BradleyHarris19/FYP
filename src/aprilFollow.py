from drive import Drive
from camera import Camera
from draw import *
from jetbot import Robot
from dt_apriltags import Detector
import numpy as np
import time
import argparse
import cv2

POLLINTERVAL = 0.01


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
    driver = Drive(robot, baseSpeed)
    resolution = (640, 480)
    
    detector = Detector(families='tag16h5', nthreads=3, quad_decimate=1.0, quad_sigma=0.0,\
           refine_edges=1, decode_sharpening=0.25, debug=0)
    cam = Camera(0, resolution, 30, stream)
    execution_time = 0
    detected = False

    try:
        while driver.running:
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
            try: time.sleep(POLLINTERVAL - time_taken)
            except: pass
            execution_time = time_taken
            fps = int(1/execution_time)

            print(f"Left Motor: {robot.left_motor.value:.2f}, Right Motor: {robot.right_motor.value:.2f}, Speed: {driver.speed:.2f}, Loop Speed: {execution_time:.3f} = {int(1/execution_time)}FPS, Detected: {detected}")

    finally:
        robot.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robot Teleoperation with Keyboard")
    parser.add_argument("--speed", type=float, default=0.5, help="Robot speed factor")
    parser.add_argument("--stream", type=bool, default=False, help="stream video over port 5555/5565")
    args = parser.parse_args()
    main(args.speed, args.stream)
