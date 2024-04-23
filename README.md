# Leader follower movement control
This project aims to emulate the Leader-Follower paradigm by developing and merging a variety of vision and movement control algorithms implemented on a group of mobile robots. The hardware used comprised two Waveshare Jetbots based on the NVIDIA Jetson Nano and a Raspberry Pi. For the vision system, Apriltags were utilised as real-world markers to track the Leader, whilst the Follower kept the tag at a set distance and in the centre of the camera's field of view. To achieve this an initial architecture based on the ROS robotics software framework was proposed but due to unforeseen issues, a pivot to a Python-based approach was needed. The produced system allowed the Follower to follow effectively without the need for any supporting data to be passed from the Leader. However, the system suffered from some mechanical limitations that caused poor performance on high-friction surfaces. 

## Archetecture
![](resources/FPY%20architecture.png)

## Report
![](resources/FYP%20Report.pdf)
