import time 
import torchvision
import torch
import cv2
import PIL.Image
import numpy as np
from IPython.display import display
import ipywidgets
import traitlets
from jetbot import Robot, Camera, bgr8_to_jpeg

try:
    robot.stop()
    camera.stop()
except:
    pass

camera = Camera()
robot = Robot()
cv2.cuda.setDevice(0)

in_image = cv2.cuda_GpuMat()
out_image = cv2.cuda_GpuMat()

line_center = 0
angle = 0.0
angle_last = 0.0
frames = 0
total_time = 0 

time_slider = ipywidgets.FloatSlider(min=0, max=100, description='time (ms)')
min_time = ipywidgets.FloatText(value=0.0, description='Min time (ms)')
avg_time = ipywidgets.FloatText(value=0.0, description='Avg time (ms)')
max_time = ipywidgets.FloatText(value=5.0, description='Max time (ms)')

height_slider = ipywidgets.IntSlider(min=1, max=25, step=1, value=10, description='c')
threash_slider = ipywidgets.IntSlider(min=0, max=255, step=5, value=55, description='thresh')
block_slider = ipywidgets.IntSlider(min=0, max=100, step=3, value=61, description='block')

a_slider = ipywidgets.IntSlider(min=-110, max=110, description='center - line_center')
b_slider = ipywidgets.IntSlider(min=30, max=120, step=5, value=70, orientation='vertical', description='distance to camera')

angle_slider = ipywidgets.FloatSlider(min=-1.0, max=1.0, description='angle')

steering_slider = ipywidgets.FloatSlider(min=-1.0, max=1.0, description='steering')
speed_slider = ipywidgets.FloatSlider(min=0, max=1.0, description='speed')
steering_gain_slider = ipywidgets.FloatSlider(min=0.10, max=0.30, step=0.01, value=0.20, description='steering gain')
steering_diff_gain_slider = ipywidgets.FloatSlider(min=0.10, max=0.30, step=0.01, value=0.25, description='steering kd')

def line(value):
    global line_center, angle_last, angle, frames, total_time
    
    start_time = time.time()
    height, width, _ = value.shape
    bottom_rows = value[height - height_slider.value:, :]
    height, width, _ = bottom_rows.shape
    
    d_image.upload(image)
    grey_bottom_rows = cv2.cvtColor(bottom_rows, cv2.COLOR_BGR2GRAY)
    
    grey_bottom_row = np.mean(grey_bottom_rows, 0, keepdims=True).astype(np.uint8)
    
    #thresh_grey_bottom_row = cv2.adaptiveThreshold(grey_bottom_row, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 5) 
    #_, thresh_grey_bottom_row = cv2.threshold(grey_bottom_row, threash_slider.value, 255, cv2.THRESH_BINARY_INV)
    
    # Upload the image to the GPU
    in_image.upload(grey_bottom_row)
    # Create a CUDA thresholding operation
    threshold = cv2.cuda.createThresh_CV(threash_slider.value, 255, cv2.cuda.THRESH_BINARY_INV)
    # Apply thresholding on the GPU
    threshold.apply(in_image, out_image)
    # Download the thresholded image from the GPU
    thresh_grey_bottom_row = out_image.download()
    
    positions = np.where(thresh_grey_bottom_row == 255)
    line_center = np.mean(positions[1]).astype(int)
    
    a_slider.value = line_center - (width/2) 
    if line_center != 0:
        angle = np.arctan2(a_slider.value, b_slider.value)
        angle_slider.value = angle
    
    pd = angle * steering_gain_slider.value + (angle - angle_last) * steering_diff_gain_slider.value
    angle_last = angle
    
    steering_slider.value = pd
    
    robot.left_motor.value = max(min(speed_slider.value + steering_slider.value, 1.0), 0.0)
    robot.right_motor.value = max(min(speed_slider.value - steering_slider.value, 1.0), 0.0)
    
    
    cv2.line(thresh_grey_bottom_row, (line_center, 0), (line_center, height), (0, 255, 0), 1)
    
    scaled_image = cv2.resize(thresh_grey_bottom_row, (width * 4, height * 4))
    img = bgr8_to_jpeg(scaled_image)
    
    time_taken = (time.time() - start_time)*1000
    total_time += time_taken
    frames += 1
    avg_time.value = total_time/frames
    if (time_taken < min_time.value): min_time.value = time_taken
    if (time_taken > max_time.value): max_time.value = time_taken
    time_slider.value = time_taken
    return img


def line_colour(value):
    global line_center
    height, width, _ = value.shape
    bottom_rows = value[height - height_slider.value:, :]
    height, width, _ = bottom_rows.shape
    
    cv2.line(bottom_rows, (line_center, 0), (line_center, height), (0, 255, 0), 1)
    
    scaled_image = cv2.resize(bottom_rows, (width * 4, height * 4))
    img = bgr8_to_jpeg(scaled_image)
    return img


camera_widget = ipywidgets.Image()
camera_link = traitlets.dlink((camera, 'value'), (camera_widget, 'value'), transform=line_colour)

line_widget = ipywidgets.Image()
line_link = traitlets.dlink((camera, 'value'), (line_widget, 'value'), transform=line)

display(camera_widget, line_widget, time_slider, ipywidgets.HBox([min_time, avg_time, max_time]), height_slider, threash_slider, block_slider)

display(ipywidgets.HBox([a_slider, b_slider]))
display(steering_slider, speed_slider, steering_gain_slider, steering_diff_gain_slider)
