#!/bin/python3
import cv2
import os

# Save all of the frames from the video file to the output folder as jpg images

def save_frames(input_file, output_folder):
    video_capture = cv2.VideoCapture(input_file)

    if not video_capture.isOpened():
        print("Error: Unable to open video file.")
        return

    os.makedirs(output_folder, exist_ok=True)

    frame_count = 0
    while True:
        ret, frame = video_capture.read()
        if not ret:
            break

        frame_count += 1
        frame_filename = f"{output_folder}/frame_{frame_count:04d}.jpg"
        cv2.imwrite(frame_filename, frame)

    video_capture.release()

if __name__ == "__main__":
    input_file_path = "april_video.mp4"
    output_folder_path = "../"

    save_frames(input_file_path, output_folder_path)
