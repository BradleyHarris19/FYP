import cv2
import numpy as np
from dt_apriltags import Detector 


imagepath = 'captured_img/captured_image.jpg'
image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)

detector = Detector(families='tag16h5',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)


tags = detector.detect(image)

print(tags[0])
