import cv2

def find_image_properties(image_path):
    # Read the image using cv2.imread()
    image = cv2.imread(image_path)

    # Check if the image is valid
    if image is None:
        return "Invalid image path"

    # Check the channel order to determine color format
    if image.shape[2] == 3 and (image[0, 0, 0] < image[0, 0, 2]):
        color_format = "BGR"
    elif image.shape[2] == 3 and (image[0, 0, 0] > image[0, 0, 2]):
        color_format = "RGB"
    else:
        color_format = "Unknown"

    # Check the shape of the image to determine the type
    height, width, channels = image.shape
    if channels == 1:
        image_type = "Grayscale"
    elif channels == 3:
        image_type = "RGB"
    elif channels == 4:
        image_type = "RGBA"
    else:
        image_type = "Unknown"

    return f"The color format is: {color_format}\nThe type of the image is: {image_type}"

# Replace 'path/to/your/image.jpg' with the actual path to your image file
image_path = 'captured_img/captured_image.jpg'
image_properties = find_image_properties(image_path)

print(image_properties)

