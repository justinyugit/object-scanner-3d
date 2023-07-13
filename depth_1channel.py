import os
from PIL import Image
import cv2
import numpy as np
np.set_printoptions(threshold=np.inf)

# Folder containing the JPEG images
input_folder = "DATA/masks2"

# Output folder to save the converted PNG images
output_folder = "DATA/masks"

# Create the output folder if it doesn't exist
os.makedirs(output_folder, exist_ok=True)

# Get a list of all files in the input folder
file_list = os.listdir(input_folder)

# Iterate over each file in the input folder
for filename in file_list:
    # Check if the file is a JPEG image
    input_file = os.path.join(input_folder, filename)
    ii = cv2.imread(input_file)

    gray_image = cv2.cvtColor(ii, cv2.COLOR_BGR2GRAY)

    # Generate the output PNG filename
    output_file = os.path.join(output_folder, os.path.splitext(filename)[0] + ".png")
    cv2.imwrite(output_file, gray_image)
    # Convert and save the image as PNG

