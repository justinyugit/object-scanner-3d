import os
from PIL import Image
import cv2
import numpy as np

# Specify the input and output directories
input_dir = 'depth2'
output_dir = 'depth'

# Create the output directory if it doesn't exist
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Get a list of all PNG files in the input directory
png_files = [f for f in os.listdir(input_dir) if f.endswith('.png')]

# Process each PNG file
for filename in png_files:
    # Open the image file
    image_path = os.path.join(input_dir, filename)
    

    # Divide each element by 85
    pixels = cv2.imread(image_path)
    new_pixels = np.divide(pixels, 85)
    new_pixels.astype(int)
    
    #new_pixels = [(int(r / 85), int(g / 85), int(b / 85)) for r, g, b in pixels]

    # Create a new image with the modified pixels
    
    
    output_path = os.path.join(output_dir, filename)
    
    cv2.imwrite(output_path, new_pixels)
    print(new_pixels.shape)

print("All PNG files processed.")
