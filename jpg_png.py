import os
from PIL import Image

# Folder containing the JPEG images
input_folder = "DATA/rgb2"

# Output folder to save the converted PNG images
output_folder = "DATA/rgb"

# Create the output folder if it doesn't exist
os.makedirs(output_folder, exist_ok=True)

# Get a list of all files in the input folder
file_list = os.listdir(input_folder)

# Iterate over each file in the input folder
for filename in file_list:
    # Check if the file is a JPEG image
    if filename.lower().endswith(".jpg") or filename.lower().endswith(".jpeg"):
        # Open the JPEG image
        input_file = os.path.join(input_folder, filename)
        image = Image.open(input_file)

        # Generate the output PNG filename
        output_file = os.path.join(output_folder, os.path.splitext(filename)[0] + ".png")

        # Convert and save the image as PNG
        image.save(output_file, "PNG")
        print(f"Converted {filename} to PNG")

print("All JPEG images converted to PNG.")