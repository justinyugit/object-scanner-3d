import cv2
import os

# Path to the input MP4 file
input_file = "depth.mp4"

# Output folder to save PNG frames
output_folder = "DATA/depth/"

# Create the output folder if it doesn't exist
os.makedirs(output_folder, exist_ok=True)

# Open the video file
video = cv2.VideoCapture(input_file)

# Initialize frame count
frame_count = 0

# Read the video frames
while True:
    # Read a single frame from the video
    ret, frame = video.read()

    # Break if the frame is not successfully read
    if not ret:
        break

    # Save the frame as a PNG image
    output_file = os.path.join(output_folder, f"{frame_count:07d}.png")
    cv2.imwrite(output_file, frame)

    # Increment frame count
    frame_count += 1

    # Display progress
    print(f"Frame {frame_count:07d} saved")

# Release the video capture object
video.release()

print("All frames extracted and saved as PNGs.")