import pyrealsense2 as rs
import cv2
import numpy as np
np.set_printoptions(threshold=np.inf)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

# Start the pipeline
pipeline.start(config)

# Create video writers
rgb_out = cv2.VideoWriter('rgb.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (640, 480))
depth_out = cv2.VideoWriter('depth.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (640, 480), isColor=False)

import time
time.sleep(5)
try:
    while True:
        # Wait for frames
        frames = pipeline.wait_for_frames()

        # Get the RGB and depth frames
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # Convert the RGB frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert the depth frame to a numpy array
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_min = np.min(depth_image)
        depth_max = np.max(depth_image)
        depth_norm = (depth_image-depth_min) / (depth_max - depth_min)
        depth_norm = depth_norm  * 255

        depth_norm = depth_norm.astype(np.uint8)

        
        
        # depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
        # Save RGB frame to video
        rgb_out.write(color_image)
        depth_out.write(depth_norm)

        # Save depth frame to video as PNG
        
        # cv2.imwrite('depth_frame.png', depth_png)

        # Show the frames
        cv2.imshow('RGB', color_image)
        cv2.imshow('Depth', depth_image)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) == ord('q'):
            break

finally:
    # Stop the pipeline
    pipeline.stop()

    # Release the video writers
    rgb_out.release()
    depth_out.release()

    # Close all OpenCV windows
    cv2.destroyAllWindows()




