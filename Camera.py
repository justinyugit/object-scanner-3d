import pyrealsense2 as rs
import numpy as np
import cv2

class Camera:

    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        profile = self.pipeline.start(config)
        p = profile.get_stream(rs.stream.color)
        self.intr = p.as_video_stream_profile().get_intrinsics()
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    def get_frames(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        return depth_image / self.depth_scale, color_image

    def stop(self):
        # Stop streaming
        self.pipeline.stop()
    
    def get_intrinsic(self):
        return np.array([[self.intr.fx, 0, self.intr.ppx], [0, self.intr.fy, self.intr.ppy], [0,0,1]])

# Example usage
if __name__ == "__main__":
    camera = Camera()

    try:
        while True:
            depth_image, color_image = camera.get_frames()

            # Display the images
            cv2.imshow('Depth Image', depth_image)
            cv2.imshow('Color Image', color_image)

            # Exit if the user presses the 'ESC' key
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        camera.stop()
        cv2.destroyAllWindows()