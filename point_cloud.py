import Camera
import numpy as np
import pptk
import open3d as o3d
import time
import urx

while True:
    try:
        robot = urx.Robot("172.19.97.157")
        break
    except:
        time.sleep(2)
        continue




camera = Camera.Camera()
intr = camera.get_intrinsic()
fx = intr[0][0]
fy = intr[1][1]
cx = intr[0][2]
cy = intr[1][2]
R = np.load("R.npy")
T = np.squeeze(np.load("T.npy"))

eyeinhand_transform = np.vstack([np.hstack([R.T, -R.T @ T.reshape(3,1)]), [0,0,0,1]])


robot_t1 = np.array(list(robot.get_pos()))
robot_r1 = np.squeeze(list(robot.get_orientation()))
robot_transform = np.vstack([np.hstack([robot_r1, robot_t1.reshape(3, 1)]), [0, 0, 0, 1]])




depth_image, _ = camera.get_frames()
rows, cols = depth_image.shape[:2]
x, y = np.meshgrid(np.arange(cols), np.arange(rows))
x = (x - cx) * depth_image / fx
y = (y - cy) * depth_image / fy
z = depth_image.copy()
point_cloud1 = np.stack((x, y, z), axis=-1)

for point in point_cloud1:
    print(point)
    homogeneous_points = np.hstack((point, np.ones((len(point), 1))))
    homogeneous_points_world = np.dot(robot_r1, homogeneous_points.T).T
v = pptk.viewer(homogeneous_points_world[:,:3])
exit()


# depth_image, _ = camera.get_frames()
# rows, cols = depth_image.shape[:2]
# x, y = np.meshgrid(np.arange(cols), np.arange(rows))
# x = (x - cx) * depth_image / fx
# y = (y - cy) * depth_image / fy
# z = depth_image.copy()
# point_cloud1 = np.stack((x, y, z), axis=-1)
# print(point_cloud1.shape)
# point_cloud1 = np.reshape(point_cloud1, (3,-1))
# point_cloud_endeffector = np.linalg.inv(eyeinhand_transform) @ np.vstack([point_cloud1, np.ones((1, point_cloud1.shape[1]))])
# point_cloud_endeffector = point_cloud_endeffector[:3, :]
# point_cloud_endeffector = np.reshape(point_cloud_endeffector, (480, 640, 3))
# print(point_cloud_endeffector.shape)
# v = pptk.viewer(point_cloud_endeffector)
# exit()
# point_cloud_robotbase = robot_transform @ np.vstack([point_cloud_endeffector, np.ones((1, point_cloud_endeffector.shape[1]))])
# point_cloud_robotbase = point_cloud_robotbase[:3, :]
# point_cloud_robotbase = np.reshape(point_cloud_robotbase.T, (480, 640, 3))


# point_cloud1 = point_cloud1 + T

# print(point_cloud)
time.sleep(1)
depth_image, _ = camera.get_frames()
rows, cols = depth_image.shape[:2]
x, y = np.meshgrid(np.arange(cols), np.arange(rows))
x = (x - cx) * depth_image / fx
y = (y - cy) * depth_image / fy

# Calculate the z coordinate of each point in the point cloud
z = depth_image.copy()

# Combine the x, y, and z coordinates into a 3D point cloud
point_cloud2 = np.stack((x, y, z), axis=-1)
print(point_cloud1.shape)
point_cloud3 = np.concatenate((point_cloud_robotbase, point_cloud2), axis=1)
print(point_cloud3.shape)
v = pptk.viewer(point_cloud_endeffector)

