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

print(f"R: {R}")
print(f"T: {T}")
f = np.vstack([np.hstack([R, T.reshape(3,1)]), [0,0,0,1]])
print(f"f{f}")
print(f"invf = {np.linalg.inv(f)}")


eyeinhand_transform = np.vstack([np.hstack([R.T, -R.T @ T.reshape(3,1)]), [0,0,0,1]])


robot_t1 = np.array(list(robot.get_pos()))
robot_r1 = np.squeeze(list(robot.get_orientation()))
robot_transform = np.vstack([np.hstack([robot_r1, robot_t1.reshape(3, 1)]), [0, 0, 0, 1]])




depth_image, _ = camera.get_frames()
rows, cols = depth_image.shape[:2]
x, y = np.meshgrid(np.arange(cols), np.arange(rows))
x = (x - cx) * depth_image / fx / 1000
y = (y - cy) * depth_image / fy /1000
z = depth_image.copy() /1000

point_cloud1 = np.stack((x, y, z), axis=-1)
T_camera_endeffector = np.vstack([np.hstack([R.T, -R.T @ T.reshape(3, 1)]), [0, 0, 0, 1]])
T_endeffector_robotbase = np.vstack([np.hstack([robot_r1, robot_t1.reshape(3, 1)]), [0, 0, 0, 1]])


pc = []
for i in range(len(point_cloud1)):
    for j in range(len(point_cloud1[i])):
        point = point_cloud1[i][j]
        point_endeffector = np.linalg.inv(T_camera_endeffector) @ np.hstack([point, 1])
        point_endeffector = point_endeffector[:3]

        # Transform the 3D point from the end-effector coordinates to the robot base coordinates
        point_robotbase = T_endeffector_robotbase @ np.hstack([point_endeffector, 1])
        point_robotbase = point_robotbase[:3]
        pc.append(point_robotbase)
        #homogeneous_points = np.hstack((point, np.ones((len(point), 1))))
        # homogeneous_points_world = np.dot(robot_r1, homogeneous_points.T).T

pc = np.array(pc).reshape(480,640,3)

print("next")
time.sleep(5)

#########################

robot_t1 = np.array(list(robot.get_pos()))
robot_r1 = np.squeeze(list(robot.get_orientation()))
robot_transform = np.vstack([np.hstack([robot_r1, robot_t1.reshape(3, 1)]), [0, 0, 0, 1]])

depth_image, _ = camera.get_frames()
rows, cols = depth_image.shape[:2]
x, y = np.meshgrid(np.arange(cols), np.arange(rows))
x = (x - cx) * depth_image / fx 
y = (y - cy) * depth_image / fy 
z = depth_image.copy() 
point_cloud2 = np.stack((x, y, z), axis=-1)
T_camera_endeffector = np.vstack([np.hstack([R.T, -R.T @ T.reshape(3, 1)]), [0, 0, 0, 1]])
T_endeffector_robotbase = np.vstack([np.hstack([robot_r1, robot_t1.reshape(3, 1)]), [0, 0, 0, 1]])


pc2 = []
for i in range(len(point_cloud2)):
    for j in range(len(point_cloud2[i])):
        point = point_cloud2[i][j]
        point_endeffector = np.linalg.inv(T_camera_endeffector) @ np.hstack([point, 1])
        
        point_endeffector = point_endeffector[:3]

        # Transform the 3D point from the end-effector coordinates to the robot base coordinates
        point_robotbase = T_endeffector_robotbase @ np.hstack([point_endeffector, 1])
        point_robotbase = point_robotbase[:3]
        pc2.append(point_robotbase)


pc2 = np.array(pc2).reshape(480,640,3)

pc3 = np.concatenate((pc, pc2), axis=0)

v = pptk.viewer([pc, pc2])

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

