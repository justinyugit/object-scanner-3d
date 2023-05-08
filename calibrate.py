import urx
import numpy as np
import time
import utils
import Camera
import cv2


while True:
    try:
        robot = urx.Robot("172.19.97.157")
        break
    except:
        time.sleep(2)
        continue

def get_calibration_point(intrinsics, color_img, checkboard_size=[9,6], square_size=0.0215,distortion=np.array([0,0,0,0])):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    obj_points = np.zeros((checkboard_size[0] * checkboard_size[1], 3), np.float32)
    obj_points[:, :2] = np.mgrid[0 : checkboard_size[0], 0 : checkboard_size[1]].T.reshape(-1, 2) * square_size

    def detect_corners(img, checkerboard_size, criteria):
        ret, corners = cv2.findChessboardCorners(img, checkerboard_size)
        if ret:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        return corners if ret else None
    # Get the pose of the checkerboard w.r.t the camera
    def get_camera_checkerboard_pose(img, checkerboard_size, camera_matrix, dist_coeffs, obj_points, criteria):
        corners = detect_corners(img, checkerboard_size, criteria)
        if corners is not None:
            print(camera_matrix)
            ret, rvecs, tvecs = cv2.solvePnP(obj_points, corners, camera_matrix, dist_coeffs)
            if ret:
                # pose = np.hstack((rvecs.flatten(), tvecs.flatten()))
                # return pose
                return tvecs, rvecs
        return None

    robot_t = list(robot.get_pos())
    robot_r = np.squeeze(list(robot.get_orientation()))
    camera_t, camera_r = get_camera_checkerboard_pose(
                color_img,
                checkboard_size,
                intrinsics,
                distortion,
                obj_points,
                criteria,
            )
    return robot_t, robot_r, camera_t, camera_r
if __name__ == "__main__":
    camera = Camera.Camera()
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []
    try:
        while True:
            depth_image, color_image = camera.get_frames()

            # Display the images
            # cv2.imshow('Depth Image', depth_image)
            cv2.imshow('Color Image', color_image)


            key = cv2.waitKey(1)
            if (key == ord('q')):
                break
            if (key == ord('c')):
                robot_t, robot_r, camera_t, camera_r = get_calibration_point(camera.get_intrinsic(), np.array(color_image))
                R_gripper2base.append(robot_r)
                t_gripper2base.append(robot_t)
                R_target2cam.append(camera_r)
                t_target2cam.append(camera_t)
                # print(f"robot_t: {robot_t}")
                # print(f"robot_r: {robot_r}")
                # print(f"camera_t: {camera_t}")
                # print(f"camera_r: {camera_r}")

    finally:
        R_gripper2base = np.array(R_gripper2base)
        t_gripper2base = np.array(t_gripper2base)
        R_target2cam = np.array(R_target2cam)
        t_target2cam = np.array(t_target2cam)
        R, T = cv2.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam)
        
        np.save("R_gripper2base.npy", R_gripper2base)
        np.save("t_gripper2base.npy", t_gripper2base)
        np.save("R_target2cam.npy", R_target2cam)
        np.save("t_target2cam.npy", t_target2cam)
        np.save("R.npy", R)
        np.save("T.npy", T)
        camera.stop()
        cv2.destroyAllWindows()
