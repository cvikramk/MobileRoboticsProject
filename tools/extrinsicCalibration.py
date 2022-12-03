# Estimate pose of camera with respect to the laser of the robot using non linear optimization given set of images and laser data

import cv2
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
# read rosbag package
import rosbag

# Read the images from folder
def readImages(path):
    # Read the images from the folder
    images = []
    for i in range(1, 11):
        img = cv2.imread(path + str(i) + '.jpg')
        images.append(img)
    return images

# Read the laser data from rosbag file
def readLaserData(path):
    bag = rosbag.Bag(path)
    laser_data = []
    for topic, msg, t in bag.read_messages(topics=['/scan']):
        laser_data.append(msg)
    return laser_data

# find the pose of the camera from checkerboard image
def findCameraPose(image, camera_matrix, dist_coeffs):
    # Find the camera pose from the checkerboard image
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Find the corners of the checkerboard
    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
    if ret == True:
        # Refine the corner locations
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        # Find the rotation and translation vectors
        ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, camera_matrix, dist_coeffs)
        return np.linalg.inv(rvecs), -tvecs
    else:
        return None, None

# Find the Normal vector to the calibration place from the camera's pose and transform the laser points to camera frame
def findNormalVector(rvecs, tvecs):
    # Find the normal vector to the calibration place from the camera's pose
    NormalVector = -rvecs[:,2]*np.inner(rvecs[:,2],tvecs)
    return NormalVector

# define the cost function
def cost(laserPoints, NormalVector, EstimatedPose):
    cost = 0
    for i in range(len(laserPoints)):
        cost += np.inner(laserPoints[i], NormalVector)
    return cost
