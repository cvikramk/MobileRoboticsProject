# Estimate pose of camera with respect to the laser of the robot using non linear optimization given set of images and laser data

import cv2
import os
import numpy as np
import matplotlib.image as mpimg
# import matplotlib
# import matplotlib.pyplot as plt
# read rosbag package
import rosbag
# import scipy
# import pymanopt
# from pymanopt.manifolds import SpecialOrthogonalGroup
# from pymanopt.manifolds import Eucledian
# from pymanopt.manifolds import Product
# from pymanopt.optimizers import SteepestDescent

# laserPoints
# NormalVector = np.array([1,0,0])

# # Read the images from folder
# def readImages(path):
#     # Read the images from the folder
#     images = []
#     for i in range(1, 11):
#         img = cv2.imread(path + str(i) + '.jpg')
#         images.append(img)
#     return images

# # Read the laser data from rosbag file
# def readLaserData(path):
#     bag = rosbag.Bag(path)
#     laser_data = []
#     for topic, msg, t in bag.read_messages(topics=['/scan']):
#         laser_data.append(msg)
#     return laser_data


# read images and laser scans

# for filename in os.listdir('images'):
#     if filename.endswith('.jpg'):
#         images.append(mpimg.imread('images/' + filename))
#     elif filename.endswith('.txt'):
#         laser_scans.append(np.loadtxt('images/' + filename))


# find the pose of the camera from checkerboard image
# def findCameraPose(image, camera_matrix, dist_coeffs):
#     # Find the camera pose from the checkerboard image
#     # Convert the image to grayscale
#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#     objp = np.zeros((6*7,3), np.float32)
#     objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
#     # Find the corners of the checkerboard
#     ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
#     if ret == True:
#         # Refine the corner locations
#         corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
#         # Find the rotation and translation vectors
#         ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, camera_matrix, dist_coeffs)
#         return np.linalg.inv(rvecs), -tvecs
#     else:
#         return None, None

# # Find the Normal vector to the calibration place from the camera's pose and transform the laser points to camera frame
# def findNormalVector(rvecs, tvecs):
#     # Find the normal vector to the calibration place from the camera's pose
#     NormalVector = -rvecs[:,2]*np.inner(rvecs[:,2],tvecs)
#     return NormalVector

# # define the cost function
# def cost(EstimatedPose):
#     cost = 0
#     for i in range(len(laserPoints)):
#         cost += (NormalVector/scipy.linalg.norm(NormalVector)) * ((scipy.linalg.inv(EstimatedPose[0:3,0:3])@(laserPoints[i] - EstimatedPose[0:3,3])) - scipy.linalg.norm(NormalVector))**2
#     return cost

# def NonLinearOptimization(cost):
#     Manifold = Product(SpecialOrthogonalGroup(3), Eucledian(3))
#     pymanopt.function.autograd(Manifold)

#     problem = pymanopt.Problem(Manifold, cost)

#     optimizer = pymanopt.optimizers.SteepestDescent()
#     result = optimizer.run(problem)

#     return result

# Transform_Laser_Cam = np.array([-1,0,0,0],[0,1,0,0],[0,0,-1,0],[-6,0.5,-8,1])
# project the points in laser to the camera frame using this transformation and plot those transformed points on the camera image data to visualise the laser points in the camera image
# we have to change the laser_data from r,theta to x,y
def projection(r, t, laser_data, image, K):
    for i in laser_data:
        print(i)
        ReorderedLaserData = np.array([i[1], i[2], i[0]])
        points = r@(ReorderedLaserData + t)
        #print(points)
        # converting points in R3 to S2
        points = points/np.linalg.norm(points)
        points = np.append(points, 1)
        Projpoints = K@points
        u = Projpoints[0]/Projpoints[2]
        v = Projpoints[1]/Projpoints[2]
        print(Projpoints)
        image = cv2.circle(image, (int(u), int(v)), radius=1, color=(0,0,255), thickness=2)
    cv2.imshow('image', image)
    cv2.waitKey(0)

if __name__=="__main__":
    image = cv2.imread("images/image_0.jpg")
    image = cv2.rotate(image, cv2.ROTATE_180)
    laser_scan = np.loadtxt("images/scan_0.txt")
    # print(laser_scan)
    # print(laser_scan.shape)
    rotation = np.eye(3)
    translation = np.array([0.08,-0.005, -0.06])
    K = np.array([[499.11014636/4, 0, 316.14098243, 0],[0, 498.6075723/3, 247.3739291, 0],[0,0,1, 0]])
    new_laser_scan = np.empty((1,3))
    for i in range(len(laser_scan)):
        if(laser_scan[i][1] in range(-1.0, 1.0) and laser_scan[i][0] in range(1.3, 2.0)):
            np.vstack((new_laser_scan, [laser_scan[i]]))
    projection(rotation, translation, new_laser_scan, image, K)
    
    # theta = np.arange(0.0, 6.2657318115234375, 0.01745329238474369)
    

# generate the data in x,y,z and feed it to the projection function, plot in the image to see the laser points