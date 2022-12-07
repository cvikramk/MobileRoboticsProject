#! /usr/env python
# code to extract 50 images and laser scans from the rosbag file and append to numpy array

import rosbag
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import time
from laser_geometry import LaserProjection
# convert PointCloud2 to PointCloud
from sensor_msgs.msg import PointCloud2, PointCloud
from sensor_msgs import point_cloud2
#from sensor_msgs import ConvertPointCloud2ToPointCloud
# plot 3d point cloud
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# create a folder to save the images
if not os.path.exists('images'):
    os.makedirs('images')

# open the rosbag file
bag = rosbag.Bag('/home/anuj/Ext_calib/2022-12-06-20-44-20.bag')

# read 50 random images from the rosbag file and store their timestamps
image_timestamps = []
for topic, msg, t in bag.read_messages(topics=['/camera/image_raw']):
    if topic == '/camera/image_raw':
        cv2.imwrite('images/image_%s.jpg' % str(len(image_timestamps)), np.fromstring(msg.data, np.uint8).reshape(msg.height, msg.width, -1))
        image_timestamps.append(t.to_sec())
        
    if len(image_timestamps) == 50:
        break

# for each timestamp in image_timestamps find the closest laser scan and store the laser scan to text file
for timestamp in image_timestamps:
    min_time_diff = 1
    min_time_diff_index = 0
    for index, (topic, msg, t) in enumerate(bag.read_messages(topics=['/scan'])):
        if topic == '/scan':
            #print(msg)
            time_diff = abs(t.to_sec() - timestamp)
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                min_time_diff_index = index

    for index, (topic, msg, t) in enumerate(bag.read_messages(topics=['/scan'])):
        if index == min_time_diff_index:
            # new_msg = msg.ranges
            # for i in range(25, len(msg.ranges)-25):
            #     new_msg.ranges[i] = 0

            # print(new_msg)
            # ada
            # convert laser scan to point cloud
            lp = LaserProjection()
            
            pc = lp.projectLaser(msg)
            # covert PointCloud2 to PointCloud
            pc = point_cloud2.read_points(pc, skip_nans=True, field_names=("x", "y", "z"))
            pcArray = np.array(list(pc))
            # plot the point cloud
            ax.scatter(pcArray[:,0], pcArray[:,1], pcArray[:,2], marker='o')
            ax.set_xlabel('X Label')
            ax.set_ylabel('Y Label')
            ax.set_zlabel('Z Label')
            # print the pc as numpy array
            # save point cloud to text file
            np.savetxt('images/scan_%s.txt' % str(image_timestamps.index(timestamp)), pcArray)
            np.savetxt('images/laser_scan_%s.txt' % str(image_timestamps.index(timestamp)), msg)
plt.show()


# close the rosbag file
bag.close()
            
# plot the laser scan data as bearing and range plot
#scan = np.load('scan/1638573700.0.npy')