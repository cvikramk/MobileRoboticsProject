#! /usr/env python
# code to extract 50 images and laser scans from the rosbag file and append to numpy array

import rosbag
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import time

# create a folder to save the images
if not os.path.exists('images'):
    os.makedirs('images')

# open the rosbag file
bag = rosbag.Bag('2022-12-03-18-08-20.bag')

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
            print(msg)
            time_diff = abs(t.to_sec() - timestamp)
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                min_time_diff_index = index

    for index, (topic, msg, t) in enumerate(bag.read_messages(topics=['/scan'])):
        if index == min_time_diff_index:
            np.savetxt('images/laser_scan_%s.txt' % str(image_timestamps.index(timestamp)), msg.ranges)
    

# close the rosbag file
bag.close()
            
# plot the laser scan data as bearing and range plot
#scan = np.load('scan/1638573700.0.npy')