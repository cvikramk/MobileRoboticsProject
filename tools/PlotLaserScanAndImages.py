#! /usr/env python
# read images and laser scan and plot image and corresponding laser scan in polar plot

import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np


# read images and laser scans
images = []
laser_scans = []
for filename in os.listdir('images'):
    if filename.endswith('.jpg'):
        images.append(mpimg.imread('images/' + filename))
    elif filename.endswith('.txt'):
        laser_scans.append(np.loadtxt('images/' + filename))

# plot images and corresponding laser scans
for index, image in enumerate(images):
    plt.figure()
    plt.subplot(1, 2, 1)
    plt.imshow(np.flip(image,0))
    plt.subplot(1, 2, 2)
    # space the plots evenly
    plt.subplots_adjust(wspace=5.5)
    print(len(laser_scans[index]))
    plt.polar(np.arange(0, 2*np.pi, np.pi/180), laser_scans[index], 'b.')
    plt.show()

# Path: MobileRoboticsProject/tools/PlotLaserScanAndImages.py