#! /bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2015, Mobile Robotics Project
# Subscribe to the /map topic and publish a modified map superimposed with the Camera FOV mask to ignore the laser data for exploration

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import tf
import numpy as np

class MaskMap:
    def __init__(self):
        # subscribe to the map topic
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        # publish the masked map
        self.masked_map_pub = rospy.Publisher('/masked_map', OccupancyGrid, queue_size=1)
        # tf listener to get the robot pose
        self.tf_listener = tf.TransformListener()


    def map_callback(self, map_msg):
        # get the robot pose
        try:
            rot, trans = self.tf_listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))

            # convert the pose of the robot to occupancy grid coordinates
            map_info = map_msg.info
            map_data = map_msg.data
            map_width = map_info.width
            map_height = map_info.height
            map_resolution = map_info.resolution

            # get the robot pose in the map frame
            #print(map_msg.info.origin)
            # print("map height",map_height)
            # print("map width",map_width)

            # reshape map data into 2D numpy array
            map_data = np.array(map_data).reshape(map_height, map_width)
            #map_data = np.flipud(map_data)
            print(trans[0])
            print(trans[1])
            map_data[int((trans[1] - (map_info.origin.position.y/map_resolution)) + trans[1]), int((trans[0] - (map_info.origin.position.y/map_resolution)) + trans[0])] = 200
            print("map origin",int((trans[1] - (map_info.origin.position.y/map_resolution))), int((trans[0] - (map_info.origin.position.y/map_resolution))))
            #map_data[0,0] = 200
            # convert numpy array to occupancy grid data
            new_msg = OccupancyGrid(map_msg.header,map_info,map_data.ravel().astype(np.int8).tolist())

            # publish the masked map
            self.masked_map_pub.publish(new_msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

if __name__ == '__main__':
    rospy.init_node('MaskMap')
    mask_map = MaskMap()
    rospy.spin()


