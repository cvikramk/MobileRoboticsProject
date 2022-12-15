#! /bin/env python
# Code to publish map from laser scan 
# Creator: Aryaman Patel
# Run this with static publisher : rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map map_laser 10

import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
from math import cos, sin
import tf
from tf.transformations import euler_from_quaternion

# Mapping class that takes in the laser scan data and publishes the map from the laser scan.
class Mapping:

    def __init__(self):
        self.map_laser = OccupancyGrid()
        self.map_laser.info.width = 384
        self.map_laser.info.height = 384
        self.map_laser.info.resolution = 0.05
        self.map_laser.info.origin.position.x = -10
        self.map_laser.info.origin.position.y = -10
        self.map_laser.info.origin.position.z = 0
        self.map_laser.info.origin.orientation.x = 0
        self.map_laser.info.origin.orientation.y = 0
        self.map_laser.info.origin.orientation.z = 0
        self.map_laser.info.origin.orientation.w = 1
        self.map_laser.data = [-1]*384*384
        # Create a 2D array of the map size all initialized to -1
        self.laser_arr = np.ones(shape=(384,384))* -1
        self.robot_pose = tf.TransformListener()
        self.pub = rospy.Publisher('map_laser',OccupancyGrid,queue_size=10)
        self.subscriber = rospy.Subscriber('/scan',LaserScan, self.laser_callback)
        
    
    def get_line(self, x1, y1, x2, y2):
        points = []
        issteep = abs(y2-y1) > abs(x2-x1)
        if issteep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        rev = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            rev = True
        deltax = x2 - x1
        deltay = abs(y2-y1)
        error = int(deltax / 2)
        y = y1
        ystep = None
        if y1 < y2:
            ystep = 1
        else:
            ystep = -1
        for x in range(x1, x2 + 1):
            if issteep:
                points.append((y, x))
            else:
                points.append((x, y))
            error -= deltay
            if error < 0:
                y += ystep
                error += deltax
        if rev:
            points.reverse()
        return points
    
    def laser_callback(self,data):
        # Get the laser scan data
        ranges = np.array(data.ranges)
        # Get the pose of the robot
        t = self.robot_pose.getLatestCommonTime("/base_link", "/map")
        position, quaternion = self.robot_pose.lookupTransform("/map", "/base_link", t) # Get the pose information.
        # Convert the quaternion to euler angles
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        # Get the x and y position of the robot
        x = position[0]
        y = position[1]
        # Get the angle of the robot
        theta = yaw
        # Get the x and y position of the robot in the map
        x_map = int((x - self.map_laser.info.origin.position.x)/self.map_laser.info.resolution)
        y_map = int((y - self.map_laser.info.origin.position.y)/self.map_laser.info.resolution)
        # Get the angle of the robot in the map
        theta_map = int(theta/self.map_laser.info.resolution)
        # Set the free and occupied cells in the map_laser based on the laser scan data
        for i in range(len(ranges)):
            if (i*data.angle_increment + theta) < np.deg2rad(30) or (i*data.angle_increment + theta) > np.deg2rad(330):
                # Get the x and y position of the laser scan data in the map
                if ranges[i] == 0.0:
                    continue
                else:
                    x_laser = int((x + (ranges[i]*3/4)*cos(theta + i*data.angle_increment) - self.map_laser.info.origin.position.x)/self.map_laser.info.resolution)
                    y_laser = int((y + (ranges[i]*3/4)*sin(theta + i*data.angle_increment) - self.map_laser.info.origin.position.y)/self.map_laser.info.resolution)
                    # Set the free cells in the map 
                    free_points = self.get_line(x_map,y_map,x_laser,y_laser)
                    for point in free_points:
                        self.laser_arr[point[0]][point[1]] = 0
                    # Set the occupied cells in the map
                    self.laser_arr[x_laser][y_laser] = 100
            else:
                continue

                # Set the points till the 
        # Conver the map to a 1D array
        self.map_update = OccupancyGrid()
        self.map_update.header.frame_id = "map_laser"
        self.map_update.header.stamp = rospy.Time.now()
        self.map_update.info = self.map_laser.info
        self.map_update.data = self.laser_arr.flatten('F')
        # Convert the map to type int8
        self.map_update.data = self.map_update.data.astype(np.int8)
        # Publish the map
        self.pub.publish(self.map_update)
        rospy.loginfo("Map Published")
                
    def listner(self):
        rospy.spin()
    


if __name__ == "__main__":
    rospy.init_node('map_from_laser',anonymous=True)
    map_laser = Mapping()
    robot_pose = tf.TransformListener()
    map_laser.listner()
