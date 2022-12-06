#! /bin/env python
# code to convert 2d laser scan from topic /scan to point cloud and publish it to topic /laser2PC

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs import point_cloud2
from laser_geometry import LaserProjection

class Laser2PC:
    def __init__(self):
        # initialize node
        rospy.init_node('laser2PC', anonymous=True)

        # initialize publisher
        self.pub = rospy.Publisher('/laser2PC', PointCloud2, queue_size=10)

        # initialize laser projector
        self.lp = LaserProjection()

        # initialize subscriber
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)

    def callback(self, data):
        # convert 2d laser scan to point cloud
        pc = self.lp.projectLaser(data)

        # publish point cloud
        self.pub.publish(pc)

if __name__ == '__main__':
    try:
        laser2PC = Laser2PC()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass