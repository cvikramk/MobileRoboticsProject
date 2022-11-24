#! /bin/env python
# Code to transform the tag pose to the global frame

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import WrenchStamped

class TagGlobalPoseEstimator:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.pose_sub = rospy.Subscriber("/tag_0", PoseStamped, self.callback)
        self.pose_pub = rospy.Publisher("/tag_0_global", PoseStamped, queue_size=10)

    def callback(self, data):
        try:
            # Check how many tag_* topics are published
            topic = rospy.get_published_topics()
            # Get the number of tags
            num_tags = len(topic)
            print("Number of tags: ", num_tags)
            print(topic)

        except:
            print("No tags detected")
        

if __name__ == '__main__':
    rospy.init_node('Estimate_Pose', anonymous=True)
    t = TagGlobalPoseEstimator()
    rospy.spin()