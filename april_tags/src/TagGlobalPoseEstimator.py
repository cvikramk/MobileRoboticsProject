#! /bin/env python
# Code to transform the pose of april tags to map frame

import rospy
import tf
import numpy as np
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8, UInt16
# from std_msgs.msg import String

class TagGlobalPoseEstimator:
    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.NumberOfTagsSubscriber = rospy.Subscriber("/NumberOfTags", numpy_msg(UInt8), self.callback)
        self.NumberOfTags = 0

    def callback(self, data):
        # convert uint8 to int
        self.NumberOfTags = np.int(data.data)
        print("Number of tags: " + str(self.NumberOfTags))
        for i in range(0, int(len(self.NumberOfTags))):
            # try:
            # read the odom to map transform
            # odom = self.tf_listener.waitForTransform("map", "odom", rospy.Time(), rospy.Duration(4.0))
            # look up the transform from odom to map
            (transOdom, rotOdom) = self.tf_listener.lookupTransform("map", "odom", rospy.Time(0))
            (transTag , rotTag) = self.tf_listener.lookupTransform("tag_","base_link", rospy.Time(0))
            # Print the transform
            print((transOdom, rotOdom))
            print((transTag , rotTag))
            # convert the tranformation and rotation from quaternion to SE(3)
            SE3ForOdom = np.empty((4,4), float)
            SE3ForOdom[0:3, 0:3] = tf.transformations.quaternion_matrix(rotOdom)[0:3, 0:3]
            SE3ForOdom[0:3, 3] = transOdom

            SE3ForTag = np.empty((4,4), float)
            SE3ForTag[0:3, 0:3] = tf.transformations.quaternion_matrix(rotTag)[0:3, 0:3]
            SE3ForTag[0:3, 3] = transTag

            # multiply the two SE(3) matrices
            SE3ForMap = SE3ForMap@SE3ForOdom

            # convert the SE(3) matrix to a translation and rotation
            transMap = SE3ForMap[0:3, 3]
            rotMap = tf.transformations.quaternion_from_matrix(SE3ForMap)

            # publish the transform
            self.tf_broadcaster.sendTransform(transMap, rotMap, rospy.Time.now(), "tag", "map")

            # except:
            #     #print("No transform found")
            #     continue

# Main function
if __name__ == '__main__':
    rospy.init_node('Estimate_Pose', anonymous=True)
    try:
        TagGlobalPoseEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass