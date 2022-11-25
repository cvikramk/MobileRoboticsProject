#! /bin/env python
# Code to detect April tags using apriltag library

import rospy
import apriltag
from tf.transformations import  quaternion_from_euler
import cv2
import numpy as np
import tf
from sensor_msgs.msg import Image
# from sensor_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8, UInt16


class TagDetection:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.callback)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        #self.NumberOfTagsPublisher = rospy.Publisher("/NumberOfTags", numpy_msg(UInt8), queue_size=10)

    def callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # image = cv2.rotate(image,cv2.ROTATE_90_CLOCKWISE)
        # image = cv2.rotate(image,cv2.ROTATE_90_CLOCKWISE)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(gray)

        for i in results:
            # Get the pose of the tag

            #pose, e0, e1 = detector.detection_pose(i, camera_params=(499.11014636, 498.6075723, 316.14098243, 247.3739291), tag_size=0.171)
            pose, e0, e1 = detector.detection_pose(i, camera_params=(-3.313, -3.313, 160.5, 120.5), tag_size=0.25)
            #print("pose of at",pose)
            #pose = pose@[[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]]
            # Draw the bounding box
            cv2.rectangle(image, (int(i.corners[0][0]), int(i.corners[0][1])),(int(i.corners[2][0]),int(i.corners[2][1])), (255, 255, 0), 10)

            # Get roll pitch yaw from rotation matrix
            r = tf.transformations.euler_from_matrix(pose)
            (transOdom, rotOdom) = self.tf_listener.lookupTransform("odom", "base_footprint", rospy.Time(0))
            SE3ForOdom = np.empty((4,4), float)
            SE3ForOdom[0:3, 0:3] = tf.transformations.quaternion_matrix(rotOdom)[0:3, 0:3]
            #print(SE3ForOdom)
            SE3ForOdom[0:3, 3] = transOdom

            # Broadcast the transform
            dist_from_leo = pose[2][3]*100
            cv2.putText(image, "Distance from Leo: " + str(dist_from_leo), (int(i.corners[0][0]), int(i.corners[0][1]-20)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            pose_prime = SE3ForOdom@pose
            r = pose_prime[0:3, 0:3]
            roll = np.arctan2(r[2, 1], r[2, 2])
            pitch = np.arctan2(-r[2, 0], np.sqrt(r[2, 1] ** 2 + r[2, 2] ** 2))
            yaw = np.arctan2(r[1, 0], r[0, 0])
            # Convert to quaternion
            q = quaternion_from_euler(roll, pitch, yaw)
            self.tf_broadcaster.sendTransform(pose_prime[0:3, 3], q, data.header.stamp, "tag1_" + str(i.tag_id), "map")
            pose = pose@SE3ForOdom
            r = pose[0:3, 0:3]
            roll = np.arctan2(r[2, 1], r[2, 2])
            pitch = np.arctan2(-r[2, 0], np.sqrt(r[2, 1] ** 2 + r[2, 2] ** 2))
            yaw = np.arctan2(r[1, 0], r[0, 0])
            # Convert to quaternion
            q = quaternion_from_euler(roll, pitch, yaw)
            # Publish the transformation
            self.tf_broadcaster.sendTransform(pose[0:3, 3], q, data.header.stamp, "tag2_" + str(i.tag_id), "map")
            # append the tag id to the array
        # publish the number of tags found
        #self.NumberOfTagsPublisher.publish(tagIdsFound)
        cv2.imshow("Image window", image)
        # #close window if q is pressed
        cv2.waitKey(1)



if __name__ == '__main__':
    rospy.init_node('tag_detection', anonymous=True)
    td = TagDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()















