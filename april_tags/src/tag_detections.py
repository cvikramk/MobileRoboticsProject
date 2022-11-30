#! /bin/env python
# Code to detect April tags using apriltag library

import rospy
#from dt_apriltags import Detector
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
        options = apriltag.DetectorOptions(families='tag36h11',
                            nthreads=1,
                        quad_decimate=1.0,
                        refine_edges=1)

        detector = apriltag.Detector(options)
        results = detector.detect(gray)
        try:
            # at_detector = Detector(families='tag36h11',
            #             nthreads=1,
            #             quad_decimate=1.0,
            #             quad_sigma=0.0,
            #             refine_edges=1,
            #             decode_sharpening=0.25,
            #             debug=0)


            # results = at_detector.detect(gray, True, (-3.313, -3.313, 160.5, 120.5), 0.25)
            # print("Number of tags detected: ", len(results))
            # print("Results: ", results)

            # Get roll pitch yaw from rotation matrix
            (transOdom, rotOdom) = self.tf_listener.lookupTransform("odom", "base_footprint", rospy.Time(0))
            SE3ForOdom = np.eye(4)
            SE3ForOdom[0:3, 0:3] = tf.transformations.quaternion_matrix(rotOdom)[0:3, 0:3]
            SE3ForOdom[0:3, 3] = transOdom
            print("pose of the robot in SE(3)",transOdom)

            for i in results:
                # Get the pose of the tag

                #pose, e0, e1 = detector.detection_pose(i, camera_params=(499.11014636, 498.6075723, 316.14098243, 247.3739291), tag_size=0.171)
                pose, e0, e1 = detector.detection_pose(i, camera_params=(265.233, 265.233, 160.5, 120.5), tag_size=0.25)
                pose_t = pose[0:3, 3]
                #project 3d points to 2d
                ProjectedPose = np.empty((3))
                ProjectedPose = [pose_t[2], -pose_t[0], 0]

                # print("pose", pose, "\n", "pose_t", pose_t)
                print("ProjectedPose", pose_t)
                # OrientationofTag = np.arctan2(pose_t[0],pose_t[2])*180/np.pi
                # print(e0)
                # print(e1)
                # detect the pose of the tag
                # pose_t = i.pose_t.flatten()
                # pose_r = i.pose_R
                # pose_error = i.pose_err
                # print("pose of the tag", pose_r)
                # print("pose of the tag", pose_t)
                # print("pose error", pose_error)

                # pose = np.eye(4)
                # pose[0:3, 0:3] = pose_r
                # pose[0:3, 3] = pose_t
                # print("pose of the tag in SE(3)",pose)

                #print("pose of at",pose)
                #pose = pose@[[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
                # pose[0][3] = -pose[0][3]
                # pose[1][3] = pose[2][3]
                # pose[2][3] = 0
                # r = tf.transformations.euler_from_matrix(pose)
                # r = pose[0:3, 0:3]
                # roll = np.arctan2(r[2, 1], r[2, 2])
                # pitch = np.arctan2(-r[2, 0], np.sqrt(r[2, 1] ** 2 + r[2, 2] ** 2))
                # yaw = np.arctan2(r[1, 0], r[0, 0])
                # Convert to quaternion
                q = quaternion_from_euler(0, 0, 0)
                #transformedProjectedPose = np.dot(np.linalg.inv(pose[0:3,0:3]),ProjectedPose)
                self.tf_broadcaster.sendTransform([pose_t[2], -pose_t[0], 0], q, data.header.stamp, "tagx_" + str(i.tag_id), "base_footprint")
                self.tf_broadcaster.sendTransform(ProjectedPose, q, data.header.stamp, "tagT_" + str(i.tag_id), "base_footprint")
                # Draw the bounding box
                cv2.rectangle(image, (int(i.corners[0][0]), int(i.corners[0][1])),(int(i.corners[2][0]),int(i.corners[2][1])), (255, 255, 0), 10)
                
                HomogeneousPose = np.append(ProjectedPose, 1)
                #print("HomogeneousPose", HomogeneousPose)
                pose_prime = np.dot(SE3ForOdom, HomogeneousPose)
                # pose_prime_final[0] = -pose_prime[2][3] / pose_prime[1][3]
                # pose_prime_final[1] = pose_prime[0][3] / -pose_prime[1][3]
                # pose_prime_final[2] = 0.0
                # pose_prime_final[0] = pose_prime[0][3]
                # pose_prime_final[1] = pose_prime[2][3]
                # pose_prime_final[2] = 0.0
                # r = pose_prime[0:3, 0:3]
                # roll = np.arctan2(r[2, 1], r[2, 2])
                # pitch = np.arctan2(-r[2, 0], np.sqrt(r[2, 1] ** 2 + r[2, 2] ** 2))
                # yaw = np.arctan2(r[1, 0], r[0, 0])
                # Convert to quaternion
                q = quaternion_from_euler(0, 0, 0)
                self.tf_broadcaster.sendTransform(pose_prime[0:3], q, data.header.stamp, "tag1_" + str(i.tag_id), "map")
            #cv2.imshow("Image window", image)
            # #close window if q is pressed
            #cv2.waitKey(1)
            #except segmentation fault
        except Exception as e:
            print(e)
            pass




if __name__ == '__main__':
    rospy.init_node('tag_detection', anonymous=True)
    td = TagDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()















