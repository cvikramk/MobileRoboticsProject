#! /bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pupil_apriltags
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler




class TagDetection:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()


    def callback(self, data):
        # Code to detect April tags
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Display the resulting frame
        detector = pupil_apriltags.Detector(families="tag36h11")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        results = detector.detect(gray, estimate_tag_pose=True, camera_params=(499.11014636, 498.6075723, 316.14098243, 247.3739291), tag_size=0.06)
        for i in results:
            print(i.tag_family, i.tag_id, i.center, i.corners)
            print("Corners ",int(i.corners[0][0]))
            cv_image = cv2.rectangle(cv_image, (int(i.corners[0][0]), int(i.corners[0][1])),(int(i.corners[2][0]),int(i.corners[2][1])), (255, 255, 0), 10)
            cv_image = cv2.putText(cv_image, str(i.tag_id), (int(i.corners[0][0]), int(i.corners[0][1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv_image = cv2.putText(cv_image, str(i.pose_R), (int(i.corners[0][0]), int(i.corners[0][1]) + 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            cv_image = cv2.putText(cv_image, str(i.pose_t), (int(i.corners[0][0]), int(i.corners[0][1]) + 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            # Get roll pitch yaw from rotation matrix
            r = i.pose_R
            roll = np.arctan2(r[2, 1], r[2, 2])
            pitch = np.arctan2(-r[2, 0], np.sqrt(r[2, 1] ** 2 + r[2, 2] ** 2))
            yaw = np.arctan2(r[1, 0], r[0, 0])
            # Convert to quaternion
            q = quaternion_from_euler(roll, pitch, yaw)
            # Publish the transformation
            self.tf_broadcaster.sendTransform(i.pose_t, q, rospy.Time.now(), "tag_" + str(i.tag_id), "map")
        cv2.imshow('frame',cv_image)
        cv2.waitKey(3)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            
        except CvBridgeError as e:
            print(e)
        

        



if __name__ == '__main__':
    rospy.init_node('tag_detection', anonymous=True)
    td = TagDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
