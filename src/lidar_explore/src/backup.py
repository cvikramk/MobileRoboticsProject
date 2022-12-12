#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import numpy as np
from math import cos, sin
import tf
from tf.transformations import euler_from_quaternion

current_position = (0,0)
current_cam_map = np.full((384,384),0)
cam_dist = 1.5

# def get_mask(center,theta,resolution,map_size):
#     new_point = (cam_dist*cos(theta),cam_dist*sin(theta))
#     new_point_grid = (new_point[0]/resolution,new_point[1]/resolution)

def sector_mask(shape,centre,radius,angle_range):
    """
    Return a boolean mask for a circular sector. The start/stop angles in  
    `angle_range` should be given in clockwise order.
    """

    x,y = np.ogrid[:shape[0],:shape[1]]
    cx,cy = centre
    tmin,tmax = np.deg2rad(angle_range)

    # ensure stop angle > start angle
    if tmax < tmin:
            tmax += 2*np.pi

    # convert cartesian --> polar coordinates
    r2 = (x-cx)*(x-cx) + (y-cy)*(y-cy)
    theta = np.arctan2(x-cx,y-cy) - tmin

    # wrap angles between 0 and 2*pi
    theta %= (2*np.pi)

    # circular mask
    circmask = r2 <= radius*radius

    # angular mask
    anglemask = theta <= (tmax-tmin)

    return circmask*anglemask  

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # print(data.data.dtype)
    slam_map = np.array(data.data,dtype=int)
    slam_map = np.reshape(slam_map,(384,384))
    new_map = np.copy(current_cam_map)
    # new_map = np.copy(slam_map)
    resolution = data.info.resolution
    # print(data.info)
    # mask = get_mask(center,pose,resolution,(data.info.width,data.info.height))
    t = robot_pose.getLatestCommonTime("/base_link", "/map")
    # position, quaternion = robot_pose.lookupTransform("/map", "/base_link", t) # Get the pose information.
    position, quaternion = robot_pose.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
    (roll, pitch, yaw) = euler_from_quaternion(quaternion)
    print()
    mask = sector_mask(new_map.shape,(int(position[0]/resolution),int(position[1]/resolution)),cam_dist/resolution,(-30,30))
    # print(np.unique(mask))
    # new_map[(0,0)]=200
    new_map[mask]=200
    # new_map[np.where(slam_map)==100]=100
    # print(new_map.ravel().)
    # new_msg = OccupancyGrid(data.header,data.info,new_map.flatten().astype(np.int8).tolist())
    new_msg = OccupancyGrid(data.header,data.info,new_map.ravel().astype(np.int8).tolist())
    pub.publish(new_msg)

def cam_callback(data):
    cam_data = np.array(data.data)
    current_cam_map = np.reshape(cam_data,(384,384))

def listener():

    

    rospy.Subscriber("/map",OccupancyGrid, callback)

    rospy.Subscriber("/camera_map",OccupancyGrid,cam_callback)

    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('camera_map', OccupancyGrid, queue_size=10)
    rospy.init_node('camera_explorer', anonymous=True)
    robot_pose = tf.TransformListener()
    listener()

    