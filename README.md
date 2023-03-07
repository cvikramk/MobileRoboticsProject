# Occumapancy Grids from laser scans. 

This is a ROS package for creating a simple Occumapancy Grid map from ``/scan`` data.



# Instruction to use:
You can run this algorithm along with ``TurtleBot3`` ros packages. Look at [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/) for the instructions to 
download the respective dependencies and ros-packages.

For the purpose of this example you will need the:
``turtlebot3-slam`` (using ``gmapping`` side-by-side to compare)
``turtlebot3-navigation`` (for ``/move_base`` and ``/amcl`` node.)
``turtlebot3-simulation`` (house environment). Also install the ``explore_lite`` pakage for 
frontier based exploratio or our environment, instead of using `turtlebot3_teleop` node.

Run the following roslaunch:
```
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```
```
roslaunch turtlebot3_slam turtlebot3_slam.launch 
```
```
roslaunch lidar_explore lidar_mapping.launch
```
Dublicate the `Map` display option in Rviz and subscribe it to `/map_laser` topic to view our auxiliary map. 

```
roslaunch explore_lite explore.launch
```
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch open_rviz:=false
```

# Output:

Out output OccumapancyGrid map will look something like this: 

![Screenshot_20221220_184227](https://user-images.githubusercontent.com/117113574/208751479-804197b4-9e2b-467b-9ffd-76c66b4208f4.png)
