# Mobile Robotics Project: Team Leonardo

Please refer to the instructions below step by step in order to test our code:
1. SSH into the raspberry pi terminal 
2. Bringup the turtlebot using the turtlebot3 bringup package and launch the robot.launch file
3. Launch the camera node in turtlebot3 using the rpicam.launch under turtlebot3 bringup package
4. In your terminal run the launch file for gmapping from the turtlebot3 slam package
5. In your terminal run the launch file for frontier exploration from the m-explore package
6. In your terminal run the launch file for april tags detections by running the tag_detection.launch from the apriltag_ros pacakge
7. In your terminal run the launch file for camera based mapping by running the Mapping_from_laser.launch from the lidar_explore package
