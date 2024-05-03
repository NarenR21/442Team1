# 442Team1

Team Max ROStappen
MFET 442
Naren Ram
Aditya Raj Gupta
Cam Johnson

Assignment.1:

Image in RVIZ with laser scan and dudley map.
![image](https://github.com/NarenR21/442Team1/assets/90937234/00d6544a-9ddf-4275-909b-44098f74aa1f)

Assignment.2:
Waypoint for Dudley in RVIZ:
![image](https://github.com/NarenR21/442Team1/assets/73058520/7541a49b-870c-443b-b205-569a4c53e5d4)

Assignment.3
yaw_error = Theta - desired_theta 
drift_error = arctan((Yd - y) / (Xd- x)) - desired_theta

steering_error = drift_error - yaw_error

Software Description and Explanation:
For this lab we developed several launch files and python files to succesffuly develop the autonomous race car.  The first file that we created was the map_follower launch file.  This is the main launch file that runs the ROS master and initalizes all of the neccessary nodes the car will rely on.  One of the most important launch files that map_follower.launch runs is the car hardware launch file.  This is the launch file that does the startup of the car and publishes the topics from the car to the ROS Master.  Some of these topics include, lidar scans, IMU data, steering velocity, accelerator velocity.
